/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   FreeRTOS application: ADC / FFT / UART tasks
  ******************************************************************************
  * 단일 while 루프로 동작하던 FFT 분석기를 3개의 태스크로 분리.
  *
  *   ADC ConvCplt(TIM3 트리거) --(sampleTickSem)--> [AdcTask] --(fftQueue)--> [FftTask]
  *                                                              |
  *                                                       (uartReadySem)
  *                                                              v
  *                                                          [UartTask]
  *
  *   공유 자원 보호:
  *     - lcdMutex  : LCD(FSMC) 버스는 AdcTask(시간영역)와 FftTask(FFT)가 공유
  *     - dataMutex : FFT 결과(fftMag)와 시간영역 스냅샷(rawBytes)을
  *                   FftTask(쓰기)와 UartTask(읽기)가 공유
  *
  *   버퍼 경쟁 방지:
  *     - adcBuf[2][] 핑퐁 버퍼. AdcTask가 한쪽을 채우는 동안 FftTask는 반대쪽을 처리.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_tasks.h"
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "tim.h"
#include "openx07v_c_lcd.h"
#include "filters.h"
#include "arm_math.h"
#include <string.h>
#include <math.h>       /* log10f (dB 스케일) */

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private defines -----------------------------------------------------------*/
#define FFT_SAMPLES     512                 /* 입력 버퍼 크기(실수+허수 인터리브) */
#define FFT_POINTS      (FFT_SAMPLES / 2)   /* FFT 포인트 수 = 256              */

/* 시간영역 3분할 표시 레이아웃: RAW / FIR / IIR 을 각자의 밴드에 그린다.
 * 값 0..63 을 각 밴드 64px 에 매핑(값↑ = 위쪽). 좌측 X0 이전은 라벨 영역(파형이 안 침범). */
#define TD_X0        28                           /* 파형 시작 x (좌측 라벨 보존) */
#define TD_BAND      64                           /* 밴드 높이(값 0..63) */
#define TD_GAP       8                            /* 밴드 사이 간격 */
#define RAW_TOP      22                           /* RAW 밴드 상단 y (헤더 아래) */
#define FIR_TOP      (RAW_TOP + TD_BAND + TD_GAP) /* = 94  */
#define IIR_TOP      (FIR_TOP + TD_BAND + TD_GAP) /* = 166 */
#define TD_BOTTOM    (IIR_TOP + TD_BAND)          /* = 230 (< 239) */

/* 태스크 우선순위 (높을수록 우선)
 * UART 를 FFT 보다 높게 둔다: UartTask 는 IT(인터럽트) 전송을 "시작"만 하고
 * 곧바로 완료 세마포어에서 잠들기 때문에 CPU 를 거의 쓰지 않는다.
 * 따라서 높은 우선순위라도 FftTask 를 굶기지 않으며, 반대로 무거운 FftTask
 * 가 UartTask 를 굶기는 문제(전송 멈춤)를 막는다. */
#define PRIO_ADC        (tskIDLE_PRIORITY + 3)   /* 최고: 샘플 누락 방지 */
#define PRIO_UART       (tskIDLE_PRIORITY + 2)   /* 중간: 전송 시작 후 잠듦 */
#define PRIO_FFT        (tskIDLE_PRIORITY + 1)   /* 최저: 남는 시간에 연산/그리기 */

/* 태스크 스택 크기(워드 단위). 큰 배열은 static 으로 빼서 스택 부담을 줄임 */
#define STACK_ADC       256
#define STACK_FFT       512
#define STACK_UART      512

/* UART 프레임 헤더/타입.  type = [신호][도메인]
 *   신호  : RAW(원본) / FIR / IIR
 *   도메인: RAW=시간영역,  FFT=주파수영역 크기 */
#define FRAME_SOF0      0x03
#define FRAME_SOF1      0x15
#define FRAME_TYPE_RAW      0x01    /* 원본 시간영역  */
#define FRAME_TYPE_FFT      0x02    /* 원본 FFT       */
#define FRAME_TYPE_FIR_RAW  0x03    /* FIR  시간영역  */
#define FRAME_TYPE_FIR_FFT  0x04    /* FIR  FFT       */
#define FRAME_TYPE_IIR_RAW  0x05    /* IIR  시간영역  */
#define FRAME_TYPE_IIR_FFT  0x06    /* IIR  FFT       */

/* External variables --------------------------------------------------------*/
extern uint8_t AdcVal;          /* main.c: ADC1 DMA 가 갱신하는 최신 샘플(8bit) */

/* Private variables ---------------------------------------------------------*/
/* 동기화 객체 */
static SemaphoreHandle_t sampleTickSem;     /* ADC ConvCplt ISR -> AdcTask  (counting) */
static QueueHandle_t     fftQueue;          /* AdcTask  -> FftTask  (버퍼 인덱스) */
static SemaphoreHandle_t uartReadySem;      /* FftTask  -> UartTask (binary)   */
static SemaphoreHandle_t uartTxDoneSem;     /* USART3 TX 완료 ISR -> UartTask (binary) */
static SemaphoreHandle_t lcdMutex;          /* LCD 공유 보호 */
static SemaphoreHandle_t dataMutex;         /* fftMag / rawBytes 공유 보호 */

/* 데이터 버퍼 (핑퐁: AdcTask 가 한쪽을 채우는 동안 FftTask 는 반대쪽 처리)
 * 3개 신호(원본/FIR/IIR)를 각각 복소 인터리브로 저장한다(허수부=0). */
static float32_t adcBuf[2][FFT_SAMPLES];    /* 원본 */
static float32_t firBuf[2][FFT_SAMPLES];    /* FIR 출력 */
static float32_t iirBuf[2][FFT_SAMPLES];    /* IIR 출력 */

/* 공유 결과 (dataMutex 보호): FftTask 가 쓰고 UartTask 가 읽음 */
static float32_t fftMag[FFT_POINTS];        /* 원본 FFT 크기 */
static float32_t firMag[FFT_POINTS];        /* FIR  FFT 크기 */
static float32_t iirMag[FFT_POINTS];        /* IIR  FFT 크기 */
static uint8_t   rawBytes[FFT_POINTS];      /* 원본 시간영역 바이트 */
static uint8_t   firBytes[FFT_POINTS];      /* FIR  시간영역 바이트 */
static uint8_t   iirBytes[FFT_POINTS];      /* IIR  시간영역 바이트 */
static float32_t fftWin[FFT_POINTS];        /* FFT 누설 저감용 Hann 창 (FftTask 가 1회 계산) */

/* Task handles */
static TaskHandle_t hAdcTask;
static TaskHandle_t hFftTask;
static TaskHandle_t hUartTask;

/* Private function prototypes -----------------------------------------------*/
static void AdcTask(void *argument);
static void FftTask(void *argument);
static void UartTask(void *argument);
static uint16_t Build_TimeFrame(uint8_t *out, uint8_t type, const uint8_t *samples, uint16_t n);
static uint16_t Build_FftFrame(uint8_t *out, uint8_t type, const float32_t *mag, uint16_t n);
static void     Uart_Send(const uint8_t *buf, uint16_t len);

/* ===========================================================================
 *  ISR hook
 * ===========================================================================*/
/**
  * @brief  ADC 변환완료 콜백 (DMA2_Stream0 IRQ 컨텍스트).
  *         TIM3 TRGO 가 트리거한 변환이 끝날 때마다(= 균일한 샘플 시점)
  *         AdcTask 를 깨운다. 하드웨어 트리거라 샘플링 시점 지터가 없다.
  *         DMA2_Stream0 IRQ 우선순위(5)는 configMAX_SYSCALL_INTERRUPT_PRIORITY(5)
  *         이하라서 FromISR API 호출이 안전하다.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t higherPrioWoken = pdFALSE;

    /* 세마포어가 아직 생성되기 전(스케줄러 시작 전 짧은 구간)에는 무시 */
    if (hadc->Instance != ADC1 || sampleTickSem == NULL)
        return;

    xSemaphoreGiveFromISR(sampleTickSem, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
}

/**
  * @brief  UART 전송 완료 콜백 (USART3 IRQ 컨텍스트에서 호출됨).
  *         HAL_UART_Transmit_IT 한 프레임 전송이 끝나면 UartTask 를 깨운다.
  *         USART3 IRQ 우선순위(5)는 configMAX_SYSCALL_INTERRUPT_PRIORITY(5)
  *         이하라서 FromISR API 호출이 안전하다.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3 && uartTxDoneSem != NULL)
    {
        BaseType_t higherPrioWoken = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &higherPrioWoken);
        portYIELD_FROM_ISR(higherPrioWoken);
    }
}

/* ===========================================================================
 *  AdcTask : 샘플 수집 + 시간영역 LCD 표시
 * ===========================================================================*/
/* 값(공칭 0..63)을 지정 밴드의 y 좌표로 매핑(값↑ = 위쪽, 밴드 밖은 클램프). */
static uint16_t MapBand(float32_t val, uint16_t top)
{
    int iv = (int)(val + 0.5f);
    if (iv < 0)  iv = 0;
    if (iv > 63) iv = 63;
    return (uint16_t)(top + (63 - iv));
}

static void AdcTask(void *argument)
{
    uint16_t idx = 0;       /* 복소 샘플 인덱스 0..FFT_POINTS-1 */
    uint8_t  cur = 0;       /* 현재 채우는 핑퐁 버퍼 */
    uint16_t x   = TD_X0;   /* 시간영역 그래프 X 좌표 (좌측 라벨 영역 이후부터) */
    uint8_t  drawDiv = 0;   /* 표시 데시메이션 (3샘플당 1회 그림: 3밴드 지우기 부하↓) */

    (void)argument;

    /* 밴드 라벨 1회 표시 (좌측, 파형 영역 밖이라 지워지지 않음) */
    if (xSemaphoreTake(lcdMutex, portMAX_DELAY) == pdTRUE)
    {
        BSP_LCD_SetFont(&Font12);
        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
        BSP_LCD_DisplayStringAt(2, RAW_TOP, (uint8_t *)"RAW", LEFT_MODE);
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAt(2, FIR_TOP, (uint8_t *)"FIR", LEFT_MODE);
        BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
        BSP_LCD_DisplayStringAt(2, IIR_TOP, (uint8_t *)"IIR", LEFT_MODE);
        xSemaphoreGive(lcdMutex);
    }

    for (;;)
    {
        /* TIM3->ADC 가 페이싱하는 샘플링 틱을 대기 (약 8.84 kHz) */
        if (xSemaphoreTake(sampleTickSem, portMAX_DELAY) != pdTRUE)
            continue;

        /* --- 1) 샘플 저장 (실수부 = ADC, 허수부 = 0) : 항상 수행 --- */
        uint8_t v = (uint8_t)(AdcVal / 4);
        adcBuf[cur][idx * 2]     = (float32_t)v;
        adcBuf[cur][idx * 2 + 1] = 0.0f;

        /* --- 2) FIR / IIR 저역통과 처리 : 매 샘플 호출(필터 상태 유지).
         *        결과를 FFT 용 핑퐁 버퍼에도 저장(허수부=0)해 FftTask 가 스펙트럼을 낸다. --- */
        float32_t fFir = Filter_FIR_LPF((float32_t)v);
        float32_t fIir = Filter_IIR_LPF((float32_t)v);
        /* 필터 출력을 FFT 용 버퍼에도 저장(허수부=0) — UART FFT 프레임용 */
        firBuf[cur][idx * 2]     = fFir;
        firBuf[cur][idx * 2 + 1] = 0.0f;
        iirBuf[cur][idx * 2]     = fIir;
        iirBuf[cur][idx * 2 + 1] = 0.0f;

        /* --- 3) 시간영역 LCD 표시 : 2샘플당 1회만 그린다.
         *        최고 우선순위인 AdcTask 가 매 샘플 무거운 LCD 그리기를 하면
         *        샘플 틱 세마포어가 비워지지 않아, 최저 우선순위 FftTask 가
         *        CPU 를 못 받아(FFT 표시가 멈춤). 그리기를 줄여 CPU 를 양보한다.
         *        (필터는 위에서 매 샘플 처리하므로 필터링 정확도엔 영향 없음.)
         *        원본(노랑) 위에 FIR(초록)/IIR(시안) 을 겹쳐 비교 표시.
         *        FIR 은 선형위상이라 약 (N-1)/2 = 15 샘플 지연되어 보인다. --- */
        if (++drawDiv >= 3)
        {
            drawDiv = 0;
            if (xSemaphoreTake(lcdMutex, 0) == pdTRUE)   /* LCD 점유 중이면 건너뜀 */
            {
                /* 다음 컬럼(세 밴드 전체)을 미리 지운다 */
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_DrawVLine(x + 1, RAW_TOP, TD_BOTTOM - RAW_TOP);

                /* 각 신호를 자기 밴드에 점으로: RAW=노랑 / FIR=초록 / IIR=시안 */
                BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                BSP_LCD_DrawVLine(x, MapBand((float32_t)v, RAW_TOP), 1);
                BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
                BSP_LCD_DrawVLine(x, MapBand(fFir, FIR_TOP), 1);
                BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
                BSP_LCD_DrawVLine(x, MapBand(fIir, IIR_TOP), 1);

                xSemaphoreGive(lcdMutex);
            }
            if (++x >= 320) x = TD_X0;
        }

        /* --- 4) 한 블록(256 샘플)이 차면 FftTask 로 넘기고 버퍼 교체 --- */
        if (++idx >= FFT_POINTS)
        {
            idx = 0;
            xQueueSend(fftQueue, &cur, 0);  /* 채워진 버퍼 인덱스를 전달 */
            cur ^= 1;                       /* 다음 블록은 반대쪽 버퍼에 */
        }
    }
}

/* float 샘플을 0..255 바이트로 클램프(필터 링잉으로 음수/초과가 생길 수 있음). */
static uint8_t Clamp8(float32_t v)
{
    int iv = (int)(v + 0.5f);
    if (iv < 0)   iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

/* 한 신호 블록을 DC 제거 -> in-place 복소 FFT -> 크기 로 변환한다.
 * (입력 buf 는 덮어써지므로, 시간영역 스냅샷은 호출 전에 떠 두어야 한다.) */
static void Fft_Compute(arm_cfft_radix4_instance_f32 *S, float32_t *buf, float32_t *magOut)
{
    uint16_t i;
    float32_t mean = 0.0f;

    for (i = 0; i < FFT_POINTS; i++)
        mean += buf[i * 2];
    mean /= (float32_t)FFT_POINTS;
    /* DC 제거 후 Hann 창 적용(누설 저감).
     * 창 = 2*0.5*(1-cos) = (1-cos): 평균 이득 1 이라 피크 크기는 거의 보존된다. */
    for (i = 0; i < FFT_POINTS; i++)
        buf[i * 2] = (buf[i * 2] - mean) * fftWin[i];   /* 허수부(buf[i*2+1]) 는 0 유지 */

    arm_cfft_radix4_f32(S, buf);
    arm_cmplx_mag_f32(buf, magOut, FFT_POINTS);
}

/* ===========================================================================
 *  FftTask : 원본/FIR/IIR 각각 FFT (UART 전송용). LCD 표시는 하지 않음.
 * ===========================================================================*/
static void FftTask(void *argument)
{
    arm_cfft_radix4_instance_f32 S;
    static float32_t magR[FFT_POINTS], magF[FFT_POINTS], magI[FFT_POINTS]; /* static: 스택 절약 */
    uint8_t bufIdx;
    uint16_t i;

    (void)argument;

    arm_cfft_radix4_init_f32(&S, FFT_POINTS, 0, 1);

    /* Hann 창 1회 계산. 사각창(창 없음)이면 강한 저주파 에너지가 전 대역으로 퍼져
     * 노이즈 플로어/오프셋처럼 보인다(저역통과 신호에서 특히 두드러짐). */
    for (i = 0; i < FFT_POINTS; i++)
        fftWin[i] = 1.0f - cosf(2.0f * PI * (float32_t)i / (float32_t)(FFT_POINTS - 1));

    for (;;)
    {
        /* AdcTask 가 채운 블록 버퍼 인덱스를 수신 */
        if (xQueueReceive(fftQueue, &bufIdx, portMAX_DELAY) != pdTRUE)
            continue;

        /* --- 1) in-place FFT 가 입력을 덮어쓰기 전에 3개 신호의 시간영역 스냅샷 --- */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            for (i = 0; i < FFT_POINTS; i++)
            {
                rawBytes[i] = Clamp8(adcBuf[bufIdx][i * 2]);
                firBytes[i] = Clamp8(firBuf[bufIdx][i * 2]);
                iirBytes[i] = Clamp8(iirBuf[bufIdx][i * 2]);
            }
            xSemaphoreGive(dataMutex);
        }

        /* --- 2) 원본 / FIR / IIR 각각 DC제거 + FFT + 크기 --- */
        Fft_Compute(&S, adcBuf[bufIdx], magR);
        Fft_Compute(&S, firBuf[bufIdx], magF);
        Fft_Compute(&S, iirBuf[bufIdx], magI);

        /* --- 3) 결과 공개 (UartTask 가 읽음) --- */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(fftMag, magR, sizeof(fftMag));
            memcpy(firMag, magF, sizeof(firMag));
            memcpy(iirMag, magI, sizeof(iirMag));
            xSemaphoreGive(dataMutex);
        }

        /* --- 4) UartTask 에 새 데이터 도착 통지 (LCD 표시는 AdcTask 의 시간영역만) --- */
        xSemaphoreGive(uartReadySem);
    }
}

/* ===========================================================================
 *  UartTask : 프레임 인코딩 + 전송 (블로킹, 최저 우선순위)
 * ===========================================================================*/
/* 한 프레임을 IT(비차단)로 전송 시작하고, 완료 세마포어에서 잠들어 대기한다.
 * 전송 중 CPU 는 FftTask 등 다른 태스크가 사용한다(자체 페이싱). */
static void Uart_Send(const uint8_t *buf, uint16_t len)
{
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, len) == HAL_OK)
        xSemaphoreTake(uartTxDoneSem, portMAX_DELAY);
}

static void UartTask(void *argument)
{
    /* 전송 버퍼(시간영역/ FFT 각 1개를 6프레임이 순차 재사용 — Uart_Send 가 완료를
     * 기다린 뒤 반환하므로 다음 Build 가 덮어써도 안전) + 로컬 복사본 (static) */
    static uint8_t   txTime[FFT_POINTS + 5];        /* 헤더5 + 256  */
    static uint8_t   txFft [FFT_POINTS * 4 + 5];    /* 헤더5 + 1024 */
    static uint8_t   locRaw[FFT_POINTS], locFir[FFT_POINTS], locIir[FFT_POINTS];
    static float32_t locMagR[FFT_POINTS], locMagF[FFT_POINTS], locMagI[FFT_POINTS];
    uint16_t len;

    (void)argument;

    for (;;)
    {
        /* FftTask 의 통지 대기 */
        if (xSemaphoreTake(uartReadySem, portMAX_DELAY) != pdTRUE)
            continue;

        /* 공유 데이터를 짧게 잠그고 6종 모두 로컬 복사 후, 전송은 잠금 없이 수행 */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(locRaw,  rawBytes, sizeof(locRaw));
            memcpy(locFir,  firBytes, sizeof(locFir));
            memcpy(locIir,  iirBytes, sizeof(locIir));
            memcpy(locMagR, fftMag,   sizeof(locMagR));
            memcpy(locMagF, firMag,   sizeof(locMagF));
            memcpy(locMagI, iirMag,   sizeof(locMagI));
            xSemaphoreGive(dataMutex);
        }

        /* 시간영역 3종 */
        len = Build_TimeFrame(txTime, FRAME_TYPE_RAW,     locRaw, FFT_POINTS);  Uart_Send(txTime, len);
        len = Build_TimeFrame(txTime, FRAME_TYPE_FIR_RAW, locFir, FFT_POINTS);  Uart_Send(txTime, len);
        len = Build_TimeFrame(txTime, FRAME_TYPE_IIR_RAW, locIir, FFT_POINTS);  Uart_Send(txTime, len);

        /* 주파수영역(FFT) 3종 */
        len = Build_FftFrame(txFft, FRAME_TYPE_FFT,     locMagR, FFT_POINTS);   Uart_Send(txFft, len);
        len = Build_FftFrame(txFft, FRAME_TYPE_FIR_FFT, locMagF, FFT_POINTS);   Uart_Send(txFft, len);
        len = Build_FftFrame(txFft, FRAME_TYPE_IIR_FFT, locMagI, FFT_POINTS);   Uart_Send(txFft, len);
    }
}

/* ===========================================================================
 *  프레임 인코딩 헬퍼
 * ===========================================================================*/
/* 시간영역 프레임: [0x03][0x15][type][len_hi][len_lo][samples...]  (len = n 바이트) */
static uint16_t Build_TimeFrame(uint8_t *out, uint8_t type, const uint8_t *samples, uint16_t n)
{
    out[0] = FRAME_SOF0;
    out[1] = FRAME_SOF1;
    out[2] = type;
    out[3] = (uint8_t)((n >> 8) & 0xFF);
    out[4] = (uint8_t)(n & 0xFF);
    memcpy(&out[5], samples, n);
    return (uint16_t)(n + 5);
}

/* FFT 프레임: [0x03][0x15][type][len_hi][len_lo][float32 LE ...]  (len = n*4 바이트) */
static uint16_t Build_FftFrame(uint8_t *out, uint8_t type, const float32_t *mag, uint16_t n)
{
    uint16_t payload = (uint16_t)(n * 4);
    out[0] = FRAME_SOF0;
    out[1] = FRAME_SOF1;
    out[2] = type;
    out[3] = (uint8_t)((payload >> 8) & 0xFF);
    out[4] = (uint8_t)(payload & 0xFF);
    memcpy(&out[5], mag, payload);  /* float32 little-endian 그대로 복사 */
    return (uint16_t)(payload + 5);
}

/* ===========================================================================
 *  초기화
 * ===========================================================================*/
void App_FreeRTOS_Init(void)
{
    /* --- FIR/IIR 필터 계수 설계 + 상태 초기화 (스케줄러 시작 전) --- */
    Filters_Init();

    /* --- 동기화 객체 생성 --- */
    sampleTickSem = xSemaphoreCreateCounting(16, 0);   /* 틱 누락 방지용 카운팅 */
    uartReadySem  = xSemaphoreCreateBinary();
    uartTxDoneSem = xSemaphoreCreateBinary();          /* UART TX 완료 통지 */
    lcdMutex      = xSemaphoreCreateMutex();           /* 우선순위 상속 적용 */
    dataMutex     = xSemaphoreCreateMutex();
    fftQueue      = xQueueCreate(2, sizeof(uint8_t));  /* 핑퐁 버퍼 인덱스 */

    configASSERT(sampleTickSem && uartReadySem && uartTxDoneSem && lcdMutex && dataMutex && fftQueue);

    /* --- 태스크 생성 --- */
    xTaskCreate(AdcTask,  "Adc",  STACK_ADC,  NULL, PRIO_ADC,  &hAdcTask);
    xTaskCreate(FftTask,  "Fft",  STACK_FFT,  NULL, PRIO_FFT,  &hFftTask);
    xTaskCreate(UartTask, "Uart", STACK_UART, NULL, PRIO_UART, &hUartTask);
}
