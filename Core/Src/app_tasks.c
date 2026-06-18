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

/* FFT 막대 그래프 표시 파라미터 (로그/dB 스케일) */
#define FFT_BASE_Y      239      /* 막대 바닥 y 좌표 */
#define FFT_MAX_HEIGHT  200      /* 막대 최대 높이(px) */
#define FFT_DB_MIN      45.0f    /* 이 dB 이하 = 막대 없음(바닥). 노이즈 플로어를 가리도록 올림(30->45) */
#define FFT_DB_MAX      90.0f    /* 이 dB 이상 = 최대 높이로 클램프 */

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

/* UART 프레임 헤더/타입 */
#define FRAME_SOF0      0x03
#define FRAME_SOF1      0x15
#define FRAME_TYPE_RAW  0x01    /* 시간영역(Raw) */
#define FRAME_TYPE_FFT  0x02    /* 주파수영역(FFT) */

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

/* 데이터 버퍼 */
static float32_t adcBuf[2][FFT_SAMPLES];    /* 핑퐁: AdcTask 가 채움 */
static float32_t fftMag[FFT_POINTS];        /* FFT 크기 결과 (dataMutex 보호) */
static uint8_t   rawBytes[FFT_POINTS];      /* 시간영역 샘플 바이트 (dataMutex 보호) */

/* Task handles */
static TaskHandle_t hAdcTask;
static TaskHandle_t hFftTask;
static TaskHandle_t hUartTask;

/* Private function prototypes -----------------------------------------------*/
static void AdcTask(void *argument);
static void FftTask(void *argument);
static void UartTask(void *argument);
static uint16_t Build_RawFrame(uint8_t *out, const uint8_t *samples, uint16_t n);
static uint16_t Build_FftFrame(uint8_t *out, const float32_t *mag, uint16_t n);

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
static void AdcTask(void *argument)
{
    uint16_t idx = 0;       /* 복소 샘플 인덱스 0..FFT_POINTS-1 */
    uint8_t  cur = 0;       /* 현재 채우는 핑퐁 버퍼 */
    uint16_t x   = 0;       /* 시간영역 그래프 X 좌표 */

    (void)argument;

    for (;;)
    {
        /* TIM3->ADC 가 페이싱하는 샘플링 틱을 대기 (약 8.84 kHz) */
        if (xSemaphoreTake(sampleTickSem, portMAX_DELAY) != pdTRUE)
            continue;

        /* --- 1) 샘플 저장 (실수부 = ADC, 허수부 = 0) : 항상 수행 --- */
        uint8_t v = (uint8_t)(AdcVal / 4);
        adcBuf[cur][idx * 2]     = (float32_t)v;
        adcBuf[cur][idx * 2 + 1] = 0.0f;

        /* --- 2) 시간영역 LCD 표시 (LCD 가 점유 중이면 건너뜀: 샘플링은 절대 막지 않음) --- */
        if (xSemaphoreTake(lcdMutex, 0) == pdTRUE)
        {
            BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
            BSP_LCD_FillCircle(x, v + 40, 1);
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            BSP_LCD_DrawVLine(x + 1, 35, 70);
            xSemaphoreGive(lcdMutex);
        }
        if (++x >= 320) x = 0;

        /* --- 3) 한 블록(256 샘플)이 차면 FftTask 로 넘기고 버퍼 교체 --- */
        if (++idx >= FFT_POINTS)
        {
            idx = 0;
            xQueueSend(fftQueue, &cur, 0);  /* 채워진 버퍼 인덱스를 전달 */
            cur ^= 1;                       /* 다음 블록은 반대쪽 버퍼에 */
        }
    }
}

/* magnitude 를 로그(dB) 스케일로 막대 끝(top) y 좌표로 변환.
 * [FFT_DB_MIN, FFT_DB_MAX] dB 구간을 [0, FFT_MAX_HEIGHT] px 에 선형 매핑하고 클램프한다.
 * (DrawLine 의 y 인자는 uint16_t 라서 클램프로 화면 밖 출력을 막는다.) */
static uint16_t Fft_BarTopY(float32_t magnitude)
{
    float32_t db = 20.0f * log10f(magnitude + 1.0f);   /* +1: log(0) 방지 */
    int h = (int)((db - FFT_DB_MIN) * ((float32_t)FFT_MAX_HEIGHT / (FFT_DB_MAX - FFT_DB_MIN)));
    if (h < 0)              h = 0;
    if (h > FFT_MAX_HEIGHT) h = FFT_MAX_HEIGHT;
    return (uint16_t)(FFT_BASE_Y - h);
}

/* ===========================================================================
 *  FftTask : FFT 변환 + magnitude + FFT LCD 표시
 * ===========================================================================*/
static void FftTask(void *argument)
{
    arm_cfft_radix4_instance_f32 S;
    static float32_t mag[FFT_POINTS];       /* 연산용 임시(스택 절약 위해 static) */
    static float32_t prevMag[FFT_POINTS];   /* 직전 표시값 = LCD 지우기 기준 */
    uint8_t bufIdx;
    uint16_t i;

    (void)argument;

    /* 파라미터가 고정이므로 초기화는 한 번만 (기존 코드는 매 사이클 호출했음) */
    arm_cfft_radix4_init_f32(&S, FFT_POINTS, 0, 1);

    for (;;)
    {
        /* AdcTask 가 채운 블록 버퍼 인덱스를 수신 */
        if (xQueueReceive(fftQueue, &bufIdx, portMAX_DELAY) != pdTRUE)
            continue;

        float32_t *in = adcBuf[bufIdx];

        /* --- 1) in-place FFT 가 Input 을 덮어쓰기 전에 시간영역(실수부) 스냅샷 저장 --- */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            for (i = 0; i < FFT_POINTS; i++)
                rawBytes[i] = (uint8_t)in[i * 2];
            xSemaphoreGive(dataMutex);
        }

        /* --- 2) DC 오프셋 제거 (실수부에만; 허수부는 0 유지) --- */
        float32_t mean = 0.0f;
        for (i = 0; i < FFT_POINTS; i++)
            mean += in[i * 2];
        mean /= (float32_t)FFT_POINTS;
        for (i = 0; i < FFT_POINTS; i++)
            in[i * 2] = in[i * 2] - mean;   /* in[i*2+1] 은 0 그대로 */

        /* --- 3) 복소 FFT (in-place) + 크기 계산 --- */
        arm_cfft_radix4_f32(&S, in);
        arm_cmplx_mag_f32(in, mag, FFT_POINTS);

        /* --- 4) 결과 공개 (UartTask 가 읽음) --- */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(fftMag, mag, sizeof(fftMag));
            xSemaphoreGive(dataMutex);
        }

        /* --- 5) FFT LCD 표시 (이전 막대 지우고 새로 그림) --- */
        if (xSemaphoreTake(lcdMutex, portMAX_DELAY) == pdTRUE)
        {
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            for (i = 2; i < (FFT_POINTS / 2); i++)
                BSP_LCD_DrawLine(i, FFT_BASE_Y, i, Fft_BarTopY(prevMag[i]));

            BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
            for (i = 2; i < (FFT_POINTS / 2); i++)
                BSP_LCD_DrawLine(i, FFT_BASE_Y, i, Fft_BarTopY(mag[i]));

            xSemaphoreGive(lcdMutex);
        }
        memcpy(prevMag, mag, sizeof(prevMag));

        /* --- 6) UartTask 에 새 데이터 도착 통지 --- */
        xSemaphoreGive(uartReadySem);
    }
}

/* ===========================================================================
 *  UartTask : 프레임 인코딩 + 전송 (블로킹, 최저 우선순위)
 * ===========================================================================*/
static void UartTask(void *argument)
{
    /* 전송 버퍼 / 로컬 복사본 (static: 스택 절약) */
    static uint8_t   txRaw[FFT_POINTS + 5];         /* 헤더5 + 256 */
    static uint8_t   txFft[FFT_POINTS * 4 + 5];     /* 헤더5 + 1024 */
    static uint8_t   localRaw[FFT_POINTS];
    static float32_t localMag[FFT_POINTS];
    uint16_t lenRaw, lenFft;

    (void)argument;

    for (;;)
    {
        /* FftTask 의 통지 대기 */
        if (xSemaphoreTake(uartReadySem, portMAX_DELAY) != pdTRUE)
            continue;

        /* 공유 데이터를 짧게 잠그고 로컬로 복사한 뒤, 전송은 잠금 없이 수행
         * (느린 전송 동안 dataMutex 를 잡고 있으면 FftTask 가 막히므로) */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(localRaw, rawBytes, sizeof(localRaw));
            memcpy(localMag, fftMag,   sizeof(localMag));
            xSemaphoreGive(dataMutex);
        }

        /* Raw(시간영역) 프레임 — 인터럽트 기반 비차단 전송 시작 후, 완료까지 잠들어 대기.
         * 전송이 진행되는 동안 CPU 는 FftTask 등 다른 태스크가 사용한다. */
        lenRaw = Build_RawFrame(txRaw, localRaw, FFT_POINTS);
        if (HAL_UART_Transmit_IT(&huart3, txRaw, lenRaw) == HAL_OK)
            xSemaphoreTake(uartTxDoneSem, portMAX_DELAY);

        /* FFT(주파수영역) 프레임 */
        lenFft = Build_FftFrame(txFft, localMag, FFT_POINTS);
        if (HAL_UART_Transmit_IT(&huart3, txFft, lenFft) == HAL_OK)
            xSemaphoreTake(uartTxDoneSem, portMAX_DELAY);
    }
}

/* ===========================================================================
 *  프레임 인코딩 헬퍼
 * ===========================================================================*/
/* Raw 프레임: [0x03][0x15][0x01][len_hi][len_lo][samples...]  (len = n 바이트) */
static uint16_t Build_RawFrame(uint8_t *out, const uint8_t *samples, uint16_t n)
{
    out[0] = FRAME_SOF0;
    out[1] = FRAME_SOF1;
    out[2] = FRAME_TYPE_RAW;
    out[3] = (uint8_t)((n >> 8) & 0xFF);
    out[4] = (uint8_t)(n & 0xFF);
    memcpy(&out[5], samples, n);
    return (uint16_t)(n + 5);
}

/* FFT 프레임: [0x03][0x15][0x02][len_hi][len_lo][float32 LE ...]  (len = n*4 바이트) */
static uint16_t Build_FftFrame(uint8_t *out, const float32_t *mag, uint16_t n)
{
    uint16_t payload = (uint16_t)(n * 4);
    out[0] = FRAME_SOF0;
    out[1] = FRAME_SOF1;
    out[2] = FRAME_TYPE_FFT;
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
