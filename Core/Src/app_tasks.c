/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   FreeRTOS application: ADC(블록 DMA) / FIR·IIR / FFT / TFT / UART
  ******************************************************************************
  * 파이프라인 (블록 단위, 균일 샘플링):
  *   1) ADC+DMA 가 TIM3 트리거로 한 블록(256 샘플)을 하드웨어로 균일하게 채운다.
  *      (더블버퍼: 한쪽을 채우는 동안 반대쪽을 처리 -> 트리거 지터 없음)
  *   2) [DspTask] 원신호를 FIR/IIR 로 변환, 원신호 포함 3개 신호를 버퍼에 저장
  *   3) [DspTask] 각 신호를 FFT 로 변환(크기)
  *   4) [DisplayTask] 시간영역(좌) + FFT(우) 를 RAW/FIR/IIR 3행으로 TFT 표시
  *   5) [UartTask] 6종 결과를 UART 전송 (ID 0x01~0x06)
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
#include <math.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private defines -----------------------------------------------------------*/
#define BLK             256                 /* 블록 / FFT 크기 (256 = 4^4, radix4 가능) */
#define FFT_BINS        (BLK / 2)           /* 표시에 쓰는 유효 빈 수 = 128 */

/* 표시 레이아웃: 3행(RAW/FIR/IIR) x 2열(시간영역 | FFT) */
#define ROW_RAW         22                  /* 1행 상단 y */
#define ROW_FIR         94                  /* 2행 */
#define ROW_IIR         166                 /* 3행 (하단 166..229) */
#define PLOT_H          64                  /* 각 플롯 높이(px) */
#define PLOT_W          146                 /* 각 플롯 폭(px) */
#define TIME_X0         22                  /* 시간영역 열 시작 x (좌측 라벨 이후) */
#define FFT_X0          172                 /* FFT 열 시작 x */

/* FFT dB 표시 구간 (8bit 입력이라 크기가 큼) */
#define FFT_DBMIN       45.0f
#define FFT_DBMAX       95.0f

/* 태스크 우선순위/스택 */
#define PRIO_DSP        (tskIDLE_PRIORITY + 3)   /* 최고: 매 블록 필터(연속성) */
#define PRIO_UART       (tskIDLE_PRIORITY + 2)
#define PRIO_DISP       (tskIDLE_PRIORITY + 1)
#define STACK_DSP       512
#define STACK_UART      512
#define STACK_DISP      512

/* UART 프레임 헤더/ID (0x01~0x06) */
#define FRAME_SOF0      0x03
#define FRAME_SOF1      0x15
#define ID_RAW_TIME     0x01
#define ID_RAW_FFT      0x02
#define ID_FIR_TIME     0x03
#define ID_FIR_FFT      0x04
#define ID_IIR_TIME     0x05
#define ID_IIR_FFT      0x06

/* Private variables ---------------------------------------------------------*/
/* ADC 블록 DMA 더블버퍼 (8bit) : [0..BLK-1]=half0, [BLK..2BLK-1]=half1 */
static uint8_t adcDma[2 * BLK];

/* 동기화 */
static QueueHandle_t     halfQ;         /* ISR -> DspTask : 완료된 half(0/1) */
static SemaphoreHandle_t dataMutex;     /* 공개 결과 보호 */
static SemaphoreHandle_t displaySem;    /* DspTask -> DisplayTask */
static SemaphoreHandle_t uartReadySem;  /* DspTask -> UartTask */
static SemaphoreHandle_t uartTxDoneSem; /* USART3 TX 완료 ISR -> UartTask */

/* DspTask 작업용 (시간영역 float) */
static float32_t rawSig[BLK], firSig[BLK], iirSig[BLK];
static float32_t fftScratch[2 * BLK];   /* FFT in-place 스크래치(복소 인터리브) */
static float32_t fftWin[BLK];           /* Hann 창 */
static arm_cfft_radix4_instance_f32 S;

/* 공개 결과 (dataMutex 보호): DisplayTask / UartTask 가 읽음 */
static uint8_t   rawBytes[BLK], firBytes[BLK], iirBytes[BLK];   /* 시간영역 0..255 */
static float32_t magRaw[BLK], magFir[BLK], magIir[BLK];         /* FFT 크기 (0..FFT_BINS 사용) */

/* Task handles */
static TaskHandle_t hDsp, hDisp, hUart;

/* Private function prototypes -----------------------------------------------*/
static void DspTask(void *argument);
static void DisplayTask(void *argument);
static void UartTask(void *argument);
static uint16_t Build_TimeFrame(uint8_t *out, uint8_t id, const uint8_t *s, uint16_t n);
static uint16_t Build_FftFrame(uint8_t *out, uint8_t id, const float32_t *mag, uint16_t n);
static void     Uart_Send(const uint8_t *buf, uint16_t len);

/* ===========================================================================
 *  ISR hooks : ADC 블록(half) 완료 -> DspTask, UART TX 완료 -> UartTask
 * ===========================================================================*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && halfQ != NULL)
    {
        uint8_t half = 0;                 /* 앞쪽 절반 채움 완료 */
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(halfQ, &half, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && halfQ != NULL)
    {
        uint8_t half = 1;                 /* 뒤쪽 절반 채움 완료 */
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(halfQ, &half, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3 && uartTxDoneSem != NULL)
    {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ===========================================================================
 *  공통 헬퍼
 * ===========================================================================*/
static uint8_t Clamp8(float32_t v)
{
    int iv = (int)(v + 0.5f);
    if (iv < 0)   iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

/* 시간영역 신호(float)를 DC 제거 + Hann 창 -> 복소 FFT -> 크기(magOut[BLK]) */
static void SpecOf(const float32_t *sig, float32_t *magOut)
{
    uint16_t i;
    float32_t mean = 0.0f, wmean = 0.0f;

    for (i = 0; i < BLK; i++)
        mean += sig[i];
    mean /= (float32_t)BLK;

    /* DC 제거 -> Hann 창. 창을 곱하면 잔류 DC 가 생겨 FFT bin0 이 솟으므로,
     * 창 적용 후 다시 평균을 빼서 DC(bin0)를 0 으로 만든다. */
    for (i = 0; i < BLK; i++)
    {
        fftScratch[2 * i]     = (sig[i] - mean) * fftWin[i];  /* 실수부 */
        fftScratch[2 * i + 1] = 0.0f;                         /* 허수부 */
        wmean += fftScratch[2 * i];
    }
    wmean /= (float32_t)BLK;
    for (i = 0; i < BLK; i++)
        fftScratch[2 * i] -= wmean;                           /* 창 적용 후 잔류 DC 제거 */

    arm_cfft_radix4_f32(&S, fftScratch);
    arm_cmplx_mag_f32(fftScratch, magOut, BLK);
}

/* ===========================================================================
 *  DspTask : 블록 수집 -> FIR/IIR -> FFT -> 결과 공개  (단계 1~3)
 * ===========================================================================*/
static void DspTask(void *argument)
{
    uint8_t  half;
    uint16_t i;

    (void)argument;

    /* Hann 창 + FFT 인스턴스 준비 */
    for (i = 0; i < BLK; i++)
        fftWin[i] = 1.0f - cosf(2.0f * PI * (float32_t)i / (float32_t)(BLK - 1));
    arm_cfft_radix4_init_f32(&S, BLK, 0, 1);

    /* (1) ADC 블록 DMA + TIM3 트리거 시작 -> 하드웨어가 균일하게 샘플링 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcDma, 2 * BLK);
    HAL_TIM_Base_Start(&htim3);

    for (;;)
    {
        if (xQueueReceive(halfQ, &half, portMAX_DELAY) != pdTRUE)
            continue;

        const uint8_t *src = &adcDma[half * BLK];   /* 방금 완료된 절반 */

        /* (2) 원신호 저장 + FIR/IIR 밴드패스 변환 (필터는 매 블록 연속 처리 -> 상태 유지) */
        for (i = 0; i < BLK; i++)
        {
            float32_t r = (float32_t)src[i];
            rawSig[i] = r;
            firSig[i] = Filter_FIR(r);
            iirSig[i] = Filter_IIR(r);
        }

        /* (3) 각 신호 FFT + 시간영역 바이트화 -> 결과 공개.
         *     BPF 출력은 0 중심(양/음)이라 표시/UART(uint8) 위해 +128 오프셋. */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            for (i = 0; i < BLK; i++)
            {
                rawBytes[i] = Clamp8(rawSig[i]);
                firBytes[i] = Clamp8(firSig[i] + 128.0f);
                iirBytes[i] = Clamp8(iirSig[i] + 128.0f);
            }
            SpecOf(rawSig, magRaw);
            SpecOf(firSig, magFir);
            SpecOf(iirSig, magIir);
            xSemaphoreGive(dataMutex);
        }

        /* (4)/(5) 표시·전송 태스크에 통지 (느린 작업은 비동기로 분리) */
        xSemaphoreGive(displaySem);
        xSemaphoreGive(uartReadySem);
    }
}

/* ===========================================================================
 *  DisplayTask : 시간영역(좌) + FFT(우) 를 RAW/FIR/IIR 3행 표시  (단계 4)
 * ===========================================================================*/
static uint16_t s_ny[PLOT_W];           /* 플롯 1개 분량 y 좌표 스크래치 */

/* s_ny 를 기준으로 이전 곡선(prevY)을 지우고 새 곡선을 그린 뒤 prevY 갱신 */
static void Plot_Render(uint16_t x0, uint16_t *prevY, uint16_t color)
{
    uint16_t c;
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    for (c = 1; c < PLOT_W; c++)
        BSP_LCD_DrawLine(x0 + c - 1, prevY[c - 1], x0 + c, prevY[c]);
    BSP_LCD_SetTextColor(color);
    for (c = 1; c < PLOT_W; c++)
        BSP_LCD_DrawLine(x0 + c - 1, s_ny[c - 1], x0 + c, s_ny[c]);
    for (c = 0; c < PLOT_W; c++)
        prevY[c] = s_ny[c];
}

/* 시간영역(0..255) 한 행을 s_ny 에 채움 (값↑ = 위) */
static void Plot_FillTime(const uint8_t *bytes, uint16_t top)
{
    uint16_t c;
    for (c = 0; c < PLOT_W; c++)
    {
        int di = (int)c * BLK / PLOT_W;
        int v  = bytes[di];
        s_ny[c] = (uint16_t)(top + (PLOT_H - 1) - (v * (PLOT_H - 1) / 255));
    }
}

/* FFT 크기 한 행을 dB 스케일로 s_ny 에 채움 */
static void Plot_FillFft(const float32_t *mag, uint16_t top)
{
    uint16_t c;
    for (c = 0; c < PLOT_W; c++)
    {
        int bin = (int)c * FFT_BINS / PLOT_W;        /* 0..127 */
        float32_t db = 20.0f * log10f(mag[bin] + 1.0f);
        int h = (int)((db - FFT_DBMIN) * ((float32_t)(PLOT_H - 1) / (FFT_DBMAX - FFT_DBMIN)));
        if (h < 0)            h = 0;
        if (h > (PLOT_H - 1)) h = (PLOT_H - 1);
        s_ny[c] = (uint16_t)(top + (PLOT_H - 1) - h);
    }
}

static void DisplayTask(void *argument)
{
    /* 공개 결과의 로컬 복사본 + 6개 플롯의 직전 곡선 */
    static uint8_t   dRB[BLK], dFB[BLK], dIB[BLK];
    static float32_t dMR[BLK], dMF[BLK], dMI[BLK];
    static uint16_t  pTR[PLOT_W], pTF[PLOT_W], pTI[PLOT_W];  /* 시간영역 prev */
    static uint16_t  pFR[PLOT_W], pFF[PLOT_W], pFI[PLOT_W];  /* FFT prev */
    uint16_t c;

    (void)argument;

    /* 행 라벨 + prev 곡선 베이스라인 초기화 (첫 지우기가 화면을 더럽히지 않게) */
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAt(1, ROW_RAW, (uint8_t *)"RAW", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAt(1, ROW_FIR, (uint8_t *)"FIR", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    BSP_LCD_DisplayStringAt(1, ROW_IIR, (uint8_t *)"IIR", LEFT_MODE);
    for (c = 0; c < PLOT_W; c++)
    {
        pTR[c] = ROW_RAW + PLOT_H - 1; pTF[c] = ROW_FIR + PLOT_H - 1; pTI[c] = ROW_IIR + PLOT_H - 1;
        pFR[c] = ROW_RAW + PLOT_H - 1; pFF[c] = ROW_FIR + PLOT_H - 1; pFI[c] = ROW_IIR + PLOT_H - 1;
    }

    for (;;)
    {
        if (xSemaphoreTake(displaySem, portMAX_DELAY) != pdTRUE)
            continue;

        /* 공개 결과를 짧게 잠그고 로컬 복사 후, 그리기는 잠금 없이 */
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(dRB, rawBytes, sizeof(dRB));
            memcpy(dFB, firBytes, sizeof(dFB));
            memcpy(dIB, iirBytes, sizeof(dIB));
            memcpy(dMR, magRaw, sizeof(dMR));
            memcpy(dMF, magFir, sizeof(dMF));
            memcpy(dMI, magIir, sizeof(dMI));
            xSemaphoreGive(dataMutex);
        }

        /* 좌측 열 = 시간영역, 우측 열 = FFT */
        Plot_FillTime(dRB, ROW_RAW); Plot_Render(TIME_X0, pTR, LCD_COLOR_YELLOW);
        Plot_FillTime(dFB, ROW_FIR); Plot_Render(TIME_X0, pTF, LCD_COLOR_GREEN);
        Plot_FillTime(dIB, ROW_IIR); Plot_Render(TIME_X0, pTI, LCD_COLOR_CYAN);
        Plot_FillFft(dMR, ROW_RAW);  Plot_Render(FFT_X0, pFR, LCD_COLOR_YELLOW);
        Plot_FillFft(dMF, ROW_FIR);  Plot_Render(FFT_X0, pFF, LCD_COLOR_GREEN);
        Plot_FillFft(dMI, ROW_IIR);  Plot_Render(FFT_X0, pFI, LCD_COLOR_CYAN);
    }
}

/* ===========================================================================
 *  UartTask : 6종 결과 전송 (ID 0x01~0x06)  (단계 5)
 * ===========================================================================*/
static void Uart_Send(const uint8_t *buf, uint16_t len)
{
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, len) == HAL_OK)
        xSemaphoreTake(uartTxDoneSem, portMAX_DELAY);
}

static void UartTask(void *argument)
{
    static uint8_t   txTime[BLK + 6];   /* 헤더5 + payload + 체크섬1 */
    static uint8_t   txFft [BLK * 4 + 6];
    static uint8_t   locRaw[BLK], locFir[BLK], locIir[BLK];
    static float32_t locMagR[BLK], locMagF[BLK], locMagI[BLK];
    uint16_t len;

    (void)argument;

    for (;;)
    {
        if (xSemaphoreTake(uartReadySem, portMAX_DELAY) != pdTRUE)
            continue;

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(locRaw,  rawBytes, sizeof(locRaw));
            memcpy(locFir,  firBytes, sizeof(locFir));
            memcpy(locIir,  iirBytes, sizeof(locIir));
            memcpy(locMagR, magRaw,   sizeof(locMagR));
            memcpy(locMagF, magFir,   sizeof(locMagF));
            memcpy(locMagI, magIir,   sizeof(locMagI));
            xSemaphoreGive(dataMutex);
        }

        len = Build_TimeFrame(txTime, ID_RAW_TIME, locRaw, BLK);  Uart_Send(txTime, len);
        len = Build_FftFrame (txFft,  ID_RAW_FFT,  locMagR, BLK); Uart_Send(txFft, len);
        len = Build_TimeFrame(txTime, ID_FIR_TIME, locFir, BLK);  Uart_Send(txTime, len);
        len = Build_FftFrame (txFft,  ID_FIR_FFT,  locMagF, BLK); Uart_Send(txFft, len);
        len = Build_TimeFrame(txTime, ID_IIR_TIME, locIir, BLK);  Uart_Send(txTime, len);
        len = Build_FftFrame (txFft,  ID_IIR_FFT,  locMagI, BLK); Uart_Send(txFft, len);
    }
}

/* 프레임: [0x03][0x15][id][len_hi][len_lo][payload...][cs]
 * cs = id ^ len_hi ^ len_lo ^ payload 의 XOR (수신측이 깨진 프레임을 폐기) */
static uint16_t Build_TimeFrame(uint8_t *out, uint8_t id, const uint8_t *s, uint16_t n)
{
    uint16_t i;
    uint8_t cs;
    out[0] = FRAME_SOF0; out[1] = FRAME_SOF1; out[2] = id;
    out[3] = (uint8_t)((n >> 8) & 0xFF); out[4] = (uint8_t)(n & 0xFF);
    memcpy(&out[5], s, n);
    cs = out[2] ^ out[3] ^ out[4];
    for (i = 0; i < n; i++) cs ^= s[i];
    out[5 + n] = cs;
    return (uint16_t)(n + 6);
}

static uint16_t Build_FftFrame(uint8_t *out, uint8_t id, const float32_t *mag, uint16_t n)
{
    uint16_t payload = (uint16_t)(n * 4), i;
    uint8_t cs;
    out[0] = FRAME_SOF0; out[1] = FRAME_SOF1; out[2] = id;
    out[3] = (uint8_t)((payload >> 8) & 0xFF); out[4] = (uint8_t)(payload & 0xFF);
    memcpy(&out[5], mag, payload);     /* float32 little-endian */
    cs = out[2] ^ out[3] ^ out[4];
    for (i = 0; i < payload; i++) cs ^= out[5 + i];
    out[5 + payload] = cs;
    return (uint16_t)(payload + 6);
}

/* ===========================================================================
 *  초기화
 * ===========================================================================*/
void App_FreeRTOS_Init(void)
{
    Filters_Init();

    halfQ         = xQueueCreate(4, sizeof(uint8_t));
    dataMutex     = xSemaphoreCreateMutex();
    displaySem    = xSemaphoreCreateBinary();
    uartReadySem  = xSemaphoreCreateBinary();
    uartTxDoneSem = xSemaphoreCreateBinary();
    configASSERT(halfQ && dataMutex && displaySem && uartReadySem && uartTxDoneSem);

    xTaskCreate(DspTask,     "Dsp",  STACK_DSP,  NULL, PRIO_DSP,  &hDsp);
    xTaskCreate(DisplayTask, "Disp", STACK_DISP, NULL, PRIO_DISP, &hDisp);
    xTaskCreate(UartTask,    "Uart", STACK_UART, NULL, PRIO_UART, &hUart);
}
