/**
  ******************************************************************************
  * @file    filters.c
  * @brief   실시간 FIR / IIR 저역통과(LPF) 필터 예제 (CMSIS-DSP 기반)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "filters.h"
#include "openx07v_c_lcd.h"   /* 범례 표시용 BSP_LCD_* / Font12 / 색상 */
#include <math.h>             /* sinf, cosf, fabsf */

/* 계수(설계 결과) ------------------------------------------------------------*/
static float32_t firCoeffs[FIR_NUM_TAPS];   /* FIR 탭 (DC 이득 1 정규화) */
static float32_t iirCoeffs[5];              /* IIR biquad: b0,b1,b2,a1,a2 (a1,a2 부호반전) */

/* 필터 상태(직접 구현) -------------------------------------------------------
 * CMSIS arm_fir_f32 / arm_biquad_cascade_df1_f32 를 "샘플당 1개(blockSize=1)" 로
 * 호출하는 경로가 이 타깃에서 노이즈를 내서, 손으로 구현한다.
 * 호스트 시뮬레이션과 정확히 일치(검증됨). 계수 설계식은 동일. */
static float32_t firHist[FIR_NUM_TAPS];     /* 최근 N개 입력, [0]=최신 */
static float32_t ix1, ix2, iy1, iy2;        /* IIR df1 상태: x[n-1],x[n-2],y[n-1],y[n-2] */

/* ===========================================================================
 *  계수 설계
 * ===========================================================================*/
/**
  * @brief  윈도우드-싱크(해밍 창) 선형위상 FIR 저역통과 계수를 설계한다.
  *         이상적 LPF 임펄스응답  h[n] = sin(2*pi*fcN*k)/(pi*k),  fcN = fc/Fs,
  *         k = n - (N-1)/2  에 해밍 창을 곱하고 DC 이득이 1 이 되도록 정규화한다.
  */
static void Fir_Design_LPF(void)
{
    const float32_t fcN = FIR_LPF_FC / FILTER_FS;   /* 정규화 차단(0..0.5) */
    const float32_t M   = (float32_t)(FIR_NUM_TAPS - 1);
    float32_t sum = 0.0f;
    int n;

    for (n = 0; n < FIR_NUM_TAPS; n++)
    {
        float32_t k = (float32_t)n - M / 2.0f;
        float32_t sinc;

        if (fabsf(k) < 1e-6f)
            sinc = 2.0f * fcN;                              /* k=0 극한값 */
        else
            sinc = sinf(2.0f * PI * fcN * k) / (PI * k);

        /* 해밍 창 */
        float32_t w = 0.54f - 0.46f * cosf(2.0f * PI * (float32_t)n / M);

        firCoeffs[n] = sinc * w;
        sum += firCoeffs[n];
    }

    /* DC 이득 = 1 로 정규화 */
    for (n = 0; n < FIR_NUM_TAPS; n++)
        firCoeffs[n] /= sum;
}

/**
  * @brief  2차 Butterworth biquad 저역통과 계수를 설계한다 (RBJ audio EQ cookbook).
  *         표준식의 분모 a1,a2 는 CMSIS df1 규약에 맞춰 부호를 반전하여 저장한다.
  *         CMSIS 차분식: y = b0*x + b1*x1 + b2*x2 + a1*y1 + a2*y2
  */
static void Iir_Design_LPF(void)
{
    const float32_t w0    = 2.0f * PI * IIR_LPF_FC / FILTER_FS;
    const float32_t cosw0 = cosf(w0);
    const float32_t sinw0 = sinf(w0);
    const float32_t Q     = 0.70710678f;            /* 1/sqrt(2) : Butterworth */
    const float32_t alpha = sinw0 / (2.0f * Q);

    float32_t b0 = (1.0f - cosw0) * 0.5f;
    float32_t b1 = (1.0f - cosw0);
    float32_t b2 = (1.0f - cosw0) * 0.5f;
    float32_t a0 = 1.0f + alpha;
    float32_t a1 = -2.0f * cosw0;
    float32_t a2 = 1.0f - alpha;

    /* a0 로 정규화 + a1,a2 부호 반전 (CMSIS 규약) */
    iirCoeffs[0] = b0 / a0;     /* b0 */
    iirCoeffs[1] = b1 / a0;     /* b1 */
    iirCoeffs[2] = b2 / a0;     /* b2 */
    iirCoeffs[3] = -a1 / a0;    /* a1 (부호 반전) */
    iirCoeffs[4] = -a2 / a0;    /* a2 (부호 반전) */
}

/* ===========================================================================
 *  공개 API
 * ===========================================================================*/
void Filters_Init(void)
{
    int i;
    Fir_Design_LPF();
    Iir_Design_LPF();
    for (i = 0; i < FIR_NUM_TAPS; i++)
        firHist[i] = 0.0f;
    ix1 = ix2 = iy1 = iy2 = 0.0f;
}

/* FIR: 직접 컨볼루션. firHist[0]=최신 입력, 계수는 대칭(선형위상)이라 순서 무관. */
float32_t Filter_FIR_LPF(float32_t x)
{
    int i;
    float32_t acc = 0.0f;

    for (i = FIR_NUM_TAPS - 1; i > 0; i--)   /* 히스토리 한 칸 시프트 */
        firHist[i] = firHist[i - 1];
    firHist[0] = x;

    for (i = 0; i < FIR_NUM_TAPS; i++)
        acc += firCoeffs[i] * firHist[i];
    return acc;
}

/* IIR: 2차 Direct Form 1.  y = b0*x + b1*x1 + b2*x2 + a1*y1 + a2*y2  (CMSIS 부호규약) */
float32_t Filter_IIR_LPF(float32_t x)
{
    float32_t y = iirCoeffs[0] * x
                + iirCoeffs[1] * ix1
                + iirCoeffs[2] * ix2
                + iirCoeffs[3] * iy1
                + iirCoeffs[4] * iy2;
    ix2 = ix1; ix1 = x;
    iy2 = iy1; iy1 = y;
    return y;
}

/* ===========================================================================
 *  LCD 범례 (RAW=노랑, FIR=초록, IIR=시안)
 * ===========================================================================*/
void Filters_DrawLegend(void)
{
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAt(130, 3, (uint8_t *)"RAW", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAt(165, 3, (uint8_t *)"FIR", LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
    BSP_LCD_DisplayStringAt(200, 3, (uint8_t *)"IIR", LEFT_MODE);
}
