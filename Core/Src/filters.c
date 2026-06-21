/**
  ******************************************************************************
  * @file    filters.c
  * @brief   실시간 FIR / IIR 밴드패스(BPF) 필터 (1.0 ~ 1.4 kHz)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "filters.h"
#include <math.h>             /* sinf, cosf, fabsf, sqrtf */

/* 계수(설계 결과) ------------------------------------------------------------*/
static float32_t firCoeffs[FIR_NUM_TAPS];   /* FIR 탭 */
static float32_t iirCoeffs[5];              /* IIR biquad: b0,b1,b2,a1,a2 (a1,a2 부호반전) */

/* 필터 상태(직접 구현, 호스트 시뮬레이션과 일치) --------------------------- */
static float32_t firHist[FIR_NUM_TAPS];     /* 최근 N개 입력, [0]=최신 */
static float32_t ix1, ix2, iy1, iy2;        /* IIR df1 상태: x[n-1],x[n-2],y[n-1],y[n-2] */

/* ===========================================================================
 *  계수 설계
 * ===========================================================================*/
/**
  * @brief  윈도우드-싱크 선형위상 BPF 계수 설계 (두 LPF 싱크의 차).
  *         이상적 BPF 임펄스응답  h[n] = (sin(2*pi*f2*k) - sin(2*pi*f1*k))/(pi*k),
  *         k = n-(N-1)/2, f = F/Fs.  해밍 창을 곱하고 중심주파수 이득 1 로 정규화한다.
  *         (DC 이득은 0 이 되어 DC 가 차단된다.)
  */
static void Fir_Design_BPF(void)
{
    const float32_t f1 = BPF_F_LOW  / FILTER_FS;
    const float32_t f2 = BPF_F_HIGH / FILTER_FS;
    const float32_t fc = (f1 + f2) * 0.5f;          /* 정규화 중심주파수 */
    const float32_t M  = (float32_t)(FIR_NUM_TAPS - 1);
    float32_t gr = 0.0f, gi = 0.0f, g;
    int n;

    for (n = 0; n < FIR_NUM_TAPS; n++)
    {
        float32_t k = (float32_t)n - M / 2.0f;
        float32_t h;

        if (fabsf(k) < 1e-6f)
            h = 2.0f * (f2 - f1);                    /* k=0 극한값 */
        else
            h = (sinf(2.0f * PI * f2 * k) - sinf(2.0f * PI * f1 * k)) / (PI * k);

        firCoeffs[n] = h * (0.54f - 0.46f * cosf(2.0f * PI * (float32_t)n / M)); /* 해밍 */
    }

    /* 중심주파수 이득 |H(fc)| = 1 로 정규화 */
    for (n = 0; n < FIR_NUM_TAPS; n++)
    {
        gr += firCoeffs[n] * cosf(2.0f * PI * fc * (float32_t)n);
        gi += firCoeffs[n] * sinf(2.0f * PI * fc * (float32_t)n);
    }
    g = sqrtf(gr * gr + gi * gi);
    if (g > 1e-6f)
        for (n = 0; n < FIR_NUM_TAPS; n++)
            firCoeffs[n] /= g;
}

/**
  * @brief  2차 biquad 밴드패스 계수 설계 (RBJ cookbook, 0 dB peak).
  *         중심 f0 = sqrt(F_low*F_high), Q = f0/(F_high-F_low).
  *         CMSIS df1 규약: a1,a2 부호 반전.  DC 이득 = 0.
  */
static void Iir_Design_BPF(void)
{
    const float32_t f0    = sqrtf(BPF_F_LOW * BPF_F_HIGH);
    const float32_t Q     = f0 / (BPF_F_HIGH - BPF_F_LOW);
    const float32_t w0    = 2.0f * PI * f0 / FILTER_FS;
    const float32_t cw    = cosf(w0);
    const float32_t sw    = sinf(w0);
    const float32_t alpha = sw / (2.0f * Q);

    float32_t b0 = alpha, b1 = 0.0f, b2 = -alpha;
    float32_t a0 = 1.0f + alpha, a1 = -2.0f * cw, a2 = 1.0f - alpha;

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
    Fir_Design_BPF();
    Iir_Design_BPF();
    for (i = 0; i < FIR_NUM_TAPS; i++)
        firHist[i] = 0.0f;
    ix1 = ix2 = iy1 = iy2 = 0.0f;
}

/* FIR: 직접 컨볼루션. firHist[0]=최신 입력, 계수 대칭(선형위상)이라 순서 무관. */
float32_t Filter_FIR(float32_t x)
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
float32_t Filter_IIR(float32_t x)
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
