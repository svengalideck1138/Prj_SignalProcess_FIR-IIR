/**
  ******************************************************************************
  * @file    filters.h
  * @brief   실시간 FIR / IIR 밴드패스(BPF) 필터 (1.0 ~ 1.4 kHz)
  ******************************************************************************
  * 계수는 빌드 시 하드코딩하지 않고 Filters_Init() 에서 통과대역/샘플레이트로부터
  * "런타임 설계" 한다. 아래 #define 만 바꾸면 다른 대역으로 즉시 재설계된다.
  *   - FIR : 윈도우드-싱크(해밍 창) 선형위상 BPF (difference-of-sincs)
  *   - IIR : 2차 biquad BPF (RBJ cookbook, 0 dB peak)
  * 두 필터 모두 DC 이득 = 0 (DC 차단).
  ******************************************************************************
  */
#ifndef __FILTERS_H
#define __FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

/* 설계 파라미터 -------------------------------------------------------------*/
#define FILTER_FS    8842.0f    /* 샘플링 주파수 [Hz] (tim.c 의 TIM3 페이싱과 동일) */
#define BPF_F_LOW    1000.0f    /* 통과대역 하한 [Hz] */
#define BPF_F_HIGH   2000.0f    /* 통과대역 상한 [Hz] */
#define FIR_NUM_TAPS 101        /* FIR 탭 수(홀수=선형위상). 좁은 대역이라 충분히 크게 */

/* API -----------------------------------------------------------------------*/
void      Filters_Init(void);          /* 계수 설계 + 상태 초기화 (1회) */
float32_t Filter_FIR(float32_t x);     /* 샘플 1개 -> FIR BPF 출력 */
float32_t Filter_IIR(float32_t x);     /* 샘플 1개 -> IIR BPF 출력 */

#ifdef __cplusplus
}
#endif

#endif /* __FILTERS_H */
