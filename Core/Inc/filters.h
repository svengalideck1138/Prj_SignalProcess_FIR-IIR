/**
  ******************************************************************************
  * @file    filters.h
  * @brief   실시간 FIR / IIR 저역통과(LPF) 필터 예제 (CMSIS-DSP 기반)
  ******************************************************************************
  * ADC 로 들어오는 시간영역 신호를 두 가지 방식의 저역통과 필터로 처리한다.
  *   - FIR : 윈도우드-싱크(해밍 창) 선형위상 LPF        -> arm_fir_f32
  *   - IIR : 2차 Butterworth biquad LPF (RBJ cookbook) -> arm_biquad_cascade_df1_f32
  *
  * 계수는 빌드 시 하드코딩하지 않고 Filters_Init() 에서 차단주파수/샘플레이트로부터
  * "런타임 설계" 한다. 아래 #define 만 바꾸면 다른 차단주파수로 즉시 재설계된다.
  ******************************************************************************
  */
#ifndef __FILTERS_H
#define __FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

/* 설계 파라미터 -------------------------------------------------------------*/
/* 샘플링 주파수: TIM3 가 ADC 를 페이싱하는 주파수와 동일해야 한다.
 * 84MHz / ((Prescaler+1)*(Period+1)) = 84e6 / (10*950) = 8842.1 Hz  (tim.c 참조) */
#define FILTER_FS        8842.0f

#define FIR_LPF_FC       500.0f   /* FIR 저역통과 차단주파수 [Hz] (0 < fc < Fs/2) */
#define IIR_LPF_FC       500.0f   /* IIR 저역통과 차단주파수 [Hz] (0 < fc < Fs/2) */

#define FIR_NUM_TAPS     31       /* FIR 탭 수. 홀수 -> 선형위상, 그룹지연 = (N-1)/2 샘플 */

/* API -----------------------------------------------------------------------*/
void      Filters_Init(void);              /* 계수 설계 + 상태 초기화 (스케줄러 시작 전에 1회) */
float32_t Filter_FIR_LPF(float32_t x);     /* 샘플 1개 입력 -> FIR LPF 출력 */
float32_t Filter_IIR_LPF(float32_t x);     /* 샘플 1개 입력 -> IIR LPF 출력 */
void      Filters_DrawLegend(void);        /* LCD 상단에 RAW/FIR/IIR 색상 범례 1회 표시 */

#ifdef __cplusplus
}
#endif

#endif /* __FILTERS_H */
