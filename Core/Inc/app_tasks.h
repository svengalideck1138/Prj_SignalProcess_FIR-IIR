/**
  ******************************************************************************
  * @file    app_tasks.h
  * @brief   FreeRTOS application: ADC / FFT / UART task interface
  ******************************************************************************
  * 실시간 FFT 스펙트럼 분석기를 3개의 FreeRTOS 태스크로 분리한 구조.
  *   - AdcTask  : ADC 변환완료(샘플)마다 수집 + 시간영역 LCD 표시
  *   - FftTask  : 256샘플 블록에 대해 CMSIS-DSP FFT + magnitude + FFT LCD 표시
  *   - UartTask : Raw/FFT 프레임 인코딩 후 UART 전송 (블로킹, 최저 우선순위)
  ******************************************************************************
  */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  동기화 객체(세마포어/큐/뮤텍스)와 3개 태스크를 생성한다.
  *         vTaskStartScheduler()(또는 osKernelStart()) 호출 전에 한 번 호출한다.
  */
void App_FreeRTOS_Init(void);

/* 샘플링 틱은 더 이상 TIM7 ISR 가 아니라 ADC 변환완료 콜백
 * (HAL_ADC_ConvCpltCallback, app_tasks.c)에서 발생한다. TIM7 은 TRGO 로
 * ADC 를 하드웨어 트리거하는 역할만 한다. */

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASKS_H */
