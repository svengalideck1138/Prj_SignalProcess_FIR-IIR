# Prj_SignalProcess_FIR-IIR

<p align="center">
  <img src="docs/demo.gif" alt="FIR/IIR 실시간 필터 데모" width="640">
</p>

STM32F407 + FreeRTOS 기반 **실시간 FIR / IIR 밴드패스 필터 + FFT 스펙트럼 분석기**.

ADC로 수집한 아날로그 신호를 FIR·IIR 두 방식으로 동시에 필터링하고, 원신호를 포함한
3개 신호를 각각 시간영역 + 주파수영역(FFT)으로 변환하여 TFT-LCD에 6개 그래프로 표시하고,
동일 결과를 UART로 PC 모니터에 전송한다.

```
                +-----------+   block DMA   +-----------+   +-----+   +-----------+
  Analog  --->  | ADC1 8bit | ------------> |  DspTask  |-->| FFT |-->| Display   |
  (PA0)         | TIM3 trig |  double-buf   | FIR / IIR |   |     |   | (TFT 6ch) |
                +-----------+               +-----------+   +-----+   +-----------+
                                                  |                   +-----------+
                                                  +-----------------> | UART 6ch  |
                                                                      +-----------+
```

---

## 1. 하드웨어 / 신호 파라미터

| 항목 | 값 | 비고 |
|------|----|----|
| MCU | STM32F407 (Cortex-M4F, ARM_MATH_CM4) | |
| 입력 | ADC1, 채널0 (PA0), **8-bit** 해상도 | 0..255 |
| 샘플링 트리거 | TIM3 TRGO (prescaler=9, period=949) | |
| 샘플링 주파수 `FILTER_FS` | **≈ 8842 Hz** | 84 MHz / (10 × 950) |
| 블록 / FFT 크기 `BLK` | **256** 샘플 | 256 = 4⁴ → radix-4 |
| 통과대역 | **1000 ~ 2000 Hz** | `BPF_F_LOW` / `BPF_F_HIGH` |
| RTOS | FreeRTOS (CMSIS-RTOS) | 3 tasks |
| 표시 | TFT-LCD (ILI9325, openx07v_c) | 3행 × 2열 = 6 plot |
| 통신 | USART3 @ **921600** bps | 6-frame 프로토콜 |

> 통과대역·탭 수·샘플레이트는 `Core/Inc/filters.h` 의 `#define` 만 바꾸면
> 빌드 시 자동 재설계된다(아래 알고리즘 참조).

---

## 2. 핵심 알고리즘

### 2.1 FIR 밴드패스 — 윈도우드-싱크 (선형 위상)

이상적인 BPF의 임펄스 응답은 두 LPF 싱크(sinc)의 차로 표현된다.

```
        sin(2π·f2·k) − sin(2π·f1·k)
h[k] = ─────────────────────────────       k = n − (N−1)/2,  f = F / Fs
                   π·k
```

- 100 % 이상적 응답은 무한 길이이므로 **해밍(Hamming) 창**을 곱해 `N = 101` 탭으로 절단한다.
  탭 수가 홀수이고 계수가 대칭이라 **선형 위상**(군지연 일정)이 보장된다.
- 설계 후 **중심주파수 이득 |H(fc)| = 1** 로 정규화한다.
- 결과적으로 **DC 이득 = 0** → 직류 성분이 차단된다.
- 실행 시에는 직접 컨볼루션:  `y[n] = Σ h[i]·x[n−i]`
- 구현: `Fir_Design_BPF()` (계수 설계) / `Filter_FIR()` (실행) — `Core/Src/filters.c`

```c
firCoeffs[n] = h * (0.54f - 0.46f * cosf(2.0f*PI*n/M));   // 해밍 창
```

### 2.2 IIR 밴드패스 — 2차 Biquad (RBJ cookbook)

RBJ "Audio EQ Cookbook" 의 0 dB-peak 밴드패스 2차 biquad.

```
        f0 = √(F_low · F_high)            // 기하중심 주파수
        Q  = f0 / (F_high − F_low)        // 대역폭으로부터 Q
        w0 = 2π·f0 / Fs ,  α = sin(w0)/(2Q)

        b0 =  α,   b1 = 0,   b2 = −α
        a0 = 1+α,  a1 = −2cos(w0),  a2 = 1−α
```

- 모든 계수를 `a0` 로 정규화하고 `a1, a2` 는 **부호를 반전**해 저장한다(CMSIS Direct-Form-1 규약).
- 실행은 2차 Direct Form 1:

```
y[n] = b0·x[n] + b1·x[n−1] + b2·x[n−2] + a1·y[n−1] + a2·y[n−2]
```

- FIR과 마찬가지로 **DC 이득 = 0**.
- 구현: `Iir_Design_BPF()` / `Filter_IIR()` — `Core/Src/filters.c`

> **FIR vs IIR**: FIR은 탭 수가 많아 연산량이 크지만 위상이 선형이고 항상 안정적이다.
> IIR은 단 5개 계수·2개 상태로 같은 대역을 구현해 매우 가볍지만 위상이 비선형이다.
> 이 프로젝트는 두 결과를 나란히 비교 표시하는 것이 목적이다.

### 2.3 FFT 스펙트럼 (`SpecOf`, `Core/Src/app_tasks.c`)

블록 256 샘플에 대해:

1. **평균(DC) 제거** → 시간영역의 직류 오프셋 제거
2. **Hann 창** 적용 → 스펙트럼 누설(leakage) 억제
3. 창을 곱하면 잔류 DC가 생겨 FFT bin0 가 솟으므로, **창 적용 후 평균을 한 번 더 제거**한다
   (DC 스파이크 제거 — 실측에서 확인된 보정).
4. `arm_cfft_radix4_f32` (256-pt) → `arm_cmplx_mag_f32` 로 크기 스펙트럼 산출.
5. 표시는 `20·log10(mag+1)` dB 스케일, 윈도우 45~95 dB.

---

## 3. ⭐ 즉시(per-sample) 처리 → 블록 버퍼링 개선 (핵심)

> **이 프로젝트에서 가장 중요한 설계 변경.**

### 문제 — 샘플마다 즉시 필터링하던 초기 구조

초기 구현은 ADC 변환을 **샘플 단위 소프트웨어 읽기**(단일 `AdcVal` 폴링/ISR)로 받아
들어오는 즉시 FIR/IIR를 돌렸다. 그 결과:

- 샘플 시점이 **소프트웨어 타이밍(ISR 지연·태스크 스케줄링)에 의존** → **트리거 지터** 발생.
- 균일하지 않은 샘플 간격 때문에 **FFT 스펙트럼이 번지고 노이즈처럼 흐려짐**.
- 매 샘플마다 깨우기/문맥전환 오버헤드가 커서 고속(8.8 kHz)에서 처리가 밀림.

> 필터 수식·계수·상태 버퍼 크기·창 함수까지 모두 검증했지만 원인은 **수집(ADC 타이밍)** 쪽이었다.

### 해결 — 블록 단위 더블버퍼 DMA + 일괄 처리

수집을 소프트웨어에서 **하드웨어(DMA)** 로 옮기고, 한 번에 한 블록씩 모아서 처리하도록 바꿨다.

| 구분 | 이전 (즉시 처리) | 이후 (블록 버퍼링) |
|------|------------------|---------------------|
| ADC 읽기 | 샘플마다 SW 읽기 (`AdcVal`) | **DMA2_Stream0 순환·MemInc**, `adcDma[2*256]` |
| 샘플 타이밍 | SW(ISR/태스크) 의존 → 지터 | **TIM3 TRGO 하드웨어 트리거** → 균일 |
| 처리 단위 | 1 샘플 | **256 샘플 블록** |
| 버퍼 | 단일 변수 | **더블버퍼** (half0 / half1) |
| 동기화 | 샘플마다 통지 | 블록(half) 완료 시 1회 통지 (`halfQ`) |
| FFT 품질 | 흐림/노이즈 | **선명한 스펙트럼** |

**더블버퍼(핑퐁) 동작:**

```
adcDma[ 0 .. 255 ]  half0  ─┐  DMA가 half0 채우는 동안 → DspTask는 half1 처리
adcDma[256 .. 511 ]  half1  ─┘  DMA가 half1 채우는 동안 → DspTask는 half0 처리
```

- `HAL_ADC_ConvHalfCpltCallback` → 앞쪽 절반(half=0) 완료를 큐에 push
- `HAL_ADC_ConvCpltCallback` → 뒤쪽 절반(half=1) 완료를 큐에 push
- DMA가 **순환(circular)** 모드라 끊김 없이 한쪽을 채우는 동안 반대쪽을 안전하게 처리 → **샘플 손실 없음 + 지터 없음**.

**필터 상태 연속성:** 블록 단위로 처리하더라도 `Filter_FIR`/`Filter_IIR` 의 내부 상태
(`firHist`, `ix1/ix2/iy1/iy2`)는 블록 경계에서 초기화하지 않고 **계속 이어서** 적용한다.
즉 "블록으로 모아 받되, 필터는 끊김 없는 연속 스트림처럼" 동작하므로 블록 경계 불연속(클릭/번짐)이 없다.

```c
// DspTask: 한 블록을 한 번에, 상태는 블록을 가로질러 유지
for (i = 0; i < BLK; i++) {
    float32_t r = (float32_t)src[i];   // src = 방금 완료된 절반
    rawSig[i] = r;
    firSig[i] = Filter_FIR(r);         // 내부 히스토리 연속
    iirSig[i] = Filter_IIR(r);
}
```

**결과:** 균일 샘플링 확보 → FFT가 선명해지고, 블록당 1회만 깨어나므로 8.8 kHz 처리에 여유가 생겼다.

---

## 4. FreeRTOS 태스크 구조

수집/필터(빠르고 연속성 중요)와 표시/전송(느리고 비동기)을 분리해, 느린 작업이
필터링을 막지 않도록 했다.

| 태스크 | 우선순위 | 역할 |
|--------|---------|------|
| **DspTask** | 최고 (+3) | 블록 수집 → FIR/IIR → FFT → 결과 공개. 필터 연속성 보장을 위해 최우선. |
| **UartTask** | +2 | 6종 프레임 인코딩 후 USART3 IT 전송 |
| **DisplayTask** | +1 | 시간영역(좌) + FFT(우) 를 RAW/FIR/IIR 3행으로 TFT 갱신 |

- 공개 결과(`rawBytes`/`firBytes`/`iirBytes`, `magRaw/Fir/Iir`)는 `dataMutex` 로 보호.
- DspTask는 결과 공개 후 `displaySem` + `uartReadySem` 으로 두 소비 태스크를 깨운다.
- BPF 출력은 0 중심(±값)이라 8-bit 표시/전송을 위해 **+128 오프셋** 후 `Clamp8`.

---

## 5. UART 프로토콜 (USART3 @ 921600)

체크섬 포함 6-프레임. 921600 bps 고속에서 라인 오류 프레임을 수신측이 폐기하도록 XOR 체크섬을 둔다.

```
[0x03][0x15][id][len_hi][len_lo][payload ...][cs]
cs = id ^ len_hi ^ len_lo ^ (payload 전체 바이트 XOR)
```

| ID | 내용 | payload |
|----|------|---------|
| 0x01 | RAW 시간영역 | 256 × uint8 |
| 0x02 | RAW FFT | 256 × float32 LE |
| 0x03 | FIR 시간영역 | 256 × uint8 |
| 0x04 | FIR FFT | 256 × float32 LE |
| 0x05 | IIR 시간영역 | 256 × uint8 |
| 0x06 | IIR FFT | 256 × float32 LE |

> PC 측은 형제 저장소 `Prj_SignalProcess_Monitor` (C# WinForms)에서 producer-consumer
> 리더 + ID별 길이 검증 + 체크섬으로 파싱하여 6개 차트를 그린다.

### 5.1 C# 수신부의 체크섬(CRC) 검사 → 통신 오류 감소

921600 bps 고속·고밀도(블록당 6프레임 ≈ 4.6 KB) 전송에서는 비트 깨짐과 동기 어긋남이
빈번하다. C# 모니터(`SerialFrameReader.cs`)는 **상태기계 파서 + 무결성 검사**로 깨진
데이터가 화면(차트)까지 올라오지 못하게 막는다. 그 결과 그래프가 튀거나 깨지는 현상이
크게 줄었다.

송신(STM32)과 수신(C#)이 **동일한 XOR 체크섬** 규약을 공유한다.

```
cs = id ^ len_hi ^ len_lo ^ (payload 전체 바이트 XOR)
```

```csharp
// SerialFrameReader.ParseByte() — 수신 누적 체크섬
case St.LenLo:
    _len |= b;
    if (_len != ExpectedLen(_id)) { _st = St.Sof0; break; }       // ① ID별 길이 검증
    _cs = (byte)(_id ^ ((_len >> 8) & 0xFF) ^ (_len & 0xFF));
    ...
case St.Payload:
    _payload[_idx++] = b;  _cs ^= b;                              // 페이로드 누적 XOR
    ...
case St.Checksum:
    if (b == _cs) FrameReceived?.Invoke(_id, _payload);           // ② 일치할 때만 전달, 불일치 폐기
    _st = St.Sof0;
```

오류를 줄이는 3중 방어:

| 방어 | 내용 | 막아주는 오류 |
|------|------|---------------|
| **① ID별 길이 검증** | `ExpectedLen(id)` 와 다른 길이면 즉시 재동기 | 페이로드 속 `0x03 0x15` 를 헤더로 오인한 **가짜 동기** |
| **② XOR 체크섬** | 수신 누적 XOR ≠ 마지막 바이트면 프레임 **폐기** | 전송 중 **비트 깨짐(라인 오류)** 프레임 |
| **③ SOF 재동기** | `0x03 0x15` 2바이트 시작코드 + 상태기계 | 바이트 유실 후 **프레임 경계 어긋남** |

또한 **producer-consumer 구조**(읽기 스레드 ↔ 파싱 태스크 분리)로 시리얼 읽기가
파싱에 막히지 않아 버퍼 오버런/데이터 끊김도 방지한다.

> 비고: 이름은 "CRC 검사"로 부르지만 실제 구현은 **1바이트 XOR 체크섬**이다.
> 가볍고 단일 비트 오류 검출에 충분하다. 더 강한 검출이 필요하면 양쪽을
> CRC-8/CRC-16 으로 교체할 수 있다(프레임 구조는 그대로).

---

## 6. 소스 구성

| 파일 | 내용 |
|------|------|
| `Core/Src/filters.c` / `Core/Inc/filters.h` | FIR/IIR 계수 설계 + 실행 (핵심 알고리즘) |
| `Core/Src/app_tasks.c` / `.h` | 블록 DMA 수집, FIR/IIR/FFT 파이프라인, 3 태스크, UART 프로토콜 |
| `Core/Src/adc.c` | ADC1 8-bit + DMA(순환/MemInc) + TIM3 트리거 설정 |
| `Core/Src/tim.c` | TIM3 페이싱(≈8842 Hz, TRGO) |
| `Drivers/CMSIS/DSP` | (트리밍된) CMSIS-DSP — FFT/필터 함수 |

> **CMSIS-DSP 주의:** 본 프로젝트의 `Drivers/CMSIS/DSP` 는 실제 사용하는 폴더만 남긴
> **트리밍 사본**(STM32Cube_FW_F4 **V1.28.3** 기준)이다. 없는 함수를 쓰면 컴파일이 아니라
> **링커 오류**(`undefined reference`)가 난다. 같은 버전의 소스 폴더를 추가하면 된다.

---
