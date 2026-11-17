// Black Pill (STM32F411CEU6) peripherals for the rover body.
//
// Clock: 25 MHz crystal -> PLL -> 100 MHz. If the crystal doesn't start
// (some clone boards), it falls back to the internal 16 MHz oscillator and
// still runs at 100 MHz, just with a slightly less accurate UART clock.
//
// Motor PWM uses MX1616 "slow decay": one input held high, the other
// PWM'd. While the PWM pin is high both inputs are high, which brakes the
// motor instead of letting it coast. That gives a speed that tracks duty
// almost linearly and, more importantly here, a rover that doesn't roll
// back off a step when you let go of the key.

#include "hw.hpp"

#include <cmath>

#include "board.hpp"
#include "stm32f4xx_hal.h"

namespace {

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim9;
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;

struct PwmPin {
  TIM_HandleTypeDef* tim;
  uint32_t channel;
};

// [motor][0] = IN1 / IN3, [motor][1] = IN2 / IN4
const PwmPin kMotorPins[6][2] = {
    {{&htim3, TIM_CHANNEL_1}, {&htim3, TIM_CHANNEL_2}},  // FL  PA6 PA7
    {{&htim3, TIM_CHANNEL_3}, {&htim3, TIM_CHANNEL_4}},  // ML  PB0 PB1
    {{&htim4, TIM_CHANNEL_1}, {&htim4, TIM_CHANNEL_2}},  // RL  PB6 PB7
    {{&htim4, TIM_CHANNEL_3}, {&htim4, TIM_CHANNEL_4}},  // FR  PB8 PB9
    {{&htim2, TIM_CHANNEL_1}, {&htim2, TIM_CHANNEL_2}},  // MR  PA5 PA1
    {{&htim2, TIM_CHANNEL_3}, {&htim2, TIM_CHANNEL_4}},  // RR  PB10 PA3
};
const PwmPin kServoPins[2] = {{&htim1, TIM_CHANNEL_1}, {&htim9, TIM_CHANNEL_1}};  // PA8, PA2

// --- UART receive ring buffer, filled from the interrupt -------------------
constexpr uint32_t kRxSize = 256;
volatile uint8_t rx_buf[kRxSize];
volatile uint32_t rx_head = 0;  // written by the ISR
volatile uint32_t rx_tail = 0;  // written by the main loop

// --- Sonar timing, captured in the EXTI interrupt -------------------------
volatile uint32_t echo_rise = 0, echo_width = 0;
volatile bool echo_rose = false, echo_fell = false;
uint32_t ping_ms = 0;
bool ping_pending = false;

void clock_init() {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitTypeDef osc = {};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 25;  // 25 MHz / 25 = 1 MHz into the PLL
  osc.PLL.PLLN = 200;
  osc.PLL.PLLP = RCC_PLLP_DIV2;  // 100 MHz
  osc.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    osc = RCC_OscInitTypeDef{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 200;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&osc);
  }

  RCC_ClkInitTypeDef clk = {};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 100 MHz
  clk.APB1CLKDivider = RCC_HCLK_DIV2;   // 50 MHz (TIM2/3/4 still see 100 MHz)
  clk.APB2CLKDivider = RCC_HCLK_DIV1;   // 100 MHz (TIM1/9)
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3);
}

void cycle_counter_init() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  const uint32_t start = DWT->CYCCNT;
  const uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while (DWT->CYCCNT - start < ticks) {
  }
}

void af_pin(GPIO_TypeDef* port, uint16_t pin, uint8_t af, uint32_t pull = GPIO_NOPULL) {
  GPIO_InitTypeDef g = {};
  g.Pin = pin;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = pull;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = af;
  HAL_GPIO_Init(port, &g);
}

void pwm_timer(TIM_HandleTypeDef& h, TIM_TypeDef* instance, uint32_t prescaler, uint32_t period,
               const uint32_t* channels, int n, uint32_t initial) {
  h.Instance = instance;
  h.Init.Prescaler = prescaler;
  h.Init.CounterMode = TIM_COUNTERMODE_UP;
  h.Init.Period = period;
  h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  h.Init.RepetitionCounter = 0;
  h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&h);

  TIM_OC_InitTypeDef oc = {};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = initial;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  for (int i = 0; i < n; ++i) {
    HAL_TIM_PWM_ConfigChannel(&h, &oc, channels[i]);
    HAL_TIM_PWM_Start(&h, channels[i]);  // also sets MOE on TIM1
  }
}

void motors_init() {
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  af_pin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF2_TIM3);
  af_pin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_AF2_TIM3);
  af_pin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF2_TIM4);
  af_pin(GPIOA, GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_3, GPIO_AF1_TIM2);
  af_pin(GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2);
  const uint32_t all[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
  const uint32_t brake = board::kPwmPeriod + 1;  // both inputs high = brake
  pwm_timer(htim2, TIM2, 0, board::kPwmPeriod, all, 4, brake);
  pwm_timer(htim3, TIM3, 0, board::kPwmPeriod, all, 4, brake);
  pwm_timer(htim4, TIM4, 0, board::kPwmPeriod, all, 4, brake);
}

void servos_init() {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  af_pin(GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1);
  af_pin(GPIOA, GPIO_PIN_2, GPIO_AF3_TIM9);
  const uint32_t ch1[] = {TIM_CHANNEL_1};
  // 100 MHz / 100 = 1 MHz tick, 20000 ticks = 50 Hz. Compare value = pulse in us.
  pwm_timer(htim1, TIM1, 99, 19999, ch1, 1, static_cast<uint32_t>(board::kServoCentreUs[0]));
  pwm_timer(htim9, TIM9, 99, 19999, ch1, 1, static_cast<uint32_t>(board::kServoCentreUs[1]));
}

void uart_init() {
  __HAL_RCC_USART1_CLK_ENABLE();
  af_pin(GPIOA, GPIO_PIN_9, GPIO_AF7_USART1);
  af_pin(GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, GPIO_PULLUP);  // idle high if the ESP32 is unplugged
  huart1.Instance = USART1;
  huart1.Init.BaudRate = board::kLinkBaud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void sonar_init() {
  GPIO_InitTypeDef g = {};
  g.Pin = BOARD_TRIG_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_TRIG_PORT, &g);
  HAL_GPIO_WritePin(BOARD_TRIG_PORT, BOARD_TRIG_PIN, GPIO_PIN_RESET);

  g.Pin = BOARD_ECHO_PIN;
  g.Mode = GPIO_MODE_IT_RISING_FALLING;
  g.Pull = GPIO_PULLDOWN;  // reads "no echo" if the sensor is missing
  HAL_GPIO_Init(BOARD_ECHO_PORT, &g);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void led_init() {
  GPIO_InitTypeDef g = {};
  g.Pin = BOARD_LED_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_PORT, &g);
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_SET);  // off
}

void watchdog_init() {
  // LSI ~32 kHz / 32 = ~1 kHz, so 300 counts is ~300 ms. If the main loop
  // ever hangs, the chip resets and the motors come back up braked.
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 300;
  HAL_IWDG_Init(&hiwdg);
}

inline void set_ccr(const PwmPin& p, uint32_t v) { __HAL_TIM_SET_COMPARE(p.tim, p.channel, v); }

}  // namespace

// ---- interrupt handlers (names come from the startup file) -----------------
extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

extern "C" void USART1_IRQHandler(void) {
  const uint32_t sr = USART1->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
    // Reading DR after SR also clears overrun/framing errors.
    const uint8_t b = static_cast<uint8_t>(USART1->DR & 0xFF);
    if (sr & USART_SR_RXNE) {
      const uint32_t next = (rx_head + 1) % kRxSize;
      if (next != rx_tail) {
        rx_buf[rx_head] = b;
        rx_head = next;
      }
    }
  }
}

extern "C" void EXTI15_10_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(BOARD_ECHO_PIN) != 0u) {
    __HAL_GPIO_EXTI_CLEAR_IT(BOARD_ECHO_PIN);
    const uint32_t now = DWT->CYCCNT;
    if (HAL_GPIO_ReadPin(BOARD_ECHO_PORT, BOARD_ECHO_PIN) == GPIO_PIN_SET) {
      echo_rise = now;
      echo_rose = true;
    } else if (echo_rose) {
      echo_width = now - echo_rise;
      echo_fell = true;
    }
  }
}

// ---- public API -------------------------------------------------------------
namespace hw {

void init() {
  HAL_Init();
  clock_init();
  cycle_counter_init();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  led_init();
  motors_init();
  servos_init();
  uart_init();
  sonar_init();
  watchdog_init();
}

uint32_t millis() { return HAL_GetTick(); }

void kick_watchdog() { HAL_IWDG_Refresh(&hiwdg); }

void led(bool on) { HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

void set_motor(int i, float duty) {
  if (i < 0 || i >= 6) return;
  if (board::kMotorInvert[i]) duty = -duty;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  const uint32_t full = board::kPwmPeriod + 1;  // compare > period: pin stays high
  const uint32_t off = static_cast<uint32_t>((1.0f - std::fabs(duty)) * static_cast<float>(full) + 0.5f);
  const PwmPin& in1 = kMotorPins[i][0];
  const PwmPin& in2 = kMotorPins[i][1];
  if (duty > 0.0f) {         // IN1 high, IN2 low for `duty` of the time
    set_ccr(in1, full);
    set_ccr(in2, off);
  } else if (duty < 0.0f) {  // mirror image
    set_ccr(in1, off);
    set_ccr(in2, full);
  } else {                   // both high: brake
    set_ccr(in1, full);
    set_ccr(in2, full);
  }
}

void brake_all() {
  for (int i = 0; i < 6; ++i) set_motor(i, 0.0f);
}

void set_servo(int k, float angle) {
  if (k < 0 || k > 1) return;
  float us = board::kServoCentreUs[k] + board::kServoSign[k] * board::kServoUsPerRad[k] * angle;
  if (us < board::kServoMinUs) us = board::kServoMinUs;
  if (us > board::kServoMaxUs) us = board::kServoMaxUs;
  set_ccr(kServoPins[k], static_cast<uint32_t>(us + 0.5f));
}

bool link_read(uint8_t& byte) {
  if (rx_tail == rx_head) return false;
  byte = rx_buf[rx_tail];
  rx_tail = (rx_tail + 1) % kRxSize;
  return true;
}

void link_write(const uint8_t* data, size_t n) {
  HAL_UART_Transmit(&huart1, const_cast<uint8_t*>(data), static_cast<uint16_t>(n), 10);
}

bool sonar_poll(float& range_m) {
  const uint32_t now = HAL_GetTick();
  if (ping_pending) {
    if (echo_fell) {
      ping_pending = false;
      const float us = static_cast<float>(echo_width) / static_cast<float>(SystemCoreClock / 1000000u);
      // Sound covers the distance twice. Pulses over ~25 ms are the
      // sensor's "nothing out there" timeout, not a real echo.
      range_m = us > 25000.0f ? 0.0f : us * 0.0001715f;
      return true;
    }
    if (now - ping_ms > 40) {
      ping_pending = false;
      if (echo_rose) {  // the sensor answered but never heard anything back
        range_m = 0.0f;
        return true;
      }
      return false;  // no response at all: unplugged or dead, stay quiet
    }
    return false;
  }
  if (now - ping_ms >= 60) {  // HC-SR04 wants >= 60 ms between pings
    // Some clones hold ECHO high for ages after a miss. Don't ping into that.
    if (HAL_GPIO_ReadPin(BOARD_ECHO_PORT, BOARD_ECHO_PIN) == GPIO_PIN_SET) return false;
    echo_rose = false;
    echo_fell = false;
    HAL_GPIO_WritePin(BOARD_TRIG_PORT, BOARD_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(BOARD_TRIG_PORT, BOARD_TRIG_PIN, GPIO_PIN_RESET);
    ping_ms = now;
    ping_pending = true;
  }
  return false;
}

}  // namespace hw
