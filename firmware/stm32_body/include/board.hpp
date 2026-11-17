// Pin map and calibration for the body controller. docs/wiring.md has the
// same table with the other end of every wire.
//
//   What                      Pin    Peripheral      Goes to
//   ------------------------  -----  --------------  ------------------------
//   Front-left motor  IN1     PA6    TIM3_CH1        MX1616 #1  IN1
//   Front-left motor  IN2     PA7    TIM3_CH2        MX1616 #1  IN2
//   Middle-left motor IN1     PB0    TIM3_CH3        MX1616 #1  IN3
//   Middle-left motor IN2     PB1    TIM3_CH4        MX1616 #1  IN4
//   Rear-left motor   IN1     PB6    TIM4_CH1        MX1616 #2  IN1
//   Rear-left motor   IN2     PB7    TIM4_CH2        MX1616 #2  IN2
//   Front-right motor IN1     PB8    TIM4_CH3        MX1616 #2  IN3
//   Front-right motor IN2     PB9    TIM4_CH4        MX1616 #2  IN4
//   Middle-right motor IN1    PA5    TIM2_CH1        MX1616 #3  IN1
//   Middle-right motor IN2    PA1    TIM2_CH2        MX1616 #3  IN2
//   Rear-right motor  IN1     PB10   TIM2_CH3        MX1616 #3  IN3
//   Rear-right motor  IN2     PA3    TIM2_CH4        MX1616 #3  IN4
//   Steering servo, left      PA8    TIM1_CH1        servo signal (orange)
//   Steering servo, right     PA2    TIM9_CH1        servo signal (orange)
//   HC-SR04 TRIG              PB12   GPIO out
//   HC-SR04 ECHO              PB13   EXTI13          via 1k/2k divider (5 V -> 3.3 V)
//   Link to ESP32 TX          PA9    USART1_TX       ESP32 GPIO16 (RX2)
//   Link to ESP32 RX          PA10   USART1_RX       ESP32 GPIO17 (TX2)
//   Status LED                PC13   GPIO out        on-board blue LED (active low)
#pragma once

#include <cstdint>

#include "stm32f4xx_hal.h"

namespace board {

constexpr uint32_t kLinkBaud = 115200;

// Motor PWM: 100 MHz timer clock / 5000 = 20 kHz, above hearing range.
// If an MX1616 runs warm, try 10 kHz (kPwmPeriod = 9999).
constexpr uint32_t kPwmPeriod = 4999;

// Wheel order matches rover::geo: FL, ML, RL, FR, MR, RR.
// The right-hand motors are mounted mirror-image, so "forward" spins them
// the other way. If a wheel runs backwards after wiring, flip its entry here
// rather than swapping wires.
constexpr bool kMotorInvert[6] = {false, false, false, true, true, true};

// Steering servos (SG90 / MG90S style: 1500 us centre, ~1000 us per 90 deg).
// Trim the centre until the wheel points straight with the rover stopped.
// Flip the sign if positive angles turn the wheel to the right.
constexpr float kServoCentreUs[2] = {1500.0f, 1500.0f};
constexpr float kServoUsPerRad[2] = {636.6f, 636.6f};
constexpr float kServoSign[2] = {+1.0f, +1.0f};
constexpr float kServoMinUs = 600.0f;
constexpr float kServoMaxUs = 2400.0f;

#define BOARD_TRIG_PORT GPIOB
#define BOARD_TRIG_PIN GPIO_PIN_12
#define BOARD_ECHO_PORT GPIOB
#define BOARD_ECHO_PIN GPIO_PIN_13
#define BOARD_LED_PORT GPIOC
#define BOARD_LED_PIN GPIO_PIN_13

}  // namespace board
