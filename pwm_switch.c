/*

  pwm_switch.c - plugin for switching PWM output

  Part of grblHAL

  GRBLHAL is Copyright (c) 2020-2022 Terje Io

  Modifications copyright (c) 2023 Sienci Labs

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

/*I think this needs to be simplified down to the basics to drive the PWM from 0-100.  Just overwrite the settings
and store them in a temp variable, then restore when switching away?  There should be function pointers for 
spindle select.*/

#include "driver.h"

#if SIENCI_LASER_PWM

#include <string.h>
#include <math.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#endif

static on_report_options_ptr on_report_options;
static spindle_state_t laser_state;
static spindle_id_t laser_id = -1;
static uint8_t n_spindle = 0;

static bool pwmEnabled = false;
static spindle_pwm_t laser_pwm;
static void laser_set_speed (uint_fast16_t pwm_value);

// Static spindle (off, on cw & on ccw)

inline static void laser_off (void)
{
#ifdef LASER_ENABLE_PIN
    DIGITAL_OUT(LASER_ENABLE_PORT, LASER_ENABLE_PIN, 1);
#endif
}

inline static void laser_on (void)
{
#ifdef LASER_ENABLE_PIN
    DIGITAL_OUT(LASER_ENABLE_PORT, LASER_ENABLE_PIN, 0);
#endif
}

// Start or stop spindle
static void laserSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        laser_off();
    else {
        laser_on();
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t laserGetState (void)
{
    spindle_state_t state = laser_state;

#ifdef LASER_ENABLE_PIN
    state.on = DIGITAL_IN(LASER_ENABLE_PORT, LASER_ENABLE_PIN);
#endif
    state.value ^= settings.spindle.invert.mask;

    return state;
}

static uint_fast16_t laserGetPWM (float rpm){
    return spindle_compute_pwm_value(&laser_pwm, rpm, false);
}

// Start or stop laser
static void laserSetStateVariable (spindle_state_t state, float rpm)
{
    if(!settings.spindle.flags.enable_rpm_controlled) {
        if(state.on)
            laser_on();
        else
            laser_off();
    }

    laser_set_speed(state.on ? spindle_compute_pwm_value(&laser_pwm, rpm, false) : laser_pwm.off_value);


}

static bool laserConfig (spindle_ptrs_t *laser){
{
    if(laser == NULL)
        return false;

    RCC_ClkInitTypeDef clock;
    uint32_t latency, prescaler = 1;

    HAL_RCC_GetClockConfig(&clock, &latency);

#if LASER_PWM_TIMER_N == 1
    if((laser->cap.variable = !settings.spindle.flags.pwm_disable && spindle_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler))) {
#else
    if((laser->cap.variable = !settings.spindle.flags.pwm_disable && spindle_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler))) {
#endif

        while(laser_pwm.period > 65534) {
            prescaler++;
#if LASER_PWM_TIMER_N == 1
            spindle_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
            spindle_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
#endif
        }

        laser->set_state = laserSetStateVariable;

        LASER_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

        TIM_Base_InitTypeDef timerInitStructure = {
            .Prescaler = prescaler - 1,
            .CounterMode = TIM_COUNTERMODE_UP,
            .Period = laser_pwm.period - 1,
            .ClockDivision = TIM_CLOCKDIVISION_DIV1,
            .RepetitionCounter = 0
        };

        TIM_Base_SetConfig(LASER_PWM_TIMER, &timerInitStructure);

        LASER_PWM_TIMER->CCER &= ~LASER_PWM_CCER_EN;
        LASER_PWM_TIMER_CCMR &= ~LASER_PWM_CCMR_OCM_CLR;
        LASER_PWM_TIMER_CCMR |= LASER_PWM_CCMR_OCM_SET;
        LASER_PWM_TIMER_CCR = 0;
#if LASER_PWM_TIMER_N == 1
        LASER_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
#endif
        if(settings.spindle.invert.pwm) {
            LASER_PWM_TIMER->CCER |= LASER_PWM_CCER_POL;
            LASER_PWM_TIMER->CR2 |= LASER_PWM_CR2_OIS;
        } else {
            LASER_PWM_TIMER->CCER &= ~LASER_PWM_CCER_POL;
            LASER_PWM_TIMER->CR2 &= ~LASER_PWM_CR2_OIS;
        }
        LASER_PWM_TIMER->CCER |= LASER_PWM_CCER_EN;
        LASER_PWM_TIMER->CR1 |= TIM_CR1_CEN;

    } else {
        if(pwmEnabled)
            laser->set_state((spindle_state_t){0}, 0.0f);

        laser->set_state = laserSetState;
    }

    spindle_update_caps(laser, laser->cap.variable ? &laser_pwm : NULL);

    return true;
}

#if PPI_ENABLE

static void laserPulseOn (uint_fast16_t pulse_length)
{
    PPI_TIMER->ARR = pulse_length;
    PPI_TIMER->EGR = TIM_EGR_UG;
    PPI_TIMER->CR1 |= TIM_CR1_CEN;
    laser_on();
}

#endif
 
}

static void laser_set_speed (uint_fast16_t pwm_value){
    if (pwm_value == laser_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.enable_rpm_controlled)
            laser_off();
        if(laser_pwm.always_on) {
            LASER_PWM_TIMER_CCR = laser_pwm.off_value;
#if LASER_PWM_TIMER_N == 1
            LASER_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
            LASER_PWM_TIMER_CCR = pwm_value;
        } else
#if LASER_PWM_TIMER_N == 1
            LASER_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
#else
            LASER_PWM_TIMER_CCR = 0;
#endif
    } else {
        if(!pwmEnabled) {
            laser_on();
            pwmEnabled = true;
        }
        LASER_PWM_TIMER_CCR = pwm_value;
#if LASER_PWM_TIMER_N == 1
        LASER_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
    }    
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:SLB Laser PWM switch v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Laser PWM switch plugin failed to initialize!", Message_Warning);
}

void pwm_switch_init (void)
{
    //initialize and register the laser PWM spindle.

    static const spindle_ptrs_t laser = {
 #ifdef LASER_PWM_TIMER_N
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.pwm_invert = On,
        .config = laserConfig,
        .get_pwm = laserGetPWM,
        .update_pwm = laser_set_speed,
  #if PPI_ENABLE
        .pulse_on = laserPulseOn,
  #endif
 #else
        .type = SpindleType_Basic,
 #endif
        .set_state = laserSetState,
        .get_state = laserGetState
    };

    laser_id = spindle_register(&laser, "SLB_LASER");

    if(laser_id) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
