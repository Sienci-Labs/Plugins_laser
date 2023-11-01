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
static on_spindle_selected_ptr on_spindle_selected;
static spindle_state_t laser_state;
static spindle_id_t laser_id = -1;
static spindle_ptrs_t *spindle_hal = NULL;

static bool pwmEnabled = false;
static spindle_pwm_t laser_pwm;
static void laser_set_speed (uint_fast16_t pwm_value);

typedef struct {
    float rpm_max;
    float rpm_min;
    float pwm_freq;
    float pwm_off_value;
    float pwm_min_value;
    float pwm_max_value;
} laser_settings_t;

laser_settings_t laser_pwm_settings;
static nvs_address_t nvs_address;

static const setting_detail_t laser_settings[] = {
     { Setting_Laser_RpmMax, Group_Spindle, "Maximum laser speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &laser_pwm_settings.rpm_max, NULL, NULL },
     { Setting_Laser_RpmMin, Group_Spindle, "Minimum laser speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &laser_pwm_settings.rpm_min, NULL, NULL },
     { Setting_Laser_PWMFreq, Group_Spindle, "Laser PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL, Setting_IsExtended, &laser_pwm_settings.pwm_freq, NULL, NULL },
     { Setting_Laser_PWMOffValue, Group_Spindle, "Laser PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &laser_pwm_settings.pwm_off_value, NULL, NULL },
     { Setting_Laser_PWMMinValue, Group_Spindle, "Laser PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &laser_pwm_settings.pwm_min_value, NULL, NULL },
     { Setting_Laser_PWMMaxValue, Group_Spindle, "Laser PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &laser_pwm_settings.pwm_max_value, NULL, NULL },
     
};

static const setting_descr_t laser_settings_descr[] = {
    { Setting_Laser_RpmMax, "Maximum S word speed for laser" },
    { Setting_Laser_RpmMin, "Minimum S word speed for laser" },
    { Setting_Laser_PWMFreq, "Laser PWM frequency" },
    { Setting_Laser_PWMOffValue, "Laser PWM off value in percent (duty cycle)." },    
    { Setting_Laser_PWMMinValue, "Laser PWM min value in percent (duty cycle)." },
    { Setting_Laser_PWMMaxValue, "Laser PWM max value in percent (duty cycle)." },    
};

// Write settings to non volatile storage (NVS).
static void laser_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&laser_pwm_settings, sizeof(laser_pwm_settings), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void laser_settings_restore (void)
{
    laser_pwm_settings.pwm_freq = 449;
    laser_pwm_settings.pwm_max_value = 100;
    laser_pwm_settings.pwm_min_value = 0;
    laser_pwm_settings.pwm_off_value = 0;
    laser_pwm_settings.rpm_max = 255;
    laser_pwm_settings.rpm_min = 0;
    
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&laser_pwm_settings, sizeof(laser_pwm_settings), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void laser_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&laser_pwm_settings, nvs_address, sizeof(laser_pwm_settings), true) != NVS_TransferResult_OK)
        laser_settings_restore();
}

static setting_details_t laser_details = {
    .settings = laser_settings,
    .n_settings = sizeof(laser_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = laser_settings_descr,
    .n_descriptions = sizeof(laser_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = laser_settings_save,
    .load = laser_settings_load,
    .restore = laser_settings_restore
};
// Static spindle (off, on cw & on ccw)
inline static void laser_off (void)
{
#ifdef LASER_ENABLE_PIN
    DIGITAL_OUT(LASER_ENABLE_PORT, LASER_ENABLE_PIN, 0);
#endif
}

inline static void laser_on (void)
{
#ifdef LASER_ENABLE_PIN
    DIGITAL_OUT(LASER_ENABLE_PORT, LASER_ENABLE_PIN, 1);
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

static inline uint_fast16_t invert_pwm (spindle_pwm_t *pwm_data, uint_fast16_t pwm_value)
{
    return pwm_data->invert_pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
}

static bool laser_precompute_pwm_values (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_data, uint32_t clock_hz)
{
    if(spindle->rpm_max > spindle->rpm_min) {
        pwm_data->rpm_min = spindle->rpm_min;
        pwm_data->period = (uint_fast16_t)((float)clock_hz / laser_pwm_settings.pwm_freq);
        if(laser_pwm_settings.pwm_off_value == 0.0f)
            pwm_data->off_value = pwm_data->invert_pwm ? pwm_data->period : 0;
        else
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * laser_pwm_settings.pwm_off_value / 100.0f));
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * laser_pwm_settings.pwm_min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * laser_pwm_settings.pwm_max_value / 100.0f) + pwm_data->offset;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (spindle->rpm_max - spindle->rpm_min);
        pwm_data->always_on = laser_pwm_settings.pwm_off_value != 0.0f;
    }
    return spindle->rpm_max > spindle->rpm_min;
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
    if((laser->cap.variable = !settings.spindle.flags.pwm_disable && laser_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler))) {
#else
    if((laser->cap.variable = !settings.spindle.flags.pwm_disable && laser_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler))) {
#endif

        while(laser_pwm.period > 65534) {
            prescaler++;
#if LASER_PWM_TIMER_N == 1
            laser_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
            laser_precompute_pwm_values(laser, &laser_pwm, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
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

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == laser_id) {

        spindle_hal = spindle;
        //spindle_hal->type = SpindleType_PWM;
        //spindle_hal->cap.variable = On;
        //spindle_hal->cap.variable = On;
        //spindle_hal->cap.pwm_invert = On;
        //spindle_hal->cap.rpm_range_locked = On;
        spindle_hal->rpm_max = laser_pwm_settings.rpm_max;
        spindle_hal->rpm_min = laser_pwm_settings.rpm_min;
        //spindle_hal->config = laserConfig;
        //spindle_hal->get_pwm = laserGetPWM;
        //spindle_hal->update_pwm = laser_set_speed;
        //spindle_hal->set_state = laserSetState;
        //spindle_hal->get_state = laserGetState;

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

void pwm_switch_init (void)
{
    //initialize and register the laser PWM spindle.

    if((nvs_address = nvs_alloc(sizeof(laser_pwm_settings)))) {

        settings_register(&laser_details);
        //laser_settings_load();
    }  

    static const spindle_ptrs_t laser = {
 #ifdef LASER_PWM_TIMER_N
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.rpm_range_locked = On,
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

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;        

    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
