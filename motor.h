#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "bdc_motor.h"
#include <driver/gpio.h>

#define MAX_SPEED 100
#define _SMOOTH_PRD 200

int _accelerationTick = 30;

int _dutyS1 = 0;
int _dutyS2 = 0;
int _dutyS3 = 0;
int _dutyS4 = 0;

unsigned long _tmr1;
unsigned long _tmr2;
unsigned long _tmr3;
unsigned long _tmr4;

static const char *TAG_MOTORS = "MOTORS";

int __MOTORS_COUNT = 0; // Do NOT change this value

_Bool device_connected = false;

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000
#define BDC_MCPWM_FREQ_HZ             25000

#define nil ((void *)0)

int constrain(int value, int leftBorder, int rightBorder) {
    return value < leftBorder ? leftBorder : value > rightBorder ? rightBorder : value;
}

int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return constrain((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow, toLow, toHigh);
}

void init_motor(bdc_motor_handle_t *motor, gpio_num_t mcpwm_gpio_a, gpio_num_t mcpwm_gpio_b, uint_fast32_t pwm_frequency,
           uint_fast32_t timer_resolution, int group_id) {

    ESP_LOGI(TAG_MOTORS, "Create DC motor");

    bdc_motor_config_t motor_config = {
            .pwm_freq_hz = pwm_frequency,
            .pwma_gpio_num = mcpwm_gpio_a,
            .pwmb_gpio_num = mcpwm_gpio_b,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
            .group_id = group_id,
            .resolution_hz = timer_resolution,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, motor));
    __MOTORS_COUNT++;
}

void enable_all_motors(bdc_motor_handle_t motors[]) {
    ESP_LOGI(TAG_MOTORS, "Enable motors");
    for (int i = 0; i < __MOTORS_COUNT; i++) {
        ESP_ERROR_CHECK(bdc_motor_enable(motors[i]));
    }
}

void stop_motor(bdc_motor_handle_t motor) {
    ESP_ERROR_CHECK(bdc_motor_brake(motor));
}

void set_speed_for_motor(bdc_motor_handle_t motor, __uint32_t speed_value) {
    if (speed_value == 0) {
        stop_motor(motor);
    } else {
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor, (400 - MAX_SPEED) + speed_value));
    }
}

void run_motor_smooth1(bdc_motor_handle_t motor, int_fast32_t duty) {

    if ((esp_timer_get_time() / 1000ULL) - _tmr1 >= _SMOOTH_PRD) {
        _tmr1 = esp_timer_get_time() / 1000ULL;
        if (abs(_dutyS1 - duty) > _accelerationTick) _dutyS1 += ((_dutyS1 < duty) ? _accelerationTick : -_accelerationTick);
        else _dutyS1 = duty;
        set_speed_for_motor(motor, abs(_dutyS1));
        if (duty > 0) ESP_ERROR_CHECK(bdc_motor_forward(motor));
        else if (duty < 0) ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        else stop_motor(motor);
    }

}
void run_motor_smooth2(bdc_motor_handle_t motor, int_fast32_t duty) {

    if ((esp_timer_get_time() / 1000ULL) - _tmr2 >= _SMOOTH_PRD) {
        _tmr2 = esp_timer_get_time() / 1000ULL;
        if (abs(_dutyS2 - duty) > _accelerationTick) _dutyS2 += ((_dutyS2 < duty) ? _accelerationTick : -_accelerationTick);
        else _dutyS2 = duty;
        set_speed_for_motor(motor, abs(_dutyS2));
        if (duty > 0) ESP_ERROR_CHECK(bdc_motor_forward(motor));
        else if (duty < 0) ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        else stop_motor(motor);
    }

}
void run_motor_smooth3(bdc_motor_handle_t motor, int_fast32_t duty) {

    if ((esp_timer_get_time() / 1000ULL) - _tmr3 >= _SMOOTH_PRD) {
        _tmr3 = esp_timer_get_time() / 1000ULL;
        if (abs(_dutyS3 - duty) > _accelerationTick) _dutyS3 += ((_dutyS3 < duty) ? _accelerationTick : -_accelerationTick);
        else _dutyS3 = duty;
        set_speed_for_motor(motor, abs(_dutyS3));
        if (duty > 0) ESP_ERROR_CHECK(bdc_motor_forward(motor));
        else if (duty < 0) ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        else stop_motor(motor);
    }

}
void run_motor_smooth4(bdc_motor_handle_t motor, int_fast32_t duty) {

    if ((esp_timer_get_time() / 1000ULL) - _tmr4 >= _SMOOTH_PRD) {
        _tmr4 = esp_timer_get_time() / 1000ULL;
        if (abs(_dutyS4 - duty) > _accelerationTick) _dutyS4 += ((_dutyS4 < duty) ? _accelerationTick : -_accelerationTick);
        else _dutyS4 = duty;
        set_speed_for_motor(motor, abs(_dutyS4));
        if (duty > 0) ESP_ERROR_CHECK(bdc_motor_forward(motor));
        else if (duty < 0) ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        else stop_motor(motor);
    }

}


void run_motor(bdc_motor_handle_t motor, __INT_FAST32_TYPE__ speed_value) {
    set_speed_for_motor(motor, abs(speed_value));
    if (speed_value > 0) ESP_ERROR_CHECK(bdc_motor_forward(motor));
    else if (speed_value < 0) ESP_ERROR_CHECK(bdc_motor_reverse(motor));
    else stop_motor(motor);
}

