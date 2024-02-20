#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 13 //Set GPIO 13 as PWM0A

#include "driver/mcpwm.h"

#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

static void mcpwm_example_gpio_initialize(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void open_servo(void)
{
    uint32_t angle;
    angle = servo_per_degree_init(45);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay to allow the servo to reach the position
}

void close_servo(void)
{
    uint32_t angle;
    angle = servo_per_degree_init(0);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay to allow the servo to reach the position
}

void app_main(void)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e., for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    uint32_t s = 0;
    // Continuously open and close the servo
    while (s<20) {
        open_servo();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
        close_servo();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
        s++;
    }
};

// #define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
// #define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
// #define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

// static void mcpwm_example_gpio_initialize(void)
// {
//     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
// }

// static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
// {
//     uint32_t cal_pulsewidth = 0;
//     cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
//     return cal_pulsewidth;
// }

// void mcpwm_example_servo_control(void *arg)
// {
//     uint32_t angle, count;
//     //1. mcpwm gpio initialization
//     mcpwm_example_gpio_initialize();

//     //2. initial mcpwm configuration
//     mcpwm_config_t pwm_config;
//     pwm_config.frequency = 50;    //frequency = 50Hz, i.e., for every servo motor time period should be 20ms
//     pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
//     pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
//     pwm_config.counter_mode = MCPWM_UP_COUNTER;
//     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

//     while (1) {
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
//             angle = servo_per_degree_init(count);
//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
//             vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//         }
//     }
// }

// void app_main(void)
// {
//     xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
// }