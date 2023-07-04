/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "math.h"
#include "stdio.h"

// Pin definitions
#define RIGHT_MOTOR_DIR_REVERSE GPIO_NUM_21
#define RIGHT_MOTOR_DIR_FORWARD GPIO_NUM_23
#define RIGHT_MOTOR_PWM         GPIO_NUM_19

#define LEFT_MOTOR_DIR_REVERSE  GPIO_NUM_25
#define LEFT_MOTOR_DIR_FORWARD  GPIO_NUM_33
#define LEFT_MOTOR_PWM          GPIO_NUM_32

#define MOTOR_DIR_PIN_SEL ((1ULL<<RIGHT_MOTOR_DIR_REVERSE) | (1ULL<<RIGHT_MOTOR_DIR_FORWARD) | (1ULL<<LEFT_MOTOR_DIR_REVERSE) | (1ULL<<LEFT_MOTOR_DIR_FORWARD))

#define DEADBAND 8

void gpio_init()
{
    Console.printf("Motor control GPIO init\n");
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = MOTOR_DIR_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
}

void pwm_gpio_config()
{
    Console.printf("MCPWM GPIO init\n");
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = RIGHT_MOTOR_PWM,
        .mcpwm0b_out_num = LEFT_MOTOR_PWM,
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
}

void mcpwm_config()
{
    Console.printf("MCPWM init\n");
    mcpwm_config_t pwm_config;

    // low frequencies work better for driving the motor at low speeds
    pwm_config.frequency = 20;     // frequency = 20Hz
    pwm_config.cmpr_a = 0.0;       // initial duty cycle 0
    pwm_config.cmpr_b = 0.0;       // initial duty cycle 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void motor_control_setup()
{
    gpio_init();
    pwm_gpio_config();
    mcpwm_config();
}

void set_motor_pwm(mcpwm_generator_t motor, float pwm)
{
    // set direction
    if (motor == MCPWM_OPR_A) {
        gpio_set_level(RIGHT_MOTOR_DIR_FORWARD, pwm >= 0);
        gpio_set_level(RIGHT_MOTOR_DIR_REVERSE, pwm < 0);
    }
    if (motor == MCPWM_OPR_B) {
        gpio_set_level(LEFT_MOTOR_DIR_FORWARD, pwm >= 0);
        gpio_set_level(LEFT_MOTOR_DIR_REVERSE, pwm < 0);
    }

    float absolute_pwm = fabs(pwm);
    // TODO: protect motor_x_pwm variables with mutexes
    if (absolute_pwm > 100)
    {
        Console.printf("ERROR: Motor duty cycle cannot exceed 100 \n"); // TODO: make this a proper error log
        absolute_pwm = 0;
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, motor, absolute_pwm);
}

float limit_range(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    motor_control_setup();
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        float rightMotor = 0.0;
        float leftMotor = 0.0;

        if (myGamepad && myGamepad->isConnected()) {
            /*myGamepad->axisX(),        // (-511 - 512) left X Axis
            myGamepad->axisY(),        // (-511 - 512) left Y axis
            myGamepad->axisRX(),       // (-511 - 512) right X axis
            myGamepad->axisRY(),       // (-511 - 512) right Y axis
            myGamepad->brake(),        // (0 - 1023): brake button
            myGamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button*/

            /*if (myGamepad->throttle() > DEADBAND) {
                rightMotor = 20 + myGamepad->throttle() / 1023.0 * 80;
                leftMotor = 20 + myGamepad->throttle() / 1023.0 * 80;
            }

            if (myGamepad->brake() > DEADBAND) {
                rightMotor = -20 + myGamepad->brake() / 1023.0 * -80;
                leftMotor = -20 + myGamepad->brake() / 1023.0 * -80;
            }*/
            float leftThumbX = myGamepad->axisX();
            float leftThumbY = myGamepad->axisY();
            float rightThumbX = myGamepad->axisRX();
            float rightThumbY = myGamepad->axisRY();
            float x = rightThumbX;
            float y = rightThumbY;
            if (fabs(leftThumbX) + fabs(leftThumbY) > fabs(rightThumbX) + fabs(rightThumbY)) {
                x = leftThumbX;
                y = leftThumbY;
            }

            float thumbX = x / 512.0 * -100;
            float thumbY = y / 512.0 * -100;
            if (fabs(thumbX) < DEADBAND && fabs(thumbY) < DEADBAND)
            {
                thumbX = 0;
                thumbY = 0;
            }

            rightMotor = limit_range(thumbY + thumbX, -100, 100);
            leftMotor = limit_range(thumbY - thumbX, -100, 100);

            set_motor_pwm(MCPWM_OPR_A, rightMotor);
            set_motor_pwm(MCPWM_OPR_B, leftMotor);
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
    //delay(150);
}
