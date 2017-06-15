#ifndef MOTORCALIBRATION_H
#define MOTORCALIBRATION_H

#define  DEBUG

#include <pca9685.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdio>
#include <stdlib.h>
#include <Python.h>
#include <time.h>
#include <signal.h>		// capture control-c


#define PCA9685_ADDR 0x40
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 400
#define PCA9685_ADDR 0x40

// Motor Constants
#define BASE_SPEED     10     // 10 percent
#define MOTOR_DEADZONE 2      // 2 percent
#define PWM_LOW_LIMIT  1940   // PWM value
#define PWM_HIGH_LIMIT 3354   // PWM value
#define PWM_ZERO_VALUE 2647   // PWM value

// Core Module
#include "../../Modules/Core/Core.h"

#endif
