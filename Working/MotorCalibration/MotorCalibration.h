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

// Core Module
#include "../../Modules/Core/Core.h"

#endif
