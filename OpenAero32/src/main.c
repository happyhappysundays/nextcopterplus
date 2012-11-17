// **************************************************************************
// OpenAero32 software for AfroFlight32
// ====================================
// Version 1.00 Alpha 1 - September 2012
//
// Based on baseflight by timecop and others
// OpenAero code by David Thompson, included open-source code as per quoted references
// Includes PID and Auto-level functions inspired by the open-sourced MultiWii project
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2012 David Thompson
// * 
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// * 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// * 
// * You should have received a copy of the GNU General Public License
// * along with this program.If not, see <http://www.gnu.org/licenses/>.
// * 
// * NB: Summary - all derivative code MUST be released with the source code!
// *
// **************************************************************************
// Version History
// ===============
// V1.00a	Based on Baseflight r208 code
//			Initial code base.

#include "board.h"
#include "mw.h"

extern uint8_t useServo;
extern rcReadRawDataPtr rcReadRawFunc;

// two receiver read functions
extern uint16_t pwmReadRawRC(uint8_t chan);
extern uint16_t spektrumReadRawRC(uint8_t chan);

// For printf implementation via UART
static void _putc(void *p, char c)
{
    uartWrite(c);
}


int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;

    systemInit();
    init_printf(NULL, _putc);
    readEEPROM();
    checkFirstTime(false);
    serialInit(cfg.serial_baudrate);

    // We have these sensors
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);

    mixerInit(); // this will set useServo var depending on mixer type

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.motor_pwm_rate;
    pwm_params.servoPwmRate = cfg.servo_pwm_rate;

    pwmInit(&pwm_params);

    // configure PWM/CPPM read function. spektrum will override that
    rcReadRawFunc = pwmReadRawRC;

    // Flash LEDS encouragingly at the user
		LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsAutodetect();
    imuInit(); // Mag is initialized inside imuInit

    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit();

    // Init spektrum if fitted
		if (feature(FEATURE_SPEKTRUM)) {
        spektrumInit();
        rcReadRawFunc = spektrumReadRawRC;
    } else {
        // spektrum and GPS are mutually exclusive
        // Optional GPS - available only when using PPM, otherwise required pins won't be usable
        if (feature(FEATURE_PPM)) {
            if (feature(FEATURE_GPS))
                gpsInit(cfg.gps_baudrate);
        }
    }

    previousTime = micros();
		
		// Do wierd calibration thing
    if (cfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = 400;
    calibratingG = 1000;
    f.SMALL_ANGLES_25 = 1;

    // loopy
    while (1) {
        loop();
    }
}

void HardFault_Handler(void)
{
    // Fall out of the sky - Would not a soft reset be an better idea?
    writeAllMotors(cfg.mincommand);
    while (1);
}
