// **************************************************************************
// OpenAero32 software for AfroFlight32
// ====================================
// Version 1.00 Alpha 1 - January 2013
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
// V1.00a	Based on Baseflight r240 code
//			Fixed aeroplane mixer and slow flaps.
//			Added many new config variables for flaps, RC and servo control.
//			Added calibrate cli commands for gyro, acc, mag and sticks.
//			Added stick calibration and per-channel RC zeros.
//			Added cli settings for servo trim and defaultrc.
//			Added cli settings dynamic PID channel and breakpoint.
//

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
	drv_adc_config_t adc_params;

    systemInit();
    init_printf(NULL, _putc);

    checkFirstTime(false);
	readEEPROM();

    // configure power ADC
    if (cfg.power_adc_channel > 0 && (cfg.power_adc_channel == 1 || cfg.power_adc_channel == 9))
        adc_params.powerAdcChannel = cfg.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        cfg.power_adc_channel = 0;
    }

    adcInit(&adc_params);

    serialInit(cfg.serial_baudrate);

    // We have these sensors
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);

    mixerInit(); // this will set useServo var depending on mixer type

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;

	pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SPEKTRUM); // spektrum support uses UART too
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.motor_pwm_rate;
    pwm_params.servoPwmRate = cfg.servo_pwm_rate;
    switch (cfg.power_adc_channel) {
        case 1:
            pwm_params.adcChannel = PWM2;
            break;
        case 9:
            pwm_params.adcChannel = PWM8;
            break;
        default:
            pwm_params.adcChannel = 0;
        break;
    }

    pwmInit(&pwm_params);

    // configure PWM/CPPM read function. spektrum below will override that
    rcReadRawFunc = pwmReadRawRC;
	
    // Init spektrum if fitted
	if (feature(FEATURE_SPEKTRUM)) 
	{
		spektrumInit();
		rcReadRawFunc = spektrumReadRawRC;
	} 
	else 
	{
		// Spektrum and GPS are mutually exclusive
		// Optional GPS - available in both PPM and PWM input mode, in PWM input, reduces number of available channels by 2.
		if (feature(FEATURE_GPS))
		{
			gpsInit(cfg.gps_baudrate);
		}
	}	
	
	
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
