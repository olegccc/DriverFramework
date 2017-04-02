/**
 * @file pca9685.cpp
 *
 * Driver for the PCA9685 I2C PWM module
 * The chip is used on the Adafruit I2C/PWM converter https://www.adafruit.com/product/815
 *
 * Parts of the code are adapted from the arduino library for the board
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * for the license of these parts see the
 * arduino_Adafruit_PWM_Servo_Driver_Library_license.txt file
 * see https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library for contributors
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <string.h>
#include "DriverFramework.hpp"
#include "PCA9685.hpp"

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OF

#define PCA9685_BUS_BUS_FREQUENCY_IN_KHZ 400
#define PCA9685_BUS_TRANSFER_TIMEOUT_IN_USECS 500
#define PCA9685_BUS_SLAVE_ADDRESS 0x40

#define PCA9685_PWMFREQ 60.0f
#define PCA9685_NCHANS 16 // total amount of pwm outputs

#define PCA9685_PWMMIN 150 // this is the 'minimum' pulse length count (out of 4096)
#define PCA9685_PWMMAX 600 // this is the 'maximum' pulse length count (out of 4096)_PWMFREQ 60.0f

#define PCA9685_PWMCENTER ((PCA9685_PWMMAX + PCA9685_PWMMIN)/2)
#define PCA9685_MAXSERVODEG 90.0f /* maximal servo deflection in degrees
				     PCA9685_PWMMIN <--> -PCA9685_MAXSERVODEG
				     PCA9685_PWMMAX <--> PCA9685_MAXSERVODEG
				     */
#define PCA9685_SCALE ((PCA9685_PWMMAX - PCA9685_PWMCENTER)/(M_DEG_TO_RAD_F * PCA9685_MAXSERVODEG)) // scales from rad to PWM

int PCA9685::pca9685_init()
{
    int ret;

    ret = setPWMFreq(PCA9685_PWMFREQ);

    memset(_current_values, 0, sizeof(_current_values));

    return ret;
}

int PCA9685::start()
{
    int result = I2CDevObj::start();

    if (result < 0) {
        DF_LOG_ERR("DevObj start failed");
        DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
        return result;
    }

    result = _setSlaveConfig(PCA9685_BUS_SLAVE_ADDRESS,
                             PCA9685_BUS_BUS_FREQUENCY_IN_KHZ,
                             PCA9685_BUS_TRANSFER_TIMEOUT_IN_USECS);

    if (result < 0) {
        DF_LOG_ERR("Could not set slave config");
        return result;
    }

    result = pca9685_init();

    if (result < 0) {
        DF_LOG_ERR("Unable to initialize driver");
        return result;
    }

    return 0;
}

int PCA9685::stop()
{
    int result = I2CDevObj::stop();

    if (result != 0) {
        DF_LOG_ERR("DevObj stop failed");
        return result;
    }

    return 0;
}

/**
 * Main loop function
 */
void PCA9685::pwm(const float* controls, int length)
{
    for (int i = 0; i < length; i++) {
        /* Scale the controls to PWM, first multiply by pi to get rad,
         * the control[i] values are on the range -1 ... 1 */
        uint16_t new_value = PCA9685_PWMCENTER + (controls[i] * M_PI_F * PCA9685_SCALE);
        DF_LOG_INFO("%d: current: %u, new %u, control %.2f", i, _current_values[i], new_value, (double) controls[i]);

        if (new_value != _current_values[i] &&
            isfinite(new_value) &&
            new_value >= PCA9685_PWMMIN &&
            new_value <= PCA9685_PWMMAX) {
            /* This value was updated, send the command to adjust the PWM value */
            setPin(i, new_value);
            _current_values[i] = new_value;
        }
    }
}

int PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    int ret;
    /* convert to correct message */

    uint8_t msg[4];
    msg[0] = on;
    msg[1] = on >> 8;
    msg[2] = off;
    msg[3] = off >> 8;

    /* try i2c transfer */
    ret = _writeReg(LED0_ON_L + 4 * num, msg, 4);

    if (ret != 0) {
        perf_count(_comms_errors);
        DF_LOG_ERR("i2c::transfer returned %d", ret);
    }

    return ret;
}

int PCA9685::setPin(uint8_t num, uint16_t val, bool invert)
{
    // Clamp value between 0 and 4095 inclusive.
    if (val > 4095) {
        val = 4095;
    }

    if (invert) {
        if (val == 0) {
            // Special value for signal fully on.
            return setPWM(num, 4096, 0);

        } else if (val == 4095) {
            // Special value for signal fully off.
            return setPWM(num, 0, 4096);

        } else {
            return setPWM(num, 0, 4095 - val);
        }

    } else {
        if (val == 4095) {
            // Special value for signal fully on.
            return setPWM(num, 4096, 0);

        } else if (val == 0) {
            // Special value for signal fully off.
            return setPWM(num, 0, 4096);

        } else {
            return setPWM(num, 0, val);
        }
    }
}

int PCA9685::setPWMFreq(float freq)
{
    int ret  = OK;
    freq *= 0.9f;  /* Correct for overshoot in the frequency setting (see issue
		https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/11). */
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = uint8_t(prescaleval + 0.5f); //implicit floor()
    uint8_t oldmode;
    ret = _readReg(PCA9685_MODE1, oldmode);

    if (ret != 0) {
        return ret;
    }

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep

    ret = _writeReg(PCA9685_MODE1, newmode); // go to sleep

    if (ret != 0) {
        return ret;
    }

    ret = _writeReg(PCA9685_PRESCALE, prescale); // set the prescaler

    if (ret != 0) {
        return ret;
    }

    ret = _writeReg(PCA9685_MODE1, oldmode);

    if (ret != 0) {
        return ret;
    }

    usleep(5000); //5ms delay (from arduino driver)

    ret = _writeReg(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.

    if (ret != 0) {
        return ret;
    }

    return 0;
}

int PCA9685::reset()
{
    warnx("resetting");
    return _writeReg(PCA9685_MODE1, 0);
}
