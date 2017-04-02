/****************************************************************************
*
*   Copyright (C) 2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#pragma once

#include "I2CDevObj.hpp"

#define PCA9685_SLAVE_ADDRESS		(0x30)
#define PCA9685_MEASURE_INTERVAL_US 20000

namespace DriverFramework
{
    class PCA9685 : public I2CDevObj
    {
    public:
        PCA9685(const char *device_path) :
                I2CDevObj("PCA9685", device_path, "/dev/pca9685", PCA9685_MEASURE_INTERVAL_US)
        {
        }

        void pwm(const float* controls, int length);

        // @return 0 on success, -errno on failure
        virtual int start();

        // @return 0 on success, -errno on failure
        virtual int stop();

        // @return 0 on success
        int probe();

    protected:
        virtual void _measure();
        virtual int _publish(struct range_sensor_data &data);

    private:
        // returns 0 on success, -errno on failure
        int pca9685_init();
        init reset();

        /**
         * Helper function to set the pwm frequency
         */
        int setPWMFreq(float freq);

        /**
         * Helper function to set the demanded pwm value
         * @param num pwm output number
         */
        int setPWM(uint8_t num, uint16_t on, uint16_t off);

        /**
         * Sets pin without having to deal with on/off tick placement and properly handles
         * a zero value as completely off.  Optional invert parameter supports inverting
         * the pulse for sinking to ground.
         * @param num pwm output number
         * @param val should be a value from 0 to 4095 inclusive.
         */
        int setPin(uint8_t num, uint16_t val, bool invert = false);

        uint16_t _current_values[16];
    };

}; // namespace DriverFramework
