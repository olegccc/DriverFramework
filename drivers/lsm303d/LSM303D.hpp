#pragma once

#define __IMU_USE_I2C
#include "ImuSensor.hpp"

namespace DriverFramework
{
/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM			0x49

#define ADDR_OUT_TEMP_L			0x05
#define ADDR_OUT_TEMP_H			0x06
#define ADDR_STATUS_M			0x07
#define ADDR_OUT_X_L_M          	0x08
#define ADDR_OUT_X_H_M          	0x09
#define ADDR_OUT_Y_L_M          	0x0A
#define ADDR_OUT_Y_H_M			0x0B
#define ADDR_OUT_Z_L_M			0x0C
#define ADDR_OUT_Z_H_M			0x0D

#define ADDR_INT_CTRL_M			0x12
#define ADDR_INT_SRC_M			0x13
#define ADDR_REFERENCE_X		0x1c
#define ADDR_REFERENCE_Y		0x1d
#define ADDR_REFERENCE_Z		0x1e

#define ADDR_STATUS_A			0x27
#define ADDR_OUT_X_L_A			0x28
#define ADDR_OUT_X_H_A			0x29
#define ADDR_OUT_Y_L_A			0x2A
#define ADDR_OUT_Y_H_A			0x2B
#define ADDR_OUT_Z_L_A			0x2C
#define ADDR_OUT_Z_H_A			0x2D

#define ADDR_CTRL_REG0			0x1F
#define ADDR_CTRL_REG1			0x20
#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define ADDR_CTRL_REG5			0x24
#define ADDR_CTRL_REG6			0x25
#define ADDR_CTRL_REG7			0x26

#define ADDR_FIFO_CTRL			0x2e
#define ADDR_FIFO_SRC			0x2f

#define ADDR_IG_CFG1			0x30
#define ADDR_IG_SRC1			0x31
#define ADDR_IG_THS1			0x32
#define ADDR_IG_DUR1			0x33
#define ADDR_IG_CFG2			0x34
#define ADDR_IG_SRC2			0x35
#define ADDR_IG_THS2			0x36
#define ADDR_IG_DUR2			0x37
#define ADDR_CLICK_CFG			0x38
#define ADDR_CLICK_SRC			0x39
#define ADDR_CLICK_THS			0x3a
#define ADDR_TIME_LIMIT			0x3b
#define ADDR_TIME_LATENCY		0x3c
#define ADDR_TIME_WINDOW		0x3d
#define ADDR_ACT_THS			0x3e
#define ADDR_ACT_DUR			0x3f

#define REG1_RATE_BITS_A		((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_POWERDOWN_A		((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A		((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A		((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A		((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A		((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_BDU_UPDATE			(1<<3)
#define REG1_Z_ENABLE_A			(1<<2)
#define REG1_Y_ENABLE_A			(1<<1)
#define REG1_X_ENABLE_A			(1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A	((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A		((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A		((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A		((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A		((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A	((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A	((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A	((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A	((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A	((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A	((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T			(1<<7)

#define REG5_RES_HIGH_M			((1<<6) | (1<<5))
#define REG5_RES_LOW_M			((0<<6) | (0<<5))

#define REG5_RATE_BITS_M		((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M		((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M		((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M		((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M		((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M		((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M		((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M	((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M	((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M	((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M	((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M	((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M	((1<<6) | (1<<5))

#define REG7_CONT_MODE_M		((0<<1) | (0<<0))

#define REG_STATUS_A_NEW_ZYXADA		0x08

#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G			8
#define LSM303D_ACCEL_DEFAULT_RATE			800
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define LSM303D_ACCEL_MAX_OUTPUT_RATE			280

#define LSM303D_MAG_DEFAULT_RANGE_GA			2
#define LSM303D_MAG_DEFAULT_RATE			100

#define LSM303D_ONE_G					9.80665f

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// update frequency 1000 Hz
#define LSM303D_MEASURE_INTERVAL_US 1000
#define LSM303D_SLAVE_ADDRESS 0x1D
#define LSM303D_BUS_FREQUENCY_IN_KHZ 400
#define LSM303D_TRANSFER_TIMEOUT_IN_USECS 900

#pragma pack(push, 1)

    struct packet {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    };
#pragma pack(pop)

    class LSM303D: public ImuSensor
    {
    public:
        LSM303D(const char *device_path, bool mag_enabled = true) :
                ImuSensor(device_path, LSM303D_MEASURE_INTERVAL_US, mag_enabled),
                _mag_enabled(mag_enabled),
                _last_temp_c(0.0f),
                _temp_initialized(false)
        {
        }

        // @return 0 on success, -errno on failure
        int writeReg(int reg, uint8_t val)
        {
            return _writeReg(reg, val);
        }

        int readReg(uint8_t address, uint8_t &val)
        {
            return _readReg(address, val);
        }

        // @return 0 on success, -errno on failure
        virtual int start();

        // @return 0 on success, -errno on failure
        virtual int stop();

    protected:
        virtual void _measure();
        virtual int _publish(struct imu_sensor_data &data);

    private:

        // @returns 0 on success, -errno on failure
        int lsm303d_init();

        // @return the number of FIFO bytes to collect
        int get_fifo_count();

        void accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth);
        void accel_set_samplerate(unsigned frequency);
        void mag_set_samplerate(unsigned frequency);

        void reset_fifo();
        void set_acc_scale(int scale);
        void set_mag_scale(int scale);

        bool _mag_enabled;
        float _last_temp_c;
        bool _temp_initialized;
        float _acc_scale;
        float _mag_scale;
    };

}

// namespace DriverFramework
