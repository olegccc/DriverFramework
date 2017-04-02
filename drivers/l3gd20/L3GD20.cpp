#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "L3GD20.hpp"

#define G_SI 9.80665f
#define PI 3.14159f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

using namespace DriverFramework;

void LS3GD20::set_gyro_scale(int scale)
{
    uint8_t bits = REG4_BDU;
    float new_range_scale_dps_digit;
    float new_range;

    if (max_dps == 0) {
        max_dps = 2000;
    }

    switch (scale) {
        case RANGE_250DPS:
            new_range = 250;
            bits |= RANGE_250DPS;
            new_range_scale_dps_digit = 8.75e-3f;
            break;
        case RANGE_500DPS:
            new_range = 500;
            bits |= RANGE_500DPS;
            new_range_scale_dps_digit = 17.5e-3f;
            break;
        case RANGE_2000DPS:
            new_range = 2000;
            bits |= RANGE_2000DPS;
            new_range_scale_dps_digit = 70e-3f;
            break;
        default:
            return;
    }

    _gyro_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
    _writeReg(ADDR_CTRL_REG4, bits);
}

int L3GD20::l3gd20_init()
{
    /* Zero the struct */
    m_synchronize.lock();

    m_sensor_data.accel_m_s2_x = 0.0f;
    m_sensor_data.accel_m_s2_y = 0.0f;
    m_sensor_data.accel_m_s2_z = 0.0f;
    m_sensor_data.gyro_rad_s_x = 0.0f;
    m_sensor_data.gyro_rad_s_y = 0.0f;
    m_sensor_data.gyro_rad_s_z = 0.0f;
    m_sensor_data.mag_ga_x = 0.0f;
    m_sensor_data.mag_ga_y = 0.0f;
    m_sensor_data.mag_ga_z = 0.0f;
    m_sensor_data.temp_c = 0.0f;

    m_sensor_data.read_counter = 0;
    m_sensor_data.error_counter = 0;
    m_sensor_data.gyro_range_hit_counter = 0;
    m_sensor_data.accel_range_hit_counter = 0;

    m_synchronize.unlock();

    _writeReg(ADDR_CTRL_REG1,
                      REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
    _writeReg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
    _writeReg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
    _writeReg(ADDR_CTRL_REG4, REG4_BDU);
    _writeReg(ADDR_CTRL_REG5, 0);
    _writeReg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

    /* disable FIFO. This makes things simpler and ensures we
     * aren't getting stale data. It means we must run the hrt
     * callback fast enough to not miss data. */
    _writeReg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

    set_samplerate(0); // 760Hz or 800Hz

    // Enable Gyroscope
    _writeReg(LSM9DS1XG_CTRL_REG4, BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G);

    // Configure Gyroscope
    _writeReg(LSM9DS1XG_CTRL_REG1_G, BITS_ODR_G_952HZ | BITS_FS_G_2000DPS);

    usleep(200);

    // Enable Accelerometer
    _writeReg(LSM9DS1XG_CTRL_REG5_XL, BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL);

    usleep(200);

    set_gyro_scale(RANGE_2000DPS);

    return 0;
}

void L3GD20::set_samplerate(unsigned frequency)
{
    uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

    if (frequency == 0 || frequency == GYRO_SAMPLERATE_DEFAULT) {
        frequency = 760;
    }

    /*
     * Use limits good for H or non-H models. Rates are slightly different
     * for L3G4200D part but register settings are the same.
     */
    if (frequency <= 100) {
        bits |= RATE_95HZ_LP_25HZ;

    } else if (frequency <= 200) {
        bits |= RATE_190HZ_LP_50HZ;

    } else if (frequency <= 400) {
        bits |= RATE_380HZ_LP_50HZ;

    } else if (frequency <= 800) {
        bits |= RATE_760HZ_LP_50HZ;

    } else {
        return;
    }

    _writeReg(ADDR_CTRL_REG1, bits);
}


int L3GD20::start()
{
    /* Open the device path specified in the class initialization. */
    // attempt to open device in start()
    int result = I2CDevObj::start();

    if (result != 0) {
        DF_LOG_ERR("DevObj start failed");
        DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
        return result;
    }

    /* Try to talk to the sensor. */
    uint8_t sensor_id;
    result = _readReg(ADDR_WHO_AM_I, sensor_id);

    if (result != 0) {
        DF_LOG_ERR("Unable to communicate with the L3GD20 sensor");
        return result;
    }

    if (sensor_id != WHO_I_AM_H && sensor_id != WHO_I_AM) {
        DF_LOG_ERR("L3GD20 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
                   sensor_id, WHO_I_AM);
        return -1;
    }

    result = l3gd20_init();

    if (result != 0) {
        DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
        return result;
    }

    result = DevObj::start();

    if (result != 0) {
        DF_LOG_ERR("DevObj start failed");
        return result;
    }

    return result;
}

int L3GD20::stop()
{
    int result = DevObj::stop();

    if (result != 0) {
        DF_LOG_ERR("DevObj stop failed");
        return result;
    }

    // We need to wait so that all measure calls are finished before
    // closing the device.
    usleep(10000);

    return 0;
}

void L3GD20::_measure()
{
    struct packet report {};

    // Read Temperature
    _readReg(ADDR_OUT_X_L | 0x80, (uint8_t *)&report.gyro_x, 6);

    DF_LOG_DEBUG("gyro x: %d, y: %d, z: %d", report.gyro_x, report.gyro_y, report.gyro_z);

    m_synchronize.lock();

    // Also check the full gyro range, however, this is very unlikely to happen.
    if (report.gyro_x == INT16_MIN || report.gyro_x == INT16_MAX ||
        report.gyro_y == INT16_MIN || report.gyro_y == INT16_MAX ||
        report.gyro_z == INT16_MIN || report.gyro_z == INT16_MAX) {
        ++m_sensor_data.gyro_range_hit_counter;
    }

    m_sensor_data.gyro_rad_s_x = float(report.gyro_x) * _gyro_scale;
    m_sensor_data.gyro_rad_s_y = float(report.gyro_y) * _gyro_scale;
    m_sensor_data.gyro_rad_s_z = -float(report.gyro_z) * _gyro_scale;

    // We need to fake this at 1000 us.
    m_sensor_data.fifo_sample_interval_us = 1000;

    ++m_sensor_data.read_counter;

    // Generate debug output every second, assuming that a sample is generated every
    // 1000 usecs
#ifdef L3GD20_DEBUG

    if (++m_sensor_data.read_counter % (1000000 / 1000) == 0) {

		DF_LOG_INFO("     gyro:  [%f, %f, %f] rad/s",
			    (double)m_sensor_data.gyro_rad_s_x,
			    (double)m_sensor_data.gyro_rad_s_y,
			    (double)m_sensor_data.gyro_rad_s_z);
	}

#endif

    _publish(m_sensor_data);

    m_synchronize.signal();
    m_synchronize.unlock();
}

int L3GD20::_publish(struct imu_sensor_data &data)
{
    // TBD
    return -1;
}
