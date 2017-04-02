#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "LSM303D.hpp"

#define G_SI 9.80665f
#define PI 3.14159f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

using namespace DriverFramework;

void LSM303D::set_acc_scale(int scale)
{
    uint8_t reg;

    _readReg(ADDR_CTRL_REG2, reg);
    reg &= ~REG2_FULL_SCALE_BITS_A;
    _writeReg(ADDR_CTRL_REG2, reg | scale);

    float new_scale_g_digit = 0.0f;

    switch (scale) {
        case REG2_FULL_SCALE_2G_A:
            _accel_range_m_s2 = 2.0f * LSM303D_ONE_G;
            setbits |= REG2_FULL_SCALE_2G_A;
            new_scale_g_digit = 0.061e-3f;
            break;
        case REG2_FULL_SCALE_4G_A:
            _accel_range_m_s2 = 4.0f * LSM303D_ONE_G;
            setbits |= REG2_FULL_SCALE_4G_A;
            new_scale_g_digit = 0.122e-3f;
            break;
        case REG2_FULL_SCALE_6G_A:
            _accel_range_m_s2 = 6.0f * LSM303D_ONE_G;
            setbits |= REG2_FULL_SCALE_6G_A;
            new_scale_g_digit = 0.183e-3f;
            break;
        case REG2_FULL_SCALE_8G_A:
            _accel_range_m_s2 = 8.0f * LSM303D_ONE_G;
            setbits |= REG2_FULL_SCALE_8G_A;
            new_scale_g_digit = 0.244e-3f;
            break;
        case REG2_FULL_SCALE_16G_A:
            _accel_range_m_s2 = 16.0f * LSM303D_ONE_G;
            setbits |= REG2_FULL_SCALE_16G_A;
            new_scale_g_digit = 0.732e-3f;
            break;
    }

    _acc_scale = new_scale_g_digit * LSM303D_ONE_G;
}

void LSM303D::set_mag_scale(int scale)
{
    uint8_t reg;
    _readReg(ADDR_CTRL_REG6, reg);
    reg &= ~REG6_FULL_SCALE_BITS_M;
    _writeReg(ADDR_CTRL_REG6, reg | scale);

    float new_scale_ga_digit = 0.0f;

    switch (scale) {
        case REG6_FULL_SCALE_2GA_M:
            new_scale_ga_digit = 0.080e-3f;
            break;
        case REG6_FULL_SCALE_4GA_M:
            new_scale_ga_digit = 0.160e-3f;
            break;
        case REG6_FULL_SCALE_8GA_M:
            new_scale_ga_digit = 0.320e-3f;
            break;
        case REG6_FULL_SCALE_12GA_M:
            new_scale_ga_digit = 0.479e-3f;
            break;
    }

    _mag_scale = new_scale_ga_digit;
}

int LSM303D::lsm303d_init()
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

    int result = _setSlaveConfig(LSM303D_SLAVE_ADDRESS,
                                 LSM303D_BUS_FREQUENCY_IN_KHZ,
                                 LSM303D_TRANSFER_TIMEOUT_IN_USECS);

    if (result < 0) {
        DF_LOG_ERR("Could not set slave config");
        return -1;
    }

    /* enable accel*/
    _writeReg(ADDR_CTRL_REG1,
                      REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_800HZ_A);

    /* enable mag */
    _writeReg(ADDR_CTRL_REG7, REG7_CONT_MODE_M);
    _writeReg(ADDR_CTRL_REG5, REG5_RES_HIGH_M | REG5_ENABLE_T);
    _writeReg(ADDR_CTRL_REG3, 0x04); // DRDY on ACCEL on INT1
    _writeReg(ADDR_CTRL_REG4, 0x04); // DRDY on MAG on INT2

    accel_set_range(LSM303D_ACCEL_DEFAULT_RANGE_G);
    accel_set_samplerate(LSM303D_ACCEL_DEFAULT_RATE);

    // we setup the anti-alias on-chip filter as 50Hz. We believe
    // this operates in the analog domain, and is critical for
    // anti-aliasing. The 2 pole software filter is designed to
    // operate in conjunction with this on-chip filter
    accel_set_onchip_lowpass_filter_bandwidth(LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

    mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);

//    _writeReg(LSM303D_REGISTER_CTRL1,
//               LSM303D_CTRL1_ACC_DATA_RATE_12_5_HZ |
//               LSM303D_CTRL1_AXEN | LSM303D_CTRL1_AYEN | LSM303D_CTRL1_AZEN
//            //    | LSM303D_CTRL1_BDU
//    );
//
//    _writeReg(LSM303D_REGISTER_CTRL2,
//               LSM303D_CTRL2_AFS_2G |
//               LSM303D_CTRL2_AAF_BANDWIDTH_773HZ
//    );
//
//    _writeReg(LSM303D_REGISTER_CTRL5,
//               LSM303D_CTRL5_TEMP_EN |
//               LSM303D_CTRL5_M_RES_HIGH |
//               LSM303D_CTRL5_MAG_DATA_RATE_100_HZ
//    );
//
//    _writeReg(LSM303D_REGISTER_CTRL6,
//               LSM303D_CTRL6_MFS_12GAUSS
//    );
//
//    _writeReg(LSM303D_REGISTER_CTRL7,
//               LSM303D_CTRL7_MAG_SENSOR_MODE_CONTINUOUS
//            //| LSM303D_CTRL7_HPF_REFERENCE_SIGNAL
//    );

    set_acc_scale(REG2_FULL_SCALE_16G_A);
    set_mag_scale(REG6_FULL_SCALE_12GA_M);

    return 0;
}

void LSM303D::accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG2_ANTIALIAS_FILTER_BW_BITS_A;

    if (bandwidth == 0) {
        bandwidth = 773;
    }

    if (bandwidth <= 50) {
        setbits |= REG2_AA_FILTER_BW_50HZ_A;
    } else if (bandwidth <= 194) {
        setbits |= REG2_AA_FILTER_BW_194HZ_A;
    } else if (bandwidth <= 362) {
        setbits |= REG2_AA_FILTER_BW_362HZ_A;
    } else if (bandwidth <= 773) {
        setbits |= REG2_AA_FILTER_BW_773HZ_A;
    } else {
        return;
    }

    uint8_t reg;
    _readReg(ADDR_CTRL_REG2, reg);
    reg &= ~clearbits;
    _writeReg(ADDR_CTRL_REG2, reg | setbits);
}

void LSM303D::accel_set_samplerate(unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG1_RATE_BITS_A;

    if (frequency == 0 || frequency == ACCEL_SAMPLERATE_DEFAULT) {
        frequency = 1600;
    }

    if (frequency <= 100) {
        setbits |= REG1_RATE_100HZ_A;
        _accel_samplerate = 100;

    } else if (frequency <= 200) {
        setbits |= REG1_RATE_200HZ_A;
        _accel_samplerate = 200;

    } else if (frequency <= 400) {
        setbits |= REG1_RATE_400HZ_A;
        _accel_samplerate = 400;

    } else if (frequency <= 800) {
        setbits |= REG1_RATE_800HZ_A;
        _accel_samplerate = 800;

    } else if (frequency <= 1600) {
        setbits |= REG1_RATE_1600HZ_A;
        _accel_samplerate = 1600;

    } else {
        return;
    }

    uint8_t reg;
    _readReg(ADDR_CTRL_REG1, reg);
    reg &= ~clearbits;
    _writeReg(ADDR_CTRL_REG1, reg | setbits);
}

void LSM303D::mag_set_samplerate(unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG5_RATE_BITS_M;

    if (frequency == 0) {
        frequency = 100;
    }

    if (frequency <= 25) {
        setbits |= REG5_RATE_25HZ_M;
        _mag_samplerate = 25;

    } else if (frequency <= 50) {
        setbits |= REG5_RATE_50HZ_M;
        _mag_samplerate = 50;

    } else if (frequency <= 100) {
        setbits |= REG5_RATE_100HZ_M;
        _mag_samplerate = 100;

    } else {
        return;
    }

    uint8_t reg;
    _readReg(ADDR_CTRL_REG5, reg);
    reg &= ~clearbits;
    _writeReg(ADDR_CTRL_REG5, reg | setbits);

    return OK;
}

int LSM303D::start()
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
        DF_LOG_ERR("Unable to communicate with the LSM303D sensor");
        goto exit;
    }

    if (sensor_id != WHO_I_AM) {
        DF_LOG_ERR("LSM303D sensor WHOAMI wrong: 0x%X, should be: 0x%X",
                   sensor_id, WHO_I_AM);
        result = -1;
        goto exit;
    }

    result = lsm303d_init();

    if (result != 0) {
        DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
        goto exit;
    }

    result = DevObj::start();

    if (result != 0) {
        DF_LOG_ERR("DevObj start failed");
        return result;
    }

    exit:

    return result;
}

int LSM303D::stop()
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

void LSM303D::_measure()
{
    struct packet report {};

    // Read Temperature
    _readReg(ADDR_OUT_TEMP_L, (uint8_t *)&report.temp, 2);

    // Read Accelerometor
    _readReg(ADDR_OUT_X_L_A, (uint8_t *)&report.accel_x, 6);

    if (_mag_enabled) {
        _readReg(ADDR_OUT_X_L_M, (uint8_t *)&report.mag_x, 6);

        DF_LOG_DEBUG("mag x: %d, y: %d, z: %d", report.mag_x, report.mag_y, report.mag_z);

    } else {
        report.mag_x = 0;
        report.mag_y = 0;
        report.mag_z = 0;
    }

    DF_LOG_DEBUG("accel x: %d, y: %d, z: %d", report.accel_x, report.accel_y, report.accel_z);
    DF_LOG_DEBUG("gyro x: %d, y: %d, z: %d", report.gyro_x, report.gyro_y, report.gyro_z);

    m_synchronize.lock();

    // Check if the full accel range of the accel has been used. If this occurs, it is
    // either a spike due to a crash/landing or a sign that the vibrations levels
    // measured are excessive.
    if (report.accel_x == INT16_MIN || report.accel_x == INT16_MAX ||
        report.accel_y == INT16_MIN || report.accel_y == INT16_MAX ||
        report.accel_z == INT16_MIN || report.accel_z == INT16_MAX) {
        ++m_sensor_data.accel_range_hit_counter;
    }

    // Inverting z to make the coordinate system right handed because the
    // sensors coordinate system is left handed according to the datasheet.

    m_sensor_data.accel_m_s2_x = report.accel_x * _acc_scale;
    m_sensor_data.accel_m_s2_y = report.accel_y * _acc_scale;
    m_sensor_data.accel_m_s2_z = -report.accel_z * _acc_scale;

    m_sensor_data.temp_c = float(report.temp) / 16.0f  + 25.0f;

    m_sensor_data.mag_ga_x = -float(report.mag_x) * _mag_scale;
    m_sensor_data.mag_ga_y = float(report.mag_y) * _mag_scale;
    m_sensor_data.mag_ga_z = -float(report.mag_z) * _mag_scale;

    // We need to fake this at 1000 us.
    m_sensor_data.fifo_sample_interval_us = 1000;

    ++m_sensor_data.read_counter;

    // Generate debug output every second, assuming that a sample is generated every
    // 1000 usecs
#ifdef LSM303D_DEBUG

    if (++m_sensor_data.read_counter % (1000000 / 1000) == 0) {

		DF_LOG_INFO("IMU: accel: [%f, %f, %f] m/s^2",
			    (double)m_sensor_data.accel_m_s2_x,
			    (double)m_sensor_data.accel_m_s2_y,
			    (double)m_sensor_data.accel_m_s2_z);

		if (_mag_enabled) {
			DF_LOG_INFO("     mag:  [%f, %f, %f] ga",
				    (double)m_sensor_data.mag_ga_x,
				    (double)m_sensor_data.mag_ga_y,
				    (double)m_sensor_data.mag_ga_z);
		}

		DF_LOG_INFO("    temp:  %f C",
			    (double)m_sensor_data.temp_c);
	}

#endif

    _publish(m_sensor_data);

    m_synchronize.signal();
    m_synchronize.unlock();
}

int LSM303D::_publish(struct imu_sensor_data &data)
{
    // TBD
    return -1;
}
