#include <string.h>
#include "DriverFramework.hpp"
#include "BMP180.hpp"

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4
#define BMP180_REGISTER_CHIPID 0xD0

#define BMP180_ID 0x55

using namespace DriverFramework;

int BMP180::loadCalibration()
{
    int16_t AC1 = reads16(0xAA);
    int16_t AC2 = reads16(0xAC);
    int16_t AC3 = reads16(0xAE);
    uint16_t AC4 = readu16(0xB0);
    uint16_t AC5 = readu16(0xB2);
    uint16_t AC6 = readu16(0xB4);
    int16_t VB1 = reads16(0xB6);
    int16_t VB2 = reads16(0xB8);
    int16_t MB = reads16(0xBA);
    int16_t MC = reads16(0xBC);
    int16_t MD = reads16(0xBE);

    double c3 = 160.0 * pow(2,-15) * AC3;
    double c4 = pow(10,-3) * pow(2,-15) * AC4;
    double b1 = pow(160,2) * pow(2,-30) * VB1;
    m_sensor_calibration.c5 = (pow(2,-15) / 160) * AC5;
    m_sensor_calibration.c6 = AC6;
    m_sensor_calibration.mc = (pow(2,11) / pow(160,2)) * MC;
    m_sensor_calibration.md = MD / 160.0;
    m_sensor_calibration.x0 = AC1;
    m_sensor_calibration.x1 = 160.0 * pow(2,-13) * AC2;
    m_sensor_calibration.x2 = pow(160,2) * pow(2,-25) * VB2;
    m_sensor_calibration.y0 = c4 * pow(2,15);
    m_sensor_calibration.y1 = c4 * c3;
    m_sensor_calibration.y2 = c4 * b1;
    m_sensor_calibration.p0 = (3791.0 - 8.0) / 1600.0;
    m_sensor_calibration.p1 = 1.0 - 7357.0 * pow(2,-20);
    m_sensor_calibration.p2 = 3038.0 * 100.0 * pow(2,-36);

    return 0;
}

int16_t BMP180::reads16(uint8_t reg) {
    uint8_t data[2];
    _readReg(reg, 2, data);
    return (int16_t)(data[0]) << 8 | (int16_t)data[1];
}

uint16_t BMP180::readu16(uint8_t reg) {
    uint8_t data[2];
    _readReg(reg, 2, data);
    return (uint16_t)(data[0]) << 8 | (uint16_t)data[1];
}

int BMP180::bmp180_init()
{
    /* Zero the struct */
    m_synchronize.lock();

    m_sensor_data.pressure_pa = 0.0f;
    m_sensor_data.temperature_c = 0.0f;
    m_sensor_data.last_read_time_usec = 0;
    m_sensor_data.read_counter = 0;
    m_sensor_data.error_counter = 0;
    m_synchronize.unlock();

    int result;
    uint8_t sensor_id;

    /* Read the ID of the BMP180 sensor to confirm it's presence. */
    result = _readReg(BMP180_REGISTER_CHIPID, &sensor_id, sizeof(sensor_id));

    if (result != 0) {
        DF_LOG_ERR("error: unable to communicate with the bmp180 pressure sensor");
        return -EIO;
    }

    if (sensor_id != BMP180_ID) {
        DF_LOG_ERR("BMP180 sensor ID returned 0x%x instead of 0x%x", sensor_id, BMP180_ID);
        return -1;
    }

    /* Load and display the internal calibration values. */
    result = loadCalibration();

    if (result != 0) {
        DF_LOG_ERR("error: unable to complete initialization of the bmp180 pressure sensor");
        return -EIO;
    }

    DF_LOG_ERR("additional sensor configuration succeeded");

    usleep(1000);
    return 0;
}

int BMP180::start()
{
    int result = I2CDevObj::start();

    if (result != 0) {
        DF_LOG_ERR("error: could not start DevObj");
        return result;
    }

    /* Configure the I2C bus parameters for the pressure sensor. */
    result = _setSlaveConfig(BMP180_SLAVE_ADDRESS,
                             BMP180_BUS_FREQUENCY_IN_KHZ,
                             BMP180_TRANSFER_TIMEOUT_IN_USECS);

    if (result != 0) {
        DF_LOG_ERR("I2C slave configuration failed");
        return result;
    }

    /* Initialize the pressure sensor for active and continuous operation. */
    result = bmp180_init();

    if (result != 0) {
        DF_LOG_ERR("error: pressure sensor initialization failed, sensor read thread not started");
        return result;
    }

    result = DevObj::start();

    if (result != 0) {
        DF_LOG_ERR("error: could not start DevObj");
        return result;
    }

    return result;
}

int BMP180::stop()
{
    int result = DevObj::stop();

    if (result != 0) {
        DF_LOG_ERR("DevObj stop failed");
        return result;
    }

    return 0;
}

void BMP180::_measure()
{
    _writeReg(BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE);
    delay(5);
    double tu = reads16(BMP180_REG_RESULT);
    double a = m_sensor_calibration.c5 * (m_sensor_calibration.tu - m_sensor_calibration.c6);
    double temperature = a + (m_sensor_calibration.mc / (a + m_sensor_calibration.md));

    _writeReg(BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE3);

    delay(26);

    uint8_t data[3];
    _readReg(BMP180_REG_RESULT, 3, data);

    double pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

    double s = temperature - 25.0;
    double x = (m_sensor_calibration.x2 * pow(s,2)) + (m_sensor_calibration.x1 * s) + m_sensor_calibration.x0;
    double y = (m_sensor_calibration.y2 * pow(s,2)) + (m_sensor_calibration.y1 * s) + m_sensor_calibration.y0;
    double z = (pu - x) / y;
    double pressure = (m_sensor_calibration.p2 * pow(z,2)) + (m_sensor_calibration.p1 * z) + m_sensor_calibration.p0;

    m_synchronize.lock();

    m_sensor_data.temperature_c = temperature;
    m_sensor_data.pressure_pa = pressure;
    m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
    m_sensor_data.read_counter++;

    _publish(m_sensor_data);

    m_synchronize.signal();
    m_synchronize.unlock();
}

int BMP180::_publish(struct baro_sensor_data &data)
{
    // TBD
    return -1;
}
