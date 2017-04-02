#pragma once

#include "BaroSensor.hpp"

namespace DriverFramework
{
    struct bmp180_sensor_calibration {
        double c5;
        double c6;
        double mc;
        double md;
        double x0, x1, x2;
        double y0, y1, y2;
        double p0, p1, p2;
    };

// update frequency is 50 Hz (44.4-51.3Hz ) at 8x oversampling
#define BMP180_MEASURE_INTERVAL_US 20000

#define BMP180_BUS_FREQUENCY_IN_KHZ 400
#define BMP180_TRANSFER_TIMEOUT_IN_USECS 9000

#define BMP180_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 6
#define BMP180_MAX_LEN_CALIB_VALUES 26

#define BMP180_SLAVE_ADDRESS 0x77

    class BMP180 : public BaroSensor
    {
    public:
        BMP180(const char *device_path) :
                BaroSensor(device_path, BMP180_MEASURE_INTERVAL_US)
        {
        }

        // @return 0 on success, -errno on failure
        virtual int start();

        // @return 0 on success, -errno on failure
        virtual int stop();

    protected:
        virtual void _measure();
        virtual int _publish(struct baro_sensor_data &data);

    private:
        int loadCalibration();

        // returns 0 on success, -errno on failure
        int bmp180_init();

        int16_t reads16(uint8_t reg);
        uint16_t readu16(uint8_t reg);

        struct bmp180_sensor_calibration 	m_sensor_calibration;
    };

}; // namespace DriverFramework
