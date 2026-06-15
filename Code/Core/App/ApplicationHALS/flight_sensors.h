
#ifndef FLIGHT_SENSORS_H
#define FLIGHT_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "BMP388.h"
#include "lsm6dsox_reg.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Flight sensor data structure */
typedef struct
{
    float altitude;
    float pressure;
    float temperature;
    float velocity;

    /* IMU accelerometer */
    float x_mg_IMU;
    float y_mg_IMU;
    float z_mg_IMU;

    /* IMU gyroscope */
    float x_gy;
    float y_gy;
    float z_gy;

    float kalman_altitude;   
    float kalman_velocity; 

    /*Flight State*/
    uint8_t flight_state;
CSBarometer_GPIO_Port
} FlightSensorData;

/* Initialization */
HAL_StatusTypeDef flight_sensors_init(void);

/* Read + update all flight sensor values */
HAL_StatusTypeDef flight_sensors_update_baro(FlightSensorData *sensordata);
HAL_StatusTypeDef flight_sensors_update_IMU_accel(FlightSensorData *sensordata);

#ifdef __cplusplus
}
#endif

#endif