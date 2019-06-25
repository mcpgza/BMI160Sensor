#include <stdio.h>
#include "bmi160.h"
#include "bmi160_defs.h"


#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_READ_MAG_HZ             20
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG               (SENSORS_READ_RATE_HZ/SENSORS_READ_MAG_HZ)

#define BSTDR_OK 0

#define SENSORS_BMI160_GYRO_FS_CFG      BMI160_GYRO_RANGE_2000_DPS
#define SENSORS_BMI160_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI160_ACCEL_CFG        16
#define SENSORS_BMI160_ACCEL_FS_CFG     BMI160_ACCEL_RANGE_16G
#define SENSORS_BMI160_G_PER_LSB_CFG    (2.0f * (float)SENSORS_BMI160_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI160_1G_IN_LSB        65536 / SENSORS_BMI160_ACCEL_CFG / 2

int main() {

//prinf("hello.c");


struct bmi160_dev bmi160Dev;


bmi160Dev.interface = BMI160_I2C_INTF;
//bmi160Dev.read = (bmi160_com_fptr_t)bstdr_burst_read;
  // assign bus write function
//bmi160Dev.write = (bmi160_com_fptr_t)bstdr_burst_write;
  // assign delay function
//bmi160Dev.delay_ms = (bmi160_delay_fptr_t)bstdr_ms_delay;
bmi160Dev.id = BMI160_I2C_ADDR+1;  // I2C device address

int8_t rslt = BMI160_OK;
rslt = bmi160_init(&bmi160Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
/*      DEBUG_PRINT("BMI160 I2C connection [OK].\n");*/
      /* Select the Output data rate, range of Gyroscope sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */


      bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
      bmi160Dev.gyro_cfg.range = SENSORS_BMI160_GYRO_FS_CFG;
      bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

      /* Select the power mode of Gyroscope sensor */
      bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

      /* Select the Output data rate, range of accelerometer sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */
      bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
      bmi160Dev.accel_cfg.range = SENSORS_BMI160_ACCEL_FS_CFG;
      bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
      /* Select the power mode of accelerometer sensor */
      bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Set the sensor configuration */
      rslt |= bmi160_set_sens_conf(&bmi160Dev);
      bmi160Dev.delay_ms(50);

      /* read sensor */
      struct bmi160_sensor_data gyr;
      rslt |= bmi160_get_sensor_data(BMI160_GYRO_ONLY, NULL, &gyr,
                                     &bmi160Dev);
      struct bmi160_sensor_data acc;
      rslt |= bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &acc, NULL,
                                     &bmi160Dev);
    }
  else
    {
 //     DEBUG_PRINT("BMI160 I2C connection [FAIL].\n");
    }


}
