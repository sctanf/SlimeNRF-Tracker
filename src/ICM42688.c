/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include "ICM42688.h"

float _gRes;

uint8_t icm_getChipID(struct i2c_dt_spec dev_i2c)
{
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, ICM42688_WHO_AM_I, &temp);
    return temp;
}

float icm_getAres(uint8_t Ascale) {
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        case AFS_2G:
            return 2.0f/32768.0f;
        case AFS_4G:
            return 4.0f/32768.0f;
        case AFS_8G:
            return 8.0f/32768.0f;
        case AFS_16G:
        default:// I assume this is a safe default given the comment
            return 16.0f/32768.0f;
    }
}

float icm_getGres(uint8_t Gscale) {
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        case GFS_15_625DPS:
            return 15.625f/32768.0f;
        case GFS_31_25DPS:
            return 31.25f/32768.0f;
        case GFS_62_50DPS:
            return 62.5f/32768.0f;
        case GFS_125DPS:
            return 125.0f/32768.0f;
        case GFS_250DPS:
            return 250.0f/32768.0f;
        case GFS_500DPS:
            return 500.0f/32768.0f;
        case GFS_1000DPS:
            return 1000.0f/32768.0f;
        case GFS_2000DPS:
        default: // I assume this is a safe default given the comment
            return 2000.0f/32768.0f;
    }
}

void icm_reset(struct i2c_dt_spec dev_i2c)
{
    // reset device
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_DEVICE_CONFIG, 0x01); // Set bit 0 to 1 to reset ICM42688
    k_msleep(2); // Wait 1 ms for all registers to reset
}

void icm_setup_WOM(struct i2c_dt_spec dev_i2c)
{
    uint8_t temp;
    i2c_reg_read_byte_dt(&dev_i2c, ICM42688_INT_STATUS, &temp); // clear reset done int flag
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_CONFIG0, AFS_8G << 5 | AODR_200Hz); // set accel ODR and FS
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_PWR_MGMT0, aMode_LP); // set accel and gyro modes
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INTF_CONFIG1, 0x00); // set low power clock
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_X_THR, 0x08); // set wake thresholds // 80 x 3.9 mg is ~312 mg
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_Y_THR, 0x08); // set wake thresholds
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_WOM_Z_THR, 0x08); // set wake thresholds
	k_busy_wait(1000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE1, 0x07); // enable WOM interrupt
	k_busy_wait(50000);
	i2c_reg_write_byte_dt(&dev_i2c, ICM42688_SMD_CONFIG, 0x05); // enable WOM feature
}

// make i2c stuff external? (portability)
// make busy wait and msleep external? (portability)
void icm_init(struct i2c_dt_spec dev_i2c, uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN)
{
    icm_getAres(Ascale);
    icm_getGres(Gscale);
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
    k_busy_wait(250); // wait >200us (datasheet 14.36)
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR); // set gyro ODR and FS
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
	k_msleep(50); // 10ms Accel, 30ms Gyro startup
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 0x00); // FIFO bypass mode
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
    i2c_reg_update_byte_dt(&dev_i2c, ICM42688_TMST_CONFIG, 0x02, 0x00); // disable FSYNC
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG1, 0x02); // enable FIFO gyro only
    i2c_reg_write_byte_dt(&dev_i2c, ICM42688_FIFO_CONFIG, 1<<6); // begin FIFO stream
}

void icm_accel_read(struct i2c_dt_spec dev_i2c, float a[3]) {
    uint8_t rawAccel[6];
    i2c_burst_read_dt(&dev_i2c, ICM42688_ACCEL_DATA_X1, &rawAccel[0], 6);
	float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
	float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
	float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
	a[0] = raw0 * _aRes;
	a[1] = raw1 * _aRes;
	a[2] = raw2 * _aRes;
}

void icm_gyro_read(struct i2c_dt_spec dev_i2c, float g[3]) {
    uint8_t rawGyro[6];
    i2c_burst_read_dt(&dev_i2c, ICM42688_GYRO_DATA_X1, &rawGyro[0], 6);
	float raw0 = (int16_t)((((int16_t)rawGyro[0]) << 8) | rawGyro[1]);
	float raw1 = (int16_t)((((int16_t)rawGyro[2]) << 8) | rawGyro[3]);
	float raw2 = (int16_t)((((int16_t)rawGyro[4]) << 8) | rawGyro[5]);
	g[0] = raw0 * _gRes;
	g[1] = raw1 * _gRes;
	g[2] = raw2 * _gRes;
}

// need to make this external
void icm_offsetBias(struct i2c_dt_spec dev_i2c, float * dest1, float * dest2)
{
    for (int ii = 0; ii < 500; ii++)
    {
        float rawData[3];
        icm_accel_read(dev_i2c, &rawData[0]);
        dest1[0] += rawData[0];
        dest1[1] += rawData[1];
        dest1[2] += rawData[2];
        icm_gyro_read(dev_i2c, &rawData[3]);
        dest2[0] += rawData[0];
        dest2[1] += rawData[1];
        dest2[2] += rawData[2];
        k_msleep(5);
    }

    dest1[0] /= 500.0f;
    dest1[1] /= 500.0f;
    dest1[2] /= 500.0f;
    dest2[0] /= 500.0f;
    dest2[1] /= 500.0f;
    dest2[2] /= 500.0f;
// need better accel calibration
    if(dest1[0] > 0.8f) {dest1[0] -= 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
    if(dest1[0] < -0.8f) {dest1[0] += 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
    if(dest1[1] > 0.8f) {dest1[1] -= 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
    if(dest1[1] < -0.8f) {dest1[1] += 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
    if(dest1[2] > 0.8f) {dest1[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
    if(dest1[2] < -0.8f) {dest1[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
}
