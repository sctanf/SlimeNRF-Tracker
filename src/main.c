#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"

#include "Fusion/Fusion.h"
#include "magneto1_4.h"

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <hal/nrf_gpio.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include "retained.h"

LOG_MODULE_REGISTER(main, 4);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED0_NODE DT_NODELABEL(pwm_led0)

#define MAIN_IMU_NODE DT_NODELABEL(icm_0)
#define MAIN_MAG_NODE DT_NODELABEL(mmc_0)
#define AUX_IMU_NODE DT_NODELABEL(icm_1)
#define AUX_MAG_NODE DT_NODELABEL(mmc_1)

uint8_t batt;
uint8_t batt_v;
unsigned int batt_pptt;

const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);
const struct i2c_dt_spec main_mag = I2C_DT_SPEC_GET(MAIN_MAG_NODE);
const struct i2c_dt_spec aux_imu = I2C_DT_SPEC_GET(AUX_IMU_NODE);
const struct i2c_dt_spec aux_mag = I2C_DT_SPEC_GET(AUX_MAG_NODE);

//const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(LED0_NODE);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);

#include "ICM42688.h"
#include "MMC5983MA.h"

// TODO: move to sensor
// ICM42688 definitions

// TODO: move to sensor
/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
	  AFS_2G, AFS_4G, AFS_8G, AFS_16G
	  GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	  AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
	  AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
	  GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_200Hz, GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN;

// MMC5983MA definitions

// TODO: move to sensor
/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
 */
uint8_t MODR = MODR_200Hz, MBW = MBW_200Hz, MSET = MSET_2000; // 200hz bw (burn power)

//#define BURN_TEST

void main(void)
{
#ifdef BURN_TEST
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 1); // Boot LED (burn power)
	k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
	uint8_t ICM42688ID = icm_getChipID(main_imu);						 // Read CHIP_ID register for ICM42688
		LOG_INF("ICM: %u", ICM42688ID);
	uint8_t MMC5983ID = mmc_getChipID(main_mag);						 // Read CHIP_ID register for MMC5983MA
		LOG_INF("MMC: %u", MMC5983ID);
	if ((ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30) // check if all I2C sensors have acknowledged
	{
		LOG_INF("Found main imus");
    	i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
		icm_reset(main_imu);												 // software reset ICM42688 to default registers
		uint8_t temp;
		i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag
		icm_init(main_imu, Ascale, Gscale, AODR, GODR, aMode, gMode, false); // configure
		mmc_SET(main_mag);													 // "deGauss" magnetometer
		mmc_init(main_mag, MODR, MBW, MSET);								 // configure
	}
#endif
	while (1)
	{
		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);
		LOG_INF("BAT: %d", batt_mV);
		k_msleep(100);
	}
}
