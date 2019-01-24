#include "bme680.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>

int fd;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  write(fd, &reg_addr,1);
  read(fd, data, len);
  return 0;
}

void user_delay_ms(uint32_t period)
{
  usleep(period*1000);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);
  write(fd, buf, len +1);
  free(buf);
  return 0;
}

int main(int argc, char* argv[])
{
  struct bme680_dev dev;
  int8_t rslt = BME680_OK;

  if ((fd = open("/dev/i2c-0", O_RDWR)) < 0) {
    printf("Failed to open the i2c bus %s", argv[1]);
    exit(1);
  }
  if (ioctl(fd, I2C_SLAVE, 0x77) < 0) {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    exit(1);
  }
  dev.dev_id = BME680_I2C_ADDR_SECONDARY;
  dev.intf = BME680_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;
  dev.amb_temp = 25;

  rslt = bme680_init(&dev);
  if (rslt == BME680_OK) {
	  /* Sensor chip ID will be printed if initialization was successful */
	  printf("Device found with chip id 0x%x\n", dev.chip_id);
  } else {
	  printf("bme680_init() failed: %d\n", rslt);
	  return 1;
  }


  uint8_t set_required_settings;

  /* Set the temperature, pressure and humidity settings */
  dev.tph_sett.os_hum = BME680_OS_2X;
  dev.tph_sett.os_pres = BME680_OS_4X;
  dev.tph_sett.os_temp = BME680_OS_8X;
  dev.tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  dev.gas_sett.heatr_temp = 320; /* degree Celsius */
  dev.gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  dev.power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
	  | BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings,&dev);

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&dev);

  /* Get the total measurement duration so as to sleep or wait till the
   * measurement is complete */
  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &dev);

  struct bme680_field_data data;

  while(1)
  {
	  user_delay_ms(meas_period); /* Delay till the measurement is ready */

	  rslt = bme680_get_sensor_data(&data, &dev);

	  printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
			  data.pressure / 100.0f, data.humidity / 1000.0f );
	  /* Avoid using measurements from an unstable heating setup */
	  if(data.status & BME680_GASM_VALID_MSK)
		  printf(", G: %d ohms", data.gas_resistance);

	  printf("\r\n");

	  /* Trigger the next measurement if you would like to read data out continuously */
	  if (dev.power_mode == BME680_FORCED_MODE) {
		  rslt = bme680_set_sensor_mode(&dev);
	  }
  }

  return 0;
}
