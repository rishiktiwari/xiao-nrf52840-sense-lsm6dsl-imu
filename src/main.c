#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#define SLEEP_MS 1000
#define LEDR_NODE DT_ALIAS(led0)
#define LEDG_NODE DT_ALIAS(led1)
#define LEDB_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec statLeds[3] = {
	GPIO_DT_SPEC_GET(LEDR_NODE, gpios),
	GPIO_DT_SPEC_GET(LEDG_NODE, gpios),
	GPIO_DT_SPEC_GET(LEDB_NODE, gpios)
};
static const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
static const struct device *lsm6ds3tr_c_en = DEVICE_DT_GET(DT_PATH(lsm6ds3tr_c_en));

static int print_samples;
static int lsm6dsl_trig_cnt;
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

	if (print_samples) {
		print_samples = 0;
		accel_x_out = accel_x;
		accel_y_out = accel_y;
		accel_z_out = accel_z;
		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;
	}
}
#endif


static int setStatLed(int red, int green, int blue) {
	return gpio_pin_set_dt(&statLeds[0], red)
			&& gpio_pin_set_dt(&statLeds[1], green)
			&& gpio_pin_set_dt(&statLeds[2], blue);
}


int main(void) 
{
	int ret;
	char out_str[64];
	struct sensor_value odr_attr;

	for (unsigned int i = 0; i < 3; i++) {
		if (!gpio_is_ready_dt(&statLeds[0])) {
			printk("led %d: not ready\n", i);
			return 0;
		}
		if (gpio_pin_configure_dt(&statLeds[0], GPIO_OUTPUT_ACTIVE) < 0) {
			printk("led %d: config fail\n", i);
			return 0;
		}
	}
	setStatLed(1, 0, 0);

	if (!device_is_ready(lsm6ds3tr_c_en)) {
        printk("lsm6ds3tr_c_en: not ready\n");
        return 0;
    }
	if (!device_is_ready(lsm6dsl_dev)) {
		printk("st_lsm6dsl: not ready\n");
		return 0;
	}
	printk("devices ready\n");
	setStatLed(0, 1, 0);

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;
	if (sensor_attr_set(lsm6ds3tr_c_en, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}
	if (sensor_attr_set(lsm6ds3tr_c_en, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	if (sensor_trigger_set(lsm6ds3tr_c_en, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}
	#endif
	
	ret = sensor_sample_fetch(lsm6ds3tr_c_en);
	if (ret < 0) {
		printk("st_lsm6dsl: sample update err, err: %d\n", ret);
		return 0;
	}

	while (true) {
		printk("\0033\014");

		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
			sensor_value_to_double(&accel_x_out),
			sensor_value_to_double(&accel_y_out),
			sensor_value_to_double(&accel_z_out)
		);
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
			sensor_value_to_double(&gyro_x_out),
			sensor_value_to_double(&gyro_y_out),
			sensor_value_to_double(&gyro_z_out)
		);

		printk("%s\n", out_str);

		print_samples = 1;
		k_msleep(SLEEP_MS);
	}

	return 0;
}
