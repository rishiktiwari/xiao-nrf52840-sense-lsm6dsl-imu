#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stdio.h>

#define LEDR_NODE DT_ALIAS(led0)
#define LEDG_NODE DT_ALIAS(led1)
#define LEDB_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec statLeds[3] = {
	GPIO_DT_SPEC_GET(LEDR_NODE, gpios),
	GPIO_DT_SPEC_GET(LEDG_NODE, gpios),
	GPIO_DT_SPEC_GET(LEDB_NODE, gpios)
};

static int print_samples;
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;


static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static void setStatLed(int red, int green, int blue) {
	gpio_pin_set_dt(&statLeds[0], red);
	gpio_pin_set_dt(&statLeds[1], green);
	gpio_pin_set_dt(&statLeds[2], blue);
	return;
}

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev, struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

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

int main(void)
{
	char out_str[64];
	struct sensor_value odr_attr;
	const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	for (unsigned int i = 0; i < 3; i++) {
		if (!gpio_is_ready_dt(&statLeds[i])) {
			printk("led %d: not ready\n", i);
			return 0;
		}
		if (gpio_pin_configure_dt(&statLeds[i], GPIO_OUTPUT_ACTIVE) < 0) {
			printk("led %d: config fail\n", i);
			return 0;
		}
	}
	setStatLed(1, 0, 0);
	k_msleep(100);

	if (lsm6dsl_dev == NULL) {
		printk("Could not get LSM6DSL device\n");
		return 0;
	}
	if (!device_is_ready(lsm6dsl_dev)) {
		printk("LSM6DSL: not ready\n");
		return 0;
	}

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;
	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}
	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}
#endif

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}
	setStatLed(0, 1, 0);
	k_msleep(100);

	while (1) {
		setStatLed(0, 0, 1);
		/* lsm6dsl accel */
		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
							  out_ev(&accel_x_out),
							  out_ev(&accel_y_out),
							  out_ev(&accel_z_out));
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
							   out_ev(&gyro_x_out),
							   out_ev(&gyro_y_out),
							   out_ev(&gyro_z_out));
		printk("%s\n\n", out_str);

		print_samples = 1;
		setStatLed(0, 0, 0);
		k_msleep(50);
	}

	return 0;
}
