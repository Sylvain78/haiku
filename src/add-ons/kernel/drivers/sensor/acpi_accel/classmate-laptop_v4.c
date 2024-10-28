// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2009  Thadeu Lima de Souza Cascardo <cascardo@holoscopio.com>
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/acpi.h>
#include <linux/backlight.h>
#include <linux/input.h>
#include <linux/rfkill.h>

struct cmpc_accel {
	int sensitivity;
	int g_select;
	int inputdev_state;
};

#define CMPC_ACCEL_DEV_STATE_CLOSED	0
#define CMPC_ACCEL_DEV_STATE_OPEN	1

#define CMPC_ACCEL_SENSITIVITY_DEFAULT		5
#define CMPC_ACCEL_G_SELECT_DEFAULT		0

#define CMPC_ACCEL_HID_V4	"ACCE0001"

/*
 * Generic input device code.
 */

typedef void (*input_device_init)(struct input_dev *dev);

static int cmpc_add_acpi_notify_device(struct acpi_device *acpi, char *name,
				       input_device_init idev_init)
{
	struct input_dev *inputdev;
	int error;

	inputdev = input_allocate_device();
	if (!inputdev)
		return -ENOMEM;
	inputdev->name = name;
	inputdev->dev.parent = &acpi->dev;
	idev_init(inputdev);
	error = input_register_device(inputdev);
	if (error) {
		input_free_device(inputdev);
		return error;
	}
	dev_set_drvdata(&acpi->dev, inputdev);
	return 0;
}

static int cmpc_remove_acpi_notify_device(struct acpi_device *acpi)
{
	struct input_dev *inputdev = dev_get_drvdata(&acpi->dev);
	input_unregister_device(inputdev);
	return 0;
}

/*
 * Accelerometer code for Classmate V4
 */
static acpi_status cmpc_start_accel_v4(acpi_handle handle)
{
	union acpi_object param[4];
	struct acpi_object_list input;
	acpi_status status;

	param[0].type = ACPI_TYPE_INTEGER;
	param[0].integer.value = 0x3;
	param[1].type = ACPI_TYPE_INTEGER;
	param[1].integer.value = 0;
	param[2].type = ACPI_TYPE_INTEGER;
	param[2].integer.value = 0;
	param[3].type = ACPI_TYPE_INTEGER;
	param[3].integer.value = 0;
	input.count = 4;
	input.pointer = param;
	status = acpi_evaluate_object(handle, "ACMD", &input, NULL);
	return status;
}

static acpi_status cmpc_stop_accel_v4(acpi_handle handle)
{
	union acpi_object param[4];
	struct acpi_object_list input;
	acpi_status status;

	param[0].type = ACPI_TYPE_INTEGER;
	param[0].integer.value = 0x4;
	param[1].type = ACPI_TYPE_INTEGER;
	param[1].integer.value = 0;
	param[2].type = ACPI_TYPE_INTEGER;
	param[2].integer.value = 0;
	param[3].type = ACPI_TYPE_INTEGER;
	param[3].integer.value = 0;
	input.count = 4;
	input.pointer = param;
	status = acpi_evaluate_object(handle, "ACMD", &input, NULL);
	return status;
}

static acpi_status cmpc_accel_set_sensitivity_v4(acpi_handle handle, int val)
{
	union acpi_object param[4];
	struct acpi_object_list input;

	param[0].type = ACPI_TYPE_INTEGER;
	param[0].integer.value = 0x02;
	param[1].type = ACPI_TYPE_INTEGER;
	param[1].integer.value = val;
	param[2].type = ACPI_TYPE_INTEGER;
	param[2].integer.value = 0;
	param[3].type = ACPI_TYPE_INTEGER;
	param[3].integer.value = 0;
	input.count = 4;
	input.pointer = param;
	return acpi_evaluate_object(handle, "ACMD", &input, NULL);
}

static acpi_status cmpc_accel_set_g_select_v4(acpi_handle handle, int val)
{
	union acpi_object param[4];
	struct acpi_object_list input;

	param[0].type = ACPI_TYPE_INTEGER;
	param[0].integer.value = 0x05;
	param[1].type = ACPI_TYPE_INTEGER;
	param[1].integer.value = val;
	param[2].type = ACPI_TYPE_INTEGER;
	param[2].integer.value = 0;
	param[3].type = ACPI_TYPE_INTEGER;
	param[3].integer.value = 0;
	input.count = 4;
	input.pointer = param;
	return acpi_evaluate_object(handle, "ACMD", &input, NULL);
}

static acpi_status cmpc_get_accel_v4(acpi_handle handle,
				     int16_t *x,
				     int16_t *y,
				     int16_t *z)
{
	union acpi_object param[4];
	struct acpi_object_list input;
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	int16_t *locs;
	acpi_status status;

	param[0].type = ACPI_TYPE_INTEGER;
	param[0].integer.value = 0x01;
	param[1].type = ACPI_TYPE_INTEGER;
	param[1].integer.value = 0;
	param[2].type = ACPI_TYPE_INTEGER;
	param[2].integer.value = 0;
	param[3].type = ACPI_TYPE_INTEGER;
	param[3].integer.value = 0;
	input.count = 4;
	input.pointer = param;
	status = acpi_evaluate_object(handle, "ACMD", &input, &output);
	if (ACPI_SUCCESS(status)) {
		union acpi_object *obj;
		obj = output.pointer;
		locs = (int16_t *) obj->buffer.pointer;
		*x = locs[0];
		*y = locs[1];
		*z = locs[2];
		kfree(output.pointer);
	}
	return status;
}

static void cmpc_accel_handler_v4(struct acpi_device *dev, u32 event)
{
	if (event == 0x81) {
		int16_t x, y, z;
		acpi_status status;

		status = cmpc_get_accel_v4(dev->handle, &x, &y, &z);
		if (ACPI_SUCCESS(status)) {
			struct input_dev *inputdev = dev_get_drvdata(&dev->dev);

			input_report_abs(inputdev, ABS_X, x);
			input_report_abs(inputdev, ABS_Y, y);
			input_report_abs(inputdev, ABS_Z, z);
			input_sync(inputdev);
		}
	}
}

static ssize_t cmpc_accel_sensitivity_show_v4(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct acpi_device *acpi;
	struct input_dev *inputdev;
	struct cmpc_accel *accel;

	acpi = to_acpi_device(dev);
	inputdev = dev_get_drvdata(&acpi->dev);
	accel = dev_get_drvdata(&inputdev->dev);

	return sprintf(buf, "%d\n", accel->sensitivity);
}

static ssize_t cmpc_accel_sensitivity_store_v4(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct acpi_device *acpi;
	struct input_dev *inputdev;
	struct cmpc_accel *accel;
	unsigned long sensitivity;
	int r;

	acpi = to_acpi_device(dev);
	inputdev = dev_get_drvdata(&acpi->dev);
	accel = dev_get_drvdata(&inputdev->dev);

	r = kstrtoul(buf, 0, &sensitivity);
	if (r)
		return r;

	/* sensitivity must be between 1 and 127 */
	if (sensitivity < 1 || sensitivity > 127)
		return -EINVAL;

	accel->sensitivity = sensitivity;
	cmpc_accel_set_sensitivity_v4(acpi->handle, sensitivity);

	return strnlen(buf, count);
}

static struct device_attribute cmpc_accel_sensitivity_attr_v4 = {
	.attr = { .name = "sensitivity", .mode = 0660 },
	.show = cmpc_accel_sensitivity_show_v4,
	.store = cmpc_accel_sensitivity_store_v4
};

static ssize_t cmpc_accel_g_select_show_v4(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct acpi_device *acpi;
	struct input_dev *inputdev;
	struct cmpc_accel *accel;

	acpi = to_acpi_device(dev);
	inputdev = dev_get_drvdata(&acpi->dev);
	accel = dev_get_drvdata(&inputdev->dev);

	return sprintf(buf, "%d\n", accel->g_select);
}

static ssize_t cmpc_accel_g_select_store_v4(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct acpi_device *acpi;
	struct input_dev *inputdev;
	struct cmpc_accel *accel;
	unsigned long g_select;
	int r;

	acpi = to_acpi_device(dev);
	inputdev = dev_get_drvdata(&acpi->dev);
	accel = dev_get_drvdata(&inputdev->dev);

	r = kstrtoul(buf, 0, &g_select);
	if (r)
		return r;

	/* 0 means 1.5g, 1 means 6g, everything else is wrong */
	if (g_select != 0 && g_select != 1)
		return -EINVAL;

	accel->g_select = g_select;
	cmpc_accel_set_g_select_v4(acpi->handle, g_select);

	return strnlen(buf, count);
}

static struct device_attribute cmpc_accel_g_select_attr_v4 = {
	.attr = { .name = "g_select", .mode = 0660 },
	.show = cmpc_accel_g_select_show_v4,
	.store = cmpc_accel_g_select_store_v4
};

static int cmpc_accel_open_v4(struct input_dev *input)
{
	struct acpi_device *acpi;
	struct cmpc_accel *accel;

	acpi = to_acpi_device(input->dev.parent);
	accel = dev_get_drvdata(&input->dev);

	cmpc_accel_set_sensitivity_v4(acpi->handle, accel->sensitivity);
	cmpc_accel_set_g_select_v4(acpi->handle, accel->g_select);

	if (ACPI_SUCCESS(cmpc_start_accel_v4(acpi->handle))) {
		accel->inputdev_state = CMPC_ACCEL_DEV_STATE_OPEN;
		return 0;
	}
	return -EIO;
}

static void cmpc_accel_close_v4(struct input_dev *input)
{
	struct acpi_device *acpi;
	struct cmpc_accel *accel;

	acpi = to_acpi_device(input->dev.parent);
	accel = dev_get_drvdata(&input->dev);

	cmpc_stop_accel_v4(acpi->handle);
	accel->inputdev_state = CMPC_ACCEL_DEV_STATE_CLOSED;
}

static void cmpc_accel_idev_init_v4(struct input_dev *inputdev)
{
	set_bit(EV_ABS, inputdev->evbit);
	input_set_abs_params(inputdev, ABS_X, -255, 255, 16, 0);
	input_set_abs_params(inputdev, ABS_Y, -255, 255, 16, 0);
	input_set_abs_params(inputdev, ABS_Z, -255, 255, 16, 0);
	inputdev->open = cmpc_accel_open_v4;
	inputdev->close = cmpc_accel_close_v4;
}

#ifdef CONFIG_PM_SLEEP
static int cmpc_accel_suspend_v4(struct device *dev)
{
	struct input_dev *inputdev;
	struct cmpc_accel *accel;

	inputdev = dev_get_drvdata(dev);
	accel = dev_get_drvdata(&inputdev->dev);

	if (accel->inputdev_state == CMPC_ACCEL_DEV_STATE_OPEN)
		return cmpc_stop_accel_v4(to_acpi_device(dev)->handle);

	return 0;
}

static int cmpc_accel_resume_v4(struct device *dev)
{
	struct input_dev *inputdev;
	struct cmpc_accel *accel;

	inputdev = dev_get_drvdata(dev);
	accel = dev_get_drvdata(&inputdev->dev);

	if (accel->inputdev_state == CMPC_ACCEL_DEV_STATE_OPEN) {
		cmpc_accel_set_sensitivity_v4(to_acpi_device(dev)->handle,
					      accel->sensitivity);
		cmpc_accel_set_g_select_v4(to_acpi_device(dev)->handle,
					   accel->g_select);

		if (ACPI_FAILURE(cmpc_start_accel_v4(to_acpi_device(dev)->handle)))
			return -EIO;
	}

	return 0;
}
#endif

static int cmpc_accel_add_v4(struct acpi_device *acpi)
{
	int error;
	struct input_dev *inputdev;
	struct cmpc_accel *accel;

	accel = kmalloc(sizeof(*accel), GFP_KERNEL);
	if (!accel)
		return -ENOMEM;

	accel->inputdev_state = CMPC_ACCEL_DEV_STATE_CLOSED;

	accel->sensitivity = CMPC_ACCEL_SENSITIVITY_DEFAULT;
	cmpc_accel_set_sensitivity_v4(acpi->handle, accel->sensitivity);

	error = device_create_file(&acpi->dev, &cmpc_accel_sensitivity_attr_v4);
	if (error)
		goto failed_sensitivity;

	accel->g_select = CMPC_ACCEL_G_SELECT_DEFAULT;
	cmpc_accel_set_g_select_v4(acpi->handle, accel->g_select);

	error = device_create_file(&acpi->dev, &cmpc_accel_g_select_attr_v4);
	if (error)
		goto failed_g_select;

	error = cmpc_add_acpi_notify_device(acpi, "cmpc_accel_v4",
					    cmpc_accel_idev_init_v4);
	if (error)
		goto failed_input;

	inputdev = dev_get_drvdata(&acpi->dev);
	dev_set_drvdata(&inputdev->dev, accel);

	return 0;

failed_input:
	device_remove_file(&acpi->dev, &cmpc_accel_g_select_attr_v4);
failed_g_select:
	device_remove_file(&acpi->dev, &cmpc_accel_sensitivity_attr_v4);
failed_sensitivity:
	kfree(accel);
	return error;
}

static void cmpc_accel_remove_v4(struct acpi_device *acpi)
{
	device_remove_file(&acpi->dev, &cmpc_accel_sensitivity_attr_v4);
	device_remove_file(&acpi->dev, &cmpc_accel_g_select_attr_v4);
	cmpc_remove_acpi_notify_device(acpi);
}

static SIMPLE_DEV_PM_OPS(cmpc_accel_pm, cmpc_accel_suspend_v4,
			 cmpc_accel_resume_v4);

static const struct acpi_device_id cmpc_accel_device_ids_v4[] = {
	{CMPC_ACCEL_HID_V4, 0},
	{"", 0}
};

static struct acpi_driver cmpc_accel_acpi_driver_v4 = {
	.name = "cmpc_accel_v4",
	.class = "cmpc_accel_v4",
	.ids = cmpc_accel_device_ids_v4,
	.ops = {
		.add = cmpc_accel_add_v4,
		.remove = cmpc_accel_remove_v4,
		.notify = cmpc_accel_handler_v4,
	},
	.drv.pm = &cmpc_accel_pm,
};




/*
 * General init/exit code.
 */

static int cmpc_init(void)
{
	int r;

	r = acpi_bus_register_driver(&cmpc_keys_acpi_driver);
	if (r)
		goto failed_keys;

	r = acpi_bus_register_driver(&cmpc_ipml_acpi_driver);
	if (r)
		goto failed_bl;

	r = acpi_bus_register_driver(&cmpc_tablet_acpi_driver);
	if (r)
		goto failed_tablet;

	r = acpi_bus_register_driver(&cmpc_accel_acpi_driver);
	if (r)
		goto failed_accel;

	r = acpi_bus_register_driver(&cmpc_accel_acpi_driver_v4);
	if (r)
		goto failed_accel_v4;

	return r;

failed_accel_v4:
	acpi_bus_unregister_driver(&cmpc_accel_acpi_driver);

failed_accel:
	acpi_bus_unregister_driver(&cmpc_tablet_acpi_driver);

failed_tablet:
	acpi_bus_unregister_driver(&cmpc_ipml_acpi_driver);

failed_bl:
	acpi_bus_unregister_driver(&cmpc_keys_acpi_driver);

failed_keys:
	return r;
}

static void cmpc_exit(void)
{
	acpi_bus_unregister_driver(&cmpc_accel_acpi_driver_v4);
	acpi_bus_unregister_driver(&cmpc_ipml_acpi_driver);
}

module_init(cmpc_init);
module_exit(cmpc_exit);

static const struct acpi_device_id cmpc_device_ids[] __maybe_unused = {
	{CMPC_ACCEL_HID_V4, 0},
	{CMPC_IPML_HID, 0},
	{"", 0}
};

MODULE_DEVICE_TABLE(acpi, cmpc_device_ids);
MODULE_DESCRIPTION("Support for Intel Classmate PC ACPI devices");
MODULE_LICENSE("GPL");
