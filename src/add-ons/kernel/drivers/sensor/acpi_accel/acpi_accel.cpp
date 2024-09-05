/*
 * Copyright 2024, Sylvain Kerjean, sylvain_kerjean@hotmail.com
 * based on acpi_accel.cpp by Jérôme Duval, jerome.duval@gmail.com.
 * Distributed under the terms of the MIT license.
 */


#include <ACPI.h>
#include <condition_variable.h>
#include <Drivers.h>
#include <Errors.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <kernel.h>


extern "C" {
#	include "acpi.h"
}

#define CMPC_ACCEL_SENSITIVITY_DEFAULT		5
#define CMPC_ACCEL_G_SELECT_DEFAULT		0

struct accel_driver_cookie {
	device_node*			node;
	acpi_device_module_info*	acpi;
	acpi_device			acpi_cookie;
};


struct accel_device_cookie {
	accel_driver_cookie*		driver_cookie;
	int32				stop_watching;
};

struct cmpc_accel {
	int sensitivity;
	int g_select;
};

#define ACPI_ACCEL_DRIVER_NAME "drivers/sensor/acpi_accel/driver_v1"
#define ACPI_ACCEL_DEVICE_NAME "drivers/sensor/acpi_accel/device_v1"

/* Base Namespace devices are published to */
#define ACPI_ACCEL_BASENAME "sensor/acpi_accel/%d"

// name of pnp generator of path ids
#define ACPI_ACCEL_PATHID_GENERATOR "acpi_accel/path_id"

#define ACPI_NAME_ACCEL "ACCE0001"

#define TRACE_ACCEL 1
#ifdef TRACE_ACCEL
#	define TRACE(x...) dprintf("acpi_accel: " x)
#else
#	define TRACE(x...)
#endif
#define ERROR(x...) dprintf("acpi_accel: " x)


static device_manager_info *sDeviceManager;
static ConditionVariable sACCELCondition;


static status_t
acpi_GetInteger(accel_driver_cookie *device,
	const char* path, uint64* number)
{
	acpi_data buf;
	acpi_object_type object;
	buf.pointer = &object;
	buf.length = sizeof(acpi_object_type);

	// Assume that what we've been pointed at is an Integer object, or
	// a method that will return an Integer.
	status_t status = device->acpi->evaluate_method(device->acpi_cookie, path,
		NULL, &buf);
	if (status == B_OK) {
		if (object.object_type == ACPI_TYPE_INTEGER)
			*number = object.integer.integer;
		else
			status = B_BAD_VALUE;
	}
	return status;
}

static acpi_status acpi_SendCommand(accel_driver_cookie *device, int command, int val) {
	acpi_object_type array[4];
	acpi_objects acpi_objects;
	acpi_objects.count = 4;
	acpi_objects.pointer = array;

	array[0].object_type = ACPI_TYPE_INTEGER;
	array[0].integer.integer = command;

	array[1].object_type = ACPI_TYPE_INTEGER;
	array[1].integer.integer = val;

	array[2].object_type = ACPI_TYPE_INTEGER;
	array[2].integer.integer = 0;

	array[3].object_type = ACPI_TYPE_INTEGER;
	array[3].integer.integer = 0;

	return device->acpi->evaluate_method(device->acpi_cookie, "ACMD", &acpi_objects, NULL);
}

void
accel_notify_handler(acpi_handle device, uint32 value, void *context)
{
	TRACE("accel_notify_handler event 0x%" B_PRIx32 "\n", value);
	sACCELCondition.NotifyAll();
}


//	#pragma mark - device module API


static status_t
acpi_accel_init_device(void *driverCookie, void **cookie)
{
	*cookie = driverCookie;
	return B_OK;
}

static void
acpi_accel_uninit_device(void *_cookie)
{

}

static acpi_status cmpc_accel_set_sensitivity_v4(accel_driver_cookie *device, int val)
{
	return acpi_SendCommand(device, 0x02, val);
}

static acpi_status cmpc_accel_set_g_select_v4(accel_driver_cookie *device, int val)
{
	return acpi_SendCommand(device, 0x05, val);
}

static acpi_status cmpc_start_accel_v4(accel_driver_cookie *device) {
	return acpi_SendCommand(device, 0x03, 0);
}

static acpi_status cmpc_get_accel_v4(accel_driver_cookie *device,
		int16_t *x,
		int16_t *y,
		int16_t *z)
{
	acpi_object_type array[4];
	acpi_objects input;
	input.count = 4;
	input.pointer = array;
	acpi_data output;
	int16_t *locs;
	acpi_status status;

	array[0].object_type = ACPI_TYPE_INTEGER;
	array[0].integer.integer = 0x01;
	array[1].object_type = ACPI_TYPE_INTEGER;
	array[1].integer.integer = 0;
	array[2].object_type = ACPI_TYPE_INTEGER;
	array[2].integer.integer = 0;
	array[3].object_type = ACPI_TYPE_INTEGER;
	array[3].integer.integer = 0;
	status = device->acpi->evaluate_method(device->acpi_cookie, "ACMD",  &input, &output);
	if (status == B_OK) {
		acpi_object_type* object = (acpi_object_type*)output.pointer;
		locs = (int16_t *)object->buffer.buffer;
		*x = locs[0];
		*y = locs[1];
		*z = locs[2];
		free(object);
		/*
		   union acpi_object *obj;
		   obj = output.pointer;
		   locs = (int16_t *) obj->buffer.pointer;
		 *x = locs[0];
		 *y = locs[1];
		 *z = locs[2];
		 kfree(output.pointer);*/
	}
	return status;
}


	static status_t
acpi_accel_open(void *initCookie, const char *path, int flags, void** cookie)
{
	accel_device_cookie *device;
	struct cmpc_accel *accel;

	device = (accel_device_cookie*)calloc(1, sizeof(accel_device_cookie));
	if (device == NULL)
		return B_NO_MEMORY;

	device->driver_cookie = (accel_driver_cookie*)initCookie;
	device->stop_watching = 0;

	*cookie = device;

	accel = (cmpc_accel *)calloc(1, sizeof(cmpc_accel));
	if (accel == NULL)
		return B_NO_MEMORY;

	/*TODO : Déplacer dans init_driver*/
	accel->sensitivity = CMPC_ACCEL_SENSITIVITY_DEFAULT;
	accel->g_select = CMPC_ACCEL_G_SELECT_DEFAULT;

	cmpc_accel_set_sensitivity_v4(device->driver_cookie, accel->sensitivity);
	cmpc_accel_set_g_select_v4(device->driver_cookie, accel->g_select);

	if (cmpc_start_accel_v4(device->driver_cookie) == B_OK) {
		return B_OK;
	}
	return B_IO_ERROR;
}


static status_t
acpi_accel_close(void* cookie)
{
	return B_OK;
}

	static status_t
acpi_accel_read(void* _cookie, off_t position, void *buffer, size_t* numBytes)
{
	if (*numBytes < 6)
		return B_IO_ERROR;

	accel_device_cookie *device = (accel_device_cookie*)_cookie;
	int16_t x,y,z;

	if (position == 0) {
		char string[26];
		acpi_status status = cmpc_get_accel_v4(device->driver_cookie, &x, &y, &z);
		if (status != B_OK)
			return B_ERROR;
		snprintf(string, sizeof(string), "x=%" B_PRIu16 ", y=%" B_PRIu16 ", z=%" B_PRIu16 "\n", x, y, z);
		size_t max_len = user_strlcpy((char*)buffer, string, *numBytes);
		if (max_len < B_OK)
			return B_BAD_ADDRESS;
		*numBytes = max_len;
	} else
		*numBytes = 0;

	return B_OK;
}


	static status_t
acpi_accel_write(void* cookie, off_t position, const void* buffer,
		size_t* numBytes)
{
	return B_ERROR;
}


	static status_t
acpi_accel_control(void* _cookie, uint32 op, void* arg, size_t len)
{
	//accel_device_cookie* device = (accel_device_cookie*)_cookie;

	return B_DEV_INVALID_IOCTL;
}


static status_t
acpi_accel_free(void* cookie)
{
	accel_device_cookie* device = (accel_device_cookie*)cookie;
	free(device);
	return B_OK;
}


//	#pragma mark - driver module API


static float
acpi_accel_support(device_node *parent)
{
	// make sure parent is really the ACPI bus manager
	const char *bus;
	if (sDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false))
		return -1;

	if (strcmp(bus, "acpi"))
		return 0.0;

	// check whether it's really a device
	uint32 device_type;
	if (sDeviceManager->get_attr_uint32(parent, ACPI_DEVICE_TYPE_ITEM,
			&device_type, false) != B_OK
		|| device_type != ACPI_TYPE_DEVICE) {
		return 0.0;
	}

	// check whether it's a accel device
	const char *name;
	if (sDeviceManager->get_attr_string(parent, ACPI_DEVICE_HID_ITEM, &name,
		false) != B_OK || strcmp(name, ACPI_NAME_ACCEL)) {
		return 0.0;
	}

	return 0.6;
}


static status_t
acpi_accel_register_device(device_node *node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, { .string = "ACPI ACCEL" }},
		{ NULL }
	};

	return sDeviceManager->register_node(node, ACPI_ACCEL_DRIVER_NAME, attrs,
		NULL, NULL);
}


static status_t
acpi_accel_init_driver(device_node *node, void **driverCookie)
{
	accel_driver_cookie *device;
	device = (accel_driver_cookie *)calloc(1, sizeof(accel_driver_cookie));
	if (device == NULL)
		return B_NO_MEMORY;

	*driverCookie = device;

	device->node = node;

	device_node *parent;
	parent = sDeviceManager->get_parent_node(node);
	sDeviceManager->get_driver(parent, (driver_module_info **)&device->acpi,
		(void **)&device->acpi_cookie);

#ifdef TRACE_ACCEL
	const char* device_path;
	if (sDeviceManager->get_attr_string(parent, ACPI_DEVICE_PATH_ITEM,
		&device_path, false) == B_OK) {
		TRACE("acpi_accel_init_driver %s\n", device_path);
	}
#endif

	sDeviceManager->put_node(parent);

	uint64 sta;
	status_t status = acpi_GetInteger(device, "_STA", &sta);
	uint64 mask = ACPI_STA_DEVICE_PRESENT | ACPI_STA_DEVICE_ENABLED
		| ACPI_STA_DEVICE_FUNCTIONING;
	if (status == B_OK && (sta & mask) != mask) {
		ERROR("acpi_accel_init_driver device disabled\n");
		return B_ERROR;
	}

	// install notify handler
	device->acpi->install_notify_handler(device->acpi_cookie,
		ACPI_ALL_NOTIFY, accel_notify_handler, device);

	return B_OK;
}


static void
acpi_accel_uninit_driver(void *driverCookie)
{
	TRACE("acpi_accel_uninit_driver\n");
	accel_driver_cookie *device = (accel_driver_cookie*)driverCookie;

	device->acpi->remove_notify_handler(device->acpi_cookie,
		ACPI_ALL_NOTIFY, accel_notify_handler);

	free(device);
}


static status_t
acpi_accel_register_child_devices(void *cookie)
{
	accel_driver_cookie *device = (accel_driver_cookie*)cookie;

	int pathID = sDeviceManager->create_id(ACPI_ACCEL_PATHID_GENERATOR);
	if (pathID < 0) {
		ERROR("register_child_devices: couldn't create a path_id\n");
		return B_ERROR;
	}

	char name[128];
	snprintf(name, sizeof(name), ACPI_ACCEL_BASENAME, pathID);

	return sDeviceManager->publish_device(device->node, name,
		ACPI_ACCEL_DEVICE_NAME);
}

module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info **)&sDeviceManager },
	{}
};


driver_module_info acpi_accel_driver_module = {
	{
		ACPI_ACCEL_DRIVER_NAME,
		0,
		NULL
	},

	acpi_accel_support,
	acpi_accel_register_device,
	acpi_accel_init_driver,
	acpi_accel_uninit_driver,
	acpi_accel_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info acpi_accel_device_module = {
	{
		ACPI_ACCEL_DEVICE_NAME,
		0,
		NULL
	},

	acpi_accel_init_device,
	acpi_accel_uninit_device,
	NULL,

	acpi_accel_open,
	acpi_accel_close,
	acpi_accel_free,
	acpi_accel_read,
	acpi_accel_write,
	NULL,
	acpi_accel_control,

	NULL,
	NULL
};

module_info *modules[] = {
	(module_info *)&acpi_accel_driver_module,
	(module_info *)&acpi_accel_device_module,
	NULL
};
