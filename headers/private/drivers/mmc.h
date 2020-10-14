/*
 * Copyright 2019-2020, Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		Adrien Destugues, pulkomandy@pulkomandy.tk
 */
#ifndef _MMC_H
#define _MMC_H


#include <device_manager.h>


#define MMC_BUS_MODULE_NAME "bus_managers/mmc_bus/driver_v1"


enum {
	CARD_TYPE_MMC,
	CARD_TYPE_SD,
	CARD_TYPE_SDHC,
	CARD_TYPE_UHS1,
	CARD_TYPE_UHS2,
	CARD_TYPE_SDIO
};


enum SDHCI_COMMANDS {
	//Basic commands, class 0
	GO_IDLE_STATE = 0,
	ALL_SEND_CID = 2,
	SEND_RELATIVE_ADDR = 3,
	SEND_IF_COND = 8,
	SEND_CSD = 9,
	
	//Block oriented read commands, class 2
	READ_SINGLE_BLOCK = 17,
	
	//Application specific commands, class 8
	APP_CMD = 55,
};

enum SDHCI_APPLICATION_COMMANDS {
	SD_SEND_OP_COND = 41,
};

enum VOLTAGE_SUPPLIED {
	HOST_27_36V = 1, //Host supplied voltage 2.7-3.6V
};

#define MULTI_SINGLE_BLOCK_SELECT (1 << 5)


#define VOLTAGE_CHECK_PATTERN 0xAA



// Interface between mmc_bus and underlying implementation (sdhci_pci or any
// other thing that can execute mmc commands)
typedef struct mmc_bus_interface {
	driver_module_info info;

	status_t (*set_clock)(void* controller, uint32_t kilohertz);
	status_t (*execute_command)(void* controller, uint8_t command,
		uint32_t argument, uint32_t* result);
	status_t (*read_naive)(void* controller, off_t pos, void* buffer, size_t* _length);
} mmc_bus_interface;


// Interface between mmc device driver (mmc_disk, sdio drivers, ...) and mmc_bus
typedef struct mmc_device_interface {
	driver_module_info info;
	status_t (*execute_command)(device_node* node, uint8_t command,
		uint32_t argument, uint32_t* result);	
	status_t (*read_naive)(void* controller, off_t pos, void* buffer, size_t* _length);
			
} mmc_device_interface;


#endif /* _MMC_H */
