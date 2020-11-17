/*
 * Copyright 2018-2020 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		B Krishnan Iyer, krishnaniyer97@gmail.com
 */
#include <new>
#include <stdio.h>
#include <string.h>

#include <bus/PCI.h>
#include <PCI_x86.h>

#include <KernelExport.h>

#include "mmc.h"
#include "sdhci_pci.h"


#define TRACE_SDHCI 1
#ifdef TRACE_SDHCI
#	define TRACE(x...) dprintf("\33[33msdhci_pci:\33[0m " x)
#else
#	define TRACE(x...) ;
#endif
#define TRACE_ALWAYS(x...)	dprintf("\33[33msdhci_pci:\33[0m " x)
#define ERROR(x...)			dprintf("\33[33msdhci_pci:\33[0m " x)
#define CALLED(x...)		TRACE("CALLED %s\n", __PRETTY_FUNCTION__)


#define SDHCI_PCI_DEVICE_MODULE_NAME "busses/mmc/sdhci_pci/driver_v1"
#define SDHCI_PCI_MMC_BUS_MODULE_NAME "busses/mmc/sdhci_pci/device/v1"

#define SLOTS_COUNT				"device/slots_count"
#define SLOT_NUMBER				"device/slot"
#define BAR_INDEX				"device/bar"


class SdhciBus {
	public:
							SdhciBus(uint8_t slot, struct registers* registers, uint8_t irq);
							~SdhciBus();

		void				EnableInterrupts(uint32_t mask);
		status_t			ExecuteCommand(uint8_t command, uint32_t argument,
								uint32_t* response);
		int32				HandleInterrupt();
		status_t			InitCheck();
		void				Reset();
		void				SetClock(int kilohertz);
		status_t			ReadNaive(uint16_t rca, off_t pos, void* buffer, size_t* _length);
		
	private:
		void				DumpRegisters(uint8_t slot);
		bool				PowerOn();
		void				RecoverError();

	private:
		uint8_t				fSlot;
		struct registers*	fRegisters;
		uint32_t			fCommandResult;
		uint8_t				fIrq;
		sem_id				fSemaphore;
		sem_id				fSemTransfer;
		sem_id				fSemRead;
		status_t			fStatus;
};


device_manager_info* gDeviceManager;
device_module_info* gMMCBusController;
static pci_x86_module_info* sPCIx86Module;


static int32
sdhci_generic_interrupt(void* data)
{
	SdhciBus* bus = (SdhciBus*)data;
	return bus->HandleInterrupt();
}


SdhciBus::SdhciBus(uint8_t slot, struct registers* registers, uint8_t irq)
	:
	fSlot(slot),
	fRegisters(registers),
	fIrq(irq),
	fSemaphore(0),
	fSemTransfer(0),
	fSemRead(0)
{
	if (irq == 0 || irq == 0xff) {
		ERROR("PCI IRQ not assigned\n");
		fStatus = B_BAD_DATA;
		return;
	}

	fSemaphore = create_sem(0, "SDHCI interrupts");
	fSemTransfer = create_sem(0, "SDHCI Transfer");
	fSemRead   = create_sem(0, "SDHCI buffer read");
	
	fStatus = install_io_interrupt_handler(fIrq,
		sdhci_generic_interrupt, this, 0);

	if (fStatus != B_OK) {
		ERROR("can't install interrupt handler\n");
		return;
	}

	// First of all, we have to make sure we are in a sane state. The easiest
	// way is to reset everything.
	Reset();

	// Then we configure the clock to the frequency needed for initialization
	SetClock(400);

	// And we turn on the power supply to the card
	// FIXME maybe this should only be done when a card is inserted?
	if (!PowerOn()) {
		ERROR("Failed to power on the card\n");
		fStatus = B_NO_INIT;
		return;
	}

	// FIXME do we need all these? Wouldn't card insertion/removal and command
	// completion be enough?
	EnableInterrupts(SDHCI_INT_CMD_CMP
		| SDHCI_INT_TRANS_CMP | SDHCI_INT_BUF_READ_READY | SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM
		| SDHCI_INT_TIMEOUT | SDHCI_INT_CRC | SDHCI_INT_INDEX
		| SDHCI_INT_BUS_POWER | SDHCI_INT_END_BIT);

	fRegisters->interrupt_status_enable |= SDHCI_INT_ERROR;
}


SdhciBus::~SdhciBus()
{
	if (fSemaphore != 0)
		delete_sem(fSemaphore);
	if (fSemTransfer != 0)
		delete_sem(fSemTransfer);
	if (fSemRead != 0)
		delete_sem(fSemRead);

	EnableInterrupts(0);
	if (fIrq != 0)
		remove_io_interrupt_handler(fIrq, sdhci_generic_interrupt, this);

	area_id regs_area = area_for(fRegisters);
	delete_area(regs_area);
}


void
SdhciBus::DumpRegisters(uint8_t slot)
{
#ifdef TRACE_SDHCI
	TRACE("Register values for slot %d:\n", slot);
	TRACE("system_address: %d\n", fRegisters->system_address);
	TRACE("%d blocks of size %d\n", fRegisters->block_count,
		fRegisters->block_size);
	TRACE("argument: %x\n", fRegisters->argument);
	TRACE("transfer_mode: %x\n", fRegisters->transfer_mode.Bits());
	TRACE("command: %x\n", fRegisters->command.Bits());
	TRACE("response:");
	for (int i = 0; i < 4; i++)
		dprintf(" %d", fRegisters->response[i]);
	dprintf("\n");
	TRACE("buffer_data_port: %d\n", fRegisters->buffer_data_port);
	TRACE("present_state: %x\n", fRegisters->present_state.Bits());
	TRACE("power_control: %d\n", fRegisters->power_control.Bits());
	TRACE("host_control: %d\n", fRegisters->host_control);
	TRACE("wakeup_control: %d\n", fRegisters->wakeup_control);
	TRACE("block_gap_control: %d\n", fRegisters->block_gap_control);
	TRACE("clock_control: %x\n", fRegisters->clock_control.Bits());
	TRACE("software_reset: %d\n", fRegisters->software_reset.Bits());
	TRACE("timeout_control: %d\n", fRegisters->timeout_control);
	TRACE("interrupt_status: %x enable: %x signal: %x\n",
		fRegisters->interrupt_status, fRegisters->interrupt_status_enable,
		fRegisters->interrupt_signal_enable);
	TRACE("auto_cmd12_error_status: %d\n", fRegisters->auto_cmd12_error_status);
	TRACE("capabilities: %lld\n", fRegisters->capabilities.Bits());
	TRACE("max_current_capabilities: %lld\n",
		fRegisters->max_current_capabilities);
	TRACE("slot_interrupt_status: %d\n", fRegisters->slot_interrupt_status);
	TRACE("host_controller_version spec %x vendor %x\n",
		fRegisters->host_controller_version.specVersion,
		fRegisters->host_controller_version.vendorVersion);
#endif
}


void
SdhciBus::EnableInterrupts(uint32_t mask)
{
	fRegisters->interrupt_status_enable = mask;
	fRegisters->interrupt_signal_enable = mask;
}

// #pragma mark -
/*
PartA2, SD Host Controller Simplified Specification, Version 4.20
§3.7.1.1 The sequence to issue an SD Command
*/
status_t
SdhciBus::ExecuteCommand(uint8_t command, uint32_t argument, uint32_t* response)
{
	TRACE("ExecuteCommand(%d, %x)\n", command, argument);
	TRACE("Present state at beginning of command : %04x\n", fRegisters->present_state.Bits());

	//1) Check Command Inhibit (CMD)
	while (fRegisters->present_state.CommandInhibitCMD()) {
		//wait for CMD line to be free
	}
	TRACE("CMD line free\n");
	//TODO specification mandates looping, but maybe we should wait on a 
	//semaphore  or do the following
	//ERROR("Execution aborted, command inhibit (CMD)\n");
	//	return B_BUSY;
	

	fRegisters->argument = argument;

	uint32_t replyType;

	switch(command) {
		case GO_IDLE_STATE:
			replyType = Command::kNoReplyType;
			break;
		case ALL_SEND_CID:
		case SEND_CSD:
			replyType = Command::kR2Type;
			break;
		case SEND_RELATIVE_ADDR:
			replyType = Command::kR6Type;
			break;
		case SELECT_DESELECT_CARD:
			replyType = Command::kR1bType;
			break;
		case SEND_IF_COND:
			replyType = Command::kR7Type;
			break;
		case READ_SINGLE_BLOCK:
			replyType = Command::kR1Type;
			break;
		case APP_CMD:
			replyType = Command::kR1Type;
			break;
		case 41: // ACMD
			replyType = Command::kR3Type;
			break;
		default:
			ERROR("Unknown command\n");
			return B_BAD_DATA;
	}

	//2) Issue the command using DAT busy ?
	if ( (replyType & Command::k32BitResponseCheckBusy) 
		== Command::k32BitResponseCheckBusy ) {
		//3) Issuing Abort Command ?
		if ( ! (command == STOP_TRANSMISSION || command == IO_ABORT)) {
			//4) Check Command Inhibit (DAT)
			TRACE("Before waiting the dat line to be free\n");
			while (fRegisters->present_state.CommandInhibitDAT()) {
				//wait for DAT line to be free
				
				//TODO specification mandates looping, but maybe we should wait
				// on a semaphore or do the following :
				//ERROR("Execution aborted, command inhibit (CMD)\n");
				//return B_BUSY;
			}
			TRACE("Dat line to be free\n");
			
		}
	}
	
	//FIXME : Assign only at this point, if needed :
	// -32 bit block count/SDMA system address
	// -block size
	// -16-bit block count
	// -argument
	// -transfer mode
	TRACE("Present state before SendCommand : %04x\n", fRegisters->present_state.Bits());
		
	fRegisters->command.SendCommand(command, replyType);

	for (int i = 1 ; i <=10; i++) {
		spin(1000000);
		TRACE("Present state(%d) after SendCommand : %04x\n", i,fRegisters->present_state.Bits());
		
	}
	acquire_sem(fSemaphore);
	
	if (fCommandResult & SDHCI_INT_ERROR) {
		fRegisters->interrupt_status |= fCommandResult;
		if (fCommandResult & SDHCI_INT_TIMEOUT) {
			ERROR("Command execution timed out\n");
			return B_TIMED_OUT;
		}
		if (fCommandResult & SDHCI_INT_CRC) {
			ERROR("CRC error\n");
			return B_BAD_VALUE;
		}
		ERROR("Command execution failed %x\n", fCommandResult);
		// TODO look at errors in interrupt_status register for more details
		// and return a more appropriate error code
		return B_ERROR;
	}
	
	if (fRegisters->present_state.CommandInhibitCMD()) {
		TRACE("Command execution failed, card stalled\n");
		// Clear the stall
		fRegisters->software_reset.ResetCommandLine();
		return B_ERROR;
	}

	switch (replyType & Command::kReplySizeMask) {
		case Command::k32BitResponse:
			*response = fRegisters->response[0];
			break;
		case Command::k128BitResponse:
			response[0] = fRegisters->response[0];
			response[1] = fRegisters->response[1];
			response[2] = fRegisters->response[2];
			response[3] = fRegisters->response[3];
			break;

		//TODO in case replyType is R1b, test if transfer_complete is present, if not, acquire_sem(fSemTransfer)

		default:
			// No response
			break;
	}

	ERROR("Command execution %d complete\n", command);
	TRACE("Present state at end of command : %04x\n", fRegisters->present_state.Bits());
	return B_OK;
}


status_t
SdhciBus::InitCheck()
{
	return fStatus;
}


void
SdhciBus::Reset()
{
	fRegisters->software_reset.ResetAll();
}


void
SdhciBus::SetClock(int kilohertz)
{
	int base_clock = fRegisters->capabilities.BaseClockFrequency();
	// Try to get as close to 400kHz as possible, but not faster
	int divider = base_clock * 1000 / kilohertz;

	if (fRegisters->host_controller_version.specVersion <= 1) {
		// Old controller only support power of two dividers up to 256,
		// round to next power of two up to 256
		if (divider > 256)
			divider = 256;

		divider--;
		divider |= divider >> 1;
		divider |= divider >> 2;
		divider |= divider >> 4;
		divider++;
	}

	divider = fRegisters->clock_control.SetDivider(divider);

	// Log the value after possible rounding by SetDivider (only even values
	// are allowed).
	TRACE("SDCLK frequency: %dMHz / %d = %dkHz\n", base_clock, divider,
		base_clock * 1000 / divider);

	// We have set the divider, now we can enable the internal clock.
	fRegisters->clock_control.EnableInternal();

	// wait until internal clock is stabilized
	while (!(fRegisters->clock_control.InternalStable()));

	fRegisters->clock_control.EnablePLL();
	while (!(fRegisters->clock_control.InternalStable()));

	// Finally, route the clock to the SD card
	fRegisters->clock_control.EnableSD();
}

// #pragma mark -
status_t SdhciBus::ReadNaive(uint16_t rca, off_t pos, void* buffer, 
	size_t* _length) {
	
	//Select card
	uint32_t response;
	ExecuteCommand(SELECT_DESELECT_CARD, rca <<16, &response);
	
	//TODO instead of waiting for Transfer complete here, 
	//implement in ExecuteCommand the TODO  : in case where replyType is R1b, 
	//test if transfer_complete is present, if not, acquire_sem(fSemTransfer) there 
	acquire_sem(fSemTransfer);
	
	//1) Set block size reg	
	fRegisters->block_size = 512;
	
	//2) Set block count
	fRegisters->block_count = 1;
	
	//3) Set argument register
	// not needed, it will be passed through the argument parameter of 
	//ExecuteCommand
	
	//4) Set transfer mode
	fRegisters->transfer_mode.SetBlockCountEnable(true);
	DumpRegisters(fSlot);
	fRegisters->transfer_mode.SetMultiSingleBlockSelect(TransferMode::kSingle);
	fRegisters->transfer_mode.SetDataTransferDirectionSelect(TransferMode::kRead);
	//Not needed since Multi/Single Block Select is 0
	//fRegisters->transfer_mode.SetBlockCountEnable(false); 
	fRegisters->transfer_mode.SetAutoCmdEnable(TransferMode::kAutoCmdDisabled);
	fRegisters->transfer_mode.SetDmaEnable(TransferMode::kNoDmaOrNoData);
	//Response Error Check by Host Controller not useful yet, maybe for ADMA3 to speed up.
	fRegisters->transfer_mode.SetResponseErrorCheckEnable(true ); 
	if (fRegisters->transfer_mode.IsResponseErrorCheckEnable()) {
		fRegisters->transfer_mode.SetResponseInterruptDisable(true);
		fRegisters->transfer_mode.SetResponseTypeR1R5(TransferMode::kR1);//Memory
	}
	//DumpRegisters(fSlot);
	//5) Set the value to Command register
	//6) Wait for Command Complete interrupt (as "response check enable" is false, no need to go to stop
	//7) Write 1 to Command Complete register
	//8) Read Response register
	ExecuteCommand(READ_SINGLE_BLOCK, pos, &response);
	//DumpRegisters(fSlot);
	
	//15) read block data from Buffer Data Port register
	acquire_sem(fSemRead);
	size_t to_read = *_length;
	size_t read = 0;
	while(to_read > 0) {
		TRACE("read : 0x%x", fRegisters->buffer_data_port);
		read++;
		to_read--;
	}
	*_length = read;
	return B_OK;	
}

static void
sdhci_stop_clock(struct registers* regs)
{
	regs->clock_control.DisableSD();
}


bool
SdhciBus::PowerOn()
{
	if (!fRegisters->present_state.IsCardInserted()) {
		TRACE("Card not inserted, not powering on for now\n");
		return false;
	}

	uint8_t supportedVoltages = fRegisters->capabilities.SupportedVoltages();
	if ((supportedVoltages & Capabilities::k3v3) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k3v3);
	else if ((supportedVoltages & Capabilities::k3v0) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k3v0);
	else if ((supportedVoltages & Capabilities::k1v8) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k1v8);
	else {
		fRegisters->power_control.PowerOff();
		ERROR("No voltage is supported\n");
		return false;
	}

	return true;
}


static status_t
init_bus(device_node* node, void** bus_cookie)
{
	CALLED();

	// Get the PCI driver and device
	pci_device_module_info* pci;
	pci_device* device;

	device_node* parent = gDeviceManager->get_parent_node(node);
	device_node* pciParent = gDeviceManager->get_parent_node(parent);
	gDeviceManager->get_driver(pciParent, (driver_module_info**)&pci,
	        (void**)&device);
	gDeviceManager->put_node(pciParent);
	gDeviceManager->put_node(parent);

	if (get_module(B_PCI_X86_MODULE_NAME, (module_info**)&sPCIx86Module)
	    != B_OK) {
	    sPCIx86Module = NULL;
		ERROR("PCIx86Module not loaded\n");
		// FIXME try probing FDT as well
		return -1;
	}

	uint8_t bar, slot;
	if (gDeviceManager->get_attr_uint8(node, SLOT_NUMBER, &slot, false) < B_OK
		|| gDeviceManager->get_attr_uint8(node, BAR_INDEX, &bar, false) < B_OK)
		return -1;

	TRACE("Register SD bus at slot %d, using bar %d\n", slot + 1, bar);

	pci_info pciInfo;
	pci->get_pci_info(device, &pciInfo);
	int msiCount = sPCIx86Module->get_msi_count(pciInfo.bus,
		pciInfo.device, pciInfo.function);
	TRACE("interrupts count: %d\n",msiCount);
	// FIXME if available, use MSI rather than good old IRQ...

	// enable bus master and io
	uint16 pcicmd = pci->read_pci_config(device, PCI_command, 2);
	pcicmd &= ~(PCI_command_int_disable | PCI_command_io);
	pcicmd |= PCI_command_master | PCI_command_memory;
	pci->write_pci_config(device, PCI_command, 2, pcicmd);

	// map the slot registers
	area_id	regs_area;
	struct registers* _regs;
	regs_area = map_physical_memory("sdhc_regs_map",
		pciInfo.u.h0.base_registers[bar],
		pciInfo.u.h0.base_register_sizes[bar], B_ANY_KERNEL_BLOCK_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, (void**)&_regs);

	if (regs_area < B_OK) {
		ERROR("Could not map registers\n");
		return B_BAD_VALUE;
	}

	// the interrupt is shared between all busses in an SDHC controller, but
	// they each register an handler. Not a problem, we will just test the
	// interrupt registers for all busses one after the other and find no
	// interrupts on the idle busses.
	uint8_t irq = pciInfo.u.h0.interrupt_line;
	TRACE("irq interrupt line: %d\n", irq);

	SdhciBus* bus = new(std::nothrow) SdhciBus(slot, _regs, irq);

	status_t status = B_NO_MEMORY;
	if (bus != NULL)
		status = bus->InitCheck();

	if (status != B_OK) {
		if (sPCIx86Module != NULL) {
			put_module(B_PCI_X86_MODULE_NAME);
			sPCIx86Module = NULL;
		}

		if (bus != NULL)
			delete bus;
		else
			delete_area(regs_area);
		return status;
	}

	// Store the created object as a cookie, allowing users of the bus to
	// locate it.
	*bus_cookie = bus;

	return status;
}


static void
uninit_bus(void* bus_cookie)
{
	SdhciBus* bus = (SdhciBus*)bus_cookie;
	delete bus;

	// FIXME do we need to put() the PCI module here?
}


void
SdhciBus::RecoverError()
{
	fRegisters->interrupt_signal_enable &= ~(SDHCI_INT_CMD_CMP
		| SDHCI_INT_TRANS_CMP | SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM);

	if (fRegisters->interrupt_status & 7)
		fRegisters->software_reset.ResetCommandLine();

	int16_t error_status = fRegisters->interrupt_status;
	fRegisters->interrupt_status &= ~(error_status);
}

// #pragma mark -
int32
SdhciBus::HandleInterrupt()
{
	CALLED();
	uint32_t intmask = fRegisters->interrupt_status;
	bool handled = false;
	
	if ((intmask == 0) || (intmask == 0xffffffff)) {
		return B_UNHANDLED_INTERRUPT;
	}

	TRACE("interrupt function called %x\n", intmask);

	// FIXME use the global "slot interrupt" register to quickly decide if an
	// interrupt is targetted to this slot
	if ( !(fRegisters->slot_interrupt_status & (1 << fSlot)) ) {
		TRACE("interrupt not for me.\n");
		return B_UNHANDLED_INTERRUPT;
	}
	
	// handling card presence interrupt
	if (intmask & (SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM)) {
		uint32_t card_present = ((intmask & SDHCI_INT_CARD_INS) != 0);
		fRegisters->interrupt_status_enable &= ~(SDHCI_INT_CARD_INS
			| SDHCI_INT_CARD_REM);
		fRegisters->interrupt_signal_enable &= ~(SDHCI_INT_CARD_INS
			| SDHCI_INT_CARD_REM);

		fRegisters->interrupt_status_enable |= card_present
		 	? SDHCI_INT_CARD_REM : SDHCI_INT_CARD_INS;
		fRegisters->interrupt_signal_enable |= card_present
			? SDHCI_INT_CARD_REM : SDHCI_INT_CARD_INS;

		fRegisters->interrupt_status |= (intmask &
			(SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM));
		TRACE("Card presence interrupt handled\n");

		return B_HANDLED_INTERRUPT;
	}

	// handling command interrupt
	if (intmask & SDHCI_INT_CMD_MASK) {
		fCommandResult = intmask;
		// Save the status before clearing so the thhread can handle it
		fRegisters->interrupt_status |= (intmask & SDHCI_INT_CMD_MASK);
		TRACE("Present state in handling interrupt: %04x\n", fRegisters->present_state.Bits());

		// Notify the thread
		release_sem_etc(fSemaphore, 1, B_DO_NOT_RESCHEDULE);
		TRACE("Command complete interrupt handled\n");
		handled = true;
	}

	// handling transfer interrupt
	if (intmask & SDHCI_INT_TRANS_CMP) {
		fRegisters->interrupt_status |= (intmask & SDHCI_INT_TRANS_CMP);
		// Notify the thread
		release_sem_etc(fSemTransfer, 1, B_DO_NOT_RESCHEDULE);
		TRACE("Transfer complete interrupt handled\n");
		handled = true;
	}
	
	// handling data transfer interrupt
	//14) wait for buffer read ready interrupt (the wait is done through the semaphore fSemRead)
	if (intmask & SDHCI_INT_BUF_READ_READY) {
		TRACE("buffer read ready interrupt raised");
		//15) clear the "buffer read ready" interrupt  bit
		fRegisters->interrupt_status |= (intmask & SDHCI_INT_BUF_READ_READY);
		//15) read block data from Buffer Data Port register
		release_sem_etc(fSemRead, 1, B_DO_NOT_RESCHEDULE);	
		handled = true;
	}
	

	// handling bus power interrupt
	if (intmask & SDHCI_INT_BUS_POWER) {
		fRegisters->interrupt_status |= SDHCI_INT_BUS_POWER;
		TRACE("card is consuming too much power\n");

		return B_HANDLED_INTERRUPT;
	}

	intmask = fRegisters->slot_interrupt_status;
	if (intmask != 0) {
		ERROR("Remaining interrupts at end of handler: %x\n", intmask);
	}
	
	if (handled)
		return B_HANDLED_INTERRUPT;
	else
		return B_UNHANDLED_INTERRUPT;
}
// #pragma mark -


static void
bus_removed(void* bus_cookie)
{
	return;
}


static status_t
register_child_devices(void* cookie)
{
	CALLED();
	device_node* node = (device_node*)cookie;
	device_node* parent = gDeviceManager->get_parent_node(node);
	pci_device_module_info* pci;
	pci_device* device;
	uint8 slots_count, bar, slotsInfo;

	gDeviceManager->get_driver(parent, (driver_module_info**)&pci,
		(void**)&device);
	slotsInfo = pci->read_pci_config(device, SDHCI_PCI_SLOT_INFO, 1);
	bar = SDHCI_PCI_SLOT_INFO_FIRST_BASE_INDEX(slotsInfo);
	slots_count = SDHCI_PCI_SLOTS(slotsInfo);

	char prettyName[25];

	if (slots_count > 6 || bar > 5) {
		ERROR("Invalid slots count: %d or BAR count: %d \n", slots_count, bar);
		return B_BAD_VALUE;
	}

	for (uint8_t slot = 0; slot <= slots_count; slot++) {

		bar = bar + slot;
		sprintf(prettyName, "SDHC bus %" B_PRIu8, slot);
		device_attr attrs[] = {
			// properties of this controller for SDHCI bus manager
			{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, { string: prettyName } },
			{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
				{string: MMC_BUS_MODULE_NAME} },
			{ B_DEVICE_BUS, B_STRING_TYPE, {string: "mmc"} },
			{ SLOT_NUMBER, B_UINT8_TYPE, { ui8: slot} },
			{ BAR_INDEX, B_UINT8_TYPE, { ui8: bar} },
			{ NULL }
		};
		if (gDeviceManager->register_node(node, SDHCI_PCI_MMC_BUS_MODULE_NAME,
				attrs, NULL, &node) != B_OK)
			return B_BAD_VALUE;
	}
	return B_OK;
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	CALLED();
	*device_cookie = node;
	return B_OK;
}


static status_t
register_device(device_node* parent)
{
	device_attr attrs[] = {
		{B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {string: "SD Host Controller"}},
		{}
	};

	return gDeviceManager->register_node(parent, SDHCI_PCI_DEVICE_MODULE_NAME,
		attrs, NULL, NULL);
}


static float
supports_device(device_node* parent)
{
	CALLED();
	const char* bus;
	uint16 type, subType;
	uint8 pciSubDeviceId;

	// make sure parent is a PCI SDHCI device node
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
		!= B_OK || gDeviceManager->get_attr_uint16(parent, B_DEVICE_SUB_TYPE,
		&subType, false) < B_OK || gDeviceManager->get_attr_uint16(parent,
		B_DEVICE_TYPE, &type, false) < B_OK) {
		ERROR("Could not find required attribute device/bus\n");
		return -1;
	}

	if (strcmp(bus, "pci") != 0)
		return 0.0f;

	if (type == PCI_base_peripheral) {
		if (subType != PCI_sd_host)
			return 0.0f;

		pci_device_module_info* pci;
		pci_device* device;
		gDeviceManager->get_driver(parent, (driver_module_info**)&pci,
			(void**)&device);
		pciSubDeviceId = pci->read_pci_config(device, PCI_revision, 1);
		TRACE("SDHCI Device found! Subtype: 0x%04x, type: 0x%04x\n",
			subType, type);
		return 0.8f;
	}

	return 0.0f;
}


static status_t
set_clock(void* controller, uint32_t kilohertz)
{
	SdhciBus* bus = (SdhciBus*)controller;
	bus->SetClock(kilohertz);
	return B_OK;
}


static status_t
execute_command(void* controller, uint8_t command, uint32_t argument,
	uint32_t* response)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->ExecuteCommand(command, argument, response);
}


//Very naive read protocol : non DMA, 32 bits at a time (size of Buffer Data Port)
static status_t 
read_naive(void* controller, uint16_t rca, off_t pos, void* buffer, size_t* _length) 
{
	CALLED();
	TRACE("read_native : truncate to first 512 bytes\n");
	
	SdhciBus* bus = (SdhciBus*)controller;
	off_t pos_0 = 0;
	*_length=512;
	return bus->ReadNaive(rca, pos_0, buffer, _length);
	
}
module_dependency module_dependencies[] = {
	{ MMC_BUS_MODULE_NAME, (module_info**)&gMMCBusController},
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};


// Device node registered for each SD slot. It implements the MMC operations so
// the bus manager can use it to communicate with SD cards.
static mmc_bus_interface gSDHCIPCIDeviceModule = {
	{
		{
			SDHCI_PCI_MMC_BUS_MODULE_NAME,
			0,
			NULL
		},
		NULL,	// supports device
		NULL,	// register device
		init_bus,
		uninit_bus,
		NULL,	// register child devices
		NULL,	// rescan
		bus_removed,
	},

	set_clock,
	execute_command,
	read_naive
};


// Root device that binds to the PCI bus. It will register an mmc_bus_interface
// node for each SD slot in the device.
static driver_module_info sSDHCIDevice = {
	{
		SDHCI_PCI_DEVICE_MODULE_NAME,
		0,
		NULL
	},
	supports_device,
	register_device,
	init_device,
	NULL,	// uninit
	register_child_devices,
	NULL,	// rescan
	NULL,	// device removed
};


module_info* modules[] = {
	(module_info* )&sSDHCIDevice,
	(module_info* )&gSDHCIPCIDeviceModule,
	NULL
};
