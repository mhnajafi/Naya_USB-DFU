/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2017 Phytec Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample app for USB DFU class driver. */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>



#include <assert.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>


LOG_MODULE_REGISTER(main);

#define MCUBOOT_PARTITION		mcuboot_partition

#define CONFIG_MCUBOOT_CLEANUP_ARM_CORE     1
#define CONFIG_CPU_HAS_ARM_MPU              1
#define CONFIG_MCUBOOT_CLEANUP_ARM_CORE     1




#define NEW_APP_ADDRESS 0x27000  

#define BOOTLOADER_DFU_START 			0xB1
#define DFU_MAGIC_OTA_APPJUM            BOOTLOADER_DFU_START  // 0xB1
#define DFU_MAGIC_OTA_RESET             0xA8
#define DFU_MAGIC_SERIAL_ONLY_RESET     0x4e
#define DFU_MAGIC_UF2_RESET             0x57
#define DFU_MAGIC_SKIP                  0x6d

#define DFU_DBL_RESET_MAGIC             0x5A1AD5      // SALADS
#define DFU_DBL_RESET_APP               0x4ee5677e
#define DFU_DBL_RESET_DELAY             500
#define DFU_DBL_RESET_MEM               0x20007F7C

#define MBR_SIZE                (0x1000)
#define SOFTDEVICE_INFO_STRUCT_OFFSET (0x2000)
#define SD_SIZE_OFFSET (SOFTDEVICE_INFO_STRUCT_OFFSET + 0x08)
#define SD_SIZE_GET(baseaddr) (*((uint32_t *) ((baseaddr) + SD_SIZE_OFFSET)))


struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

#if defined ( __CC_ARM )
__asm static void bootloader_util_reset(uint32_t start_addr)
{
    LDR   R5, [R0]              ; Get App initial MSP for bootloader.
    MSR   MSP, R5               ; Set the main stack pointer to the applications MSP.
    LDR   R0, [R0, #0x04]       ; Load Reset handler into R0. This will be first argument to branch instruction (BX).

    MOVS  R4, #0xFF             ; Load ones to R4.
    SXTB  R4, R4                ; Sign extend R4 to obtain 0xFFFFFFFF instead of 0xFF.
    MRS   R5, IPSR              ; Load IPSR to R5 to check for handler or thread mode.
    CMP   R5, #0x00             ; Compare, if 0 then we are in thread mode and can continue to reset handler of bootloader.
    BNE   isr_abort             ; If not zero we need to exit current ISR and jump to reset handler of bootloader.

    MOV   LR, R4                ; Clear the link register and set to ones to ensure no return, R4 = 0xFFFFFFFF.
    BX    R0                    ; Branch to reset handler of bootloader.

isr_abort
                                ; R4 contains ones from line above. Will be popped as R12 when exiting ISR (Cleaning up the registers).
    MOV   R5, R4                ; Fill with ones before jumping to reset handling. We be popped as LR when exiting ISR. Ensures no return to application.
    MOV   R6, R0                ; Move address of reset handler to R6. Will be popped as PC when exiting ISR. Ensures the reset handler will be executed when exist ISR.
    MOVS  r7, #0x21             ; Move MSB reset value of xPSR to R7. Will be popped as xPSR when exiting ISR. xPSR is 0x21000000 thus MSB is 0x21.
    REV   r7, r7                ; Reverse byte order to put 0x21 as MSB.
    PUSH  {r4-r7}               ; Push everything to new stack to allow interrupt handler to fetch it on exiting the ISR.

    MOVS  R4, #0x00             ; Fill with zeros before jumping to reset handling. We be popped as R0 when exiting ISR (Cleaning up of the registers).
    MOVS  R5, #0x00             ; Fill with zeros before jumping to reset handling. We be popped as R1 when exiting ISR (Cleaning up of the registers).
    MOVS  R6, #0x00             ; Fill with zeros before jumping to reset handling. We be popped as R2 when exiting ISR (Cleaning up of the registers).
    MOVS  R7, #0x00             ; Fill with zeros before jumping to reset handling. We be popped as R3 when exiting ISR (Cleaning up of the registers).
    PUSH  {r4-r7}               ; Push zeros (R4-R7) to stack to prepare for exiting the interrupt routine.

    MOVS  R0, #0xF9             ; Move the execution return command into register, 0xFFFFFFF9.
    SXTB  R0, R0                ; Sign extend R0 to obtain 0xFFFFFFF9 instead of 0xF9.
    BX    R0                    ; No return - Handler mode will be exited. Stack will be popped and execution will continue in reset handler initializing other application.
    ALIGN
}
#elif defined ( __GNUC__ )
static inline void bootloader_util_reset (uint32_t start_addr) __attribute__ ((optimize("-fomit-frame-pointer")));
static inline void bootloader_util_reset(uint32_t start_addr)
{
    __asm volatile(
        "ldr   r0, [%0]\t\n"            // Get App initial MSP for bootloader.
        "msr   msp, r0\t\n"             // Set the main stack pointer to the applications MSP.
        "ldr   r0, [%0, #0x04]\t\n"     // Load Reset handler into R0.

        "movs  r4, #0xFF\t\n"           // Move ones to R4.
        "sxtb  r4, r4\t\n"              // Sign extend R4 to obtain 0xFFFFFFFF instead of 0xFF.

        "mrs   r5, IPSR\t\n"            // Load IPSR to R5 to check for handler or thread mode.
        "cmp   r5, #0x00\t\n"           // Compare, if 0 then we are in thread mode and can continue to reset handler of bootloader.
        "bne   isr_abort\t\n"           // If not zero we need to exit current ISR and jump to reset handler of bootloader.

        "mov   lr, r4\t\n"              // Clear the link register and set to ones to ensure no return.
        "bx    r0\t\n"                  // Branch to reset handler of bootloader.

        "isr_abort:  \t\n"

        "mov   r5, r4\t\n"              // Fill with ones before jumping to reset handling. Will be popped as LR when exiting ISR. Ensures no return to application.
        "mov   r6, r0\t\n"              // Move address of reset handler to R6. Will be popped as PC when exiting ISR. Ensures the reset handler will be executed when exist ISR.
        "movs  r7, #0x21\t\n"           // Move MSB reset value of xPSR to R7. Will be popped as xPSR when exiting ISR. xPSR is 0x21000000 thus MSB is 0x21.
        "rev   r7, r7\t\n"              // Reverse byte order to put 0x21 as MSB.
        "push  {r4-r7}\t\n"             // Push everything to new stack to allow interrupt handler to fetch it on exiting the ISR.

        "movs  r4, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R0 when exiting ISR (Cleaning up of the registers).
        "movs  r5, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R1 when exiting ISR (Cleaning up of the registers).
        "movs  r6, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R2 when exiting ISR (Cleaning up of the registers).
        "movs  r7, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R3 when exiting ISR (Cleaning up of the registers).
        "push  {r4-r7}\t\n"             // Push zeros (R4-R7) to stack to prepare for exiting the interrupt routine.

        "movs  r0, #0xF9\t\n"           // Move the execution return command into register, 0xFFFFFFF9.
        "sxtb  r0, r0\t\n"              // Sign extend R0 to obtain 0xFFFFFFF9 instead of 0xF9.
        "bx    r0\t\n"                  // No return - Handler mode will be exited. Stack will be popped and execution will continue in reset handler initializing other application.
        ".align\t\n"
        :: "r" (start_addr)             // Argument list for the gcc assembly. start_addr is %0.
        :  "r0", "r4", "r5", "r6", "r7" // List of register maintained manually.
    );
}
#elif defined ( __ICCARM__ )
static inline void bootloader_util_reset(uint32_t start_addr)
{
    asm("ldr   r5, [%0]\n"                    // Get App initial MSP for bootloader.
        "msr   msp, r5\n"                     // Set the main stack pointer to the applications MSP.
        "ldr   r0, [%0, #0x04]\n"             // Load Reset handler into R0.

        "movs  r4, #0x00\n"                   // Load zero into R4.
        "mvns  r4, r4\n"                      // Invert R4 to ensure it contain ones.

        "mrs   r5, IPSR\n"                    // Load IPSR to R5 to check for handler or thread mode 
        "cmp   r5, #0x00\n"                   // Compare, if 0 then we are in thread mode and can continue to reset handler of bootloader.
        "bne.n isr_abort\n"                   // If not zero we need to exit current ISR and jump to reset handler of bootloader.

        "mov   lr, r4\n"                      // Clear the link register and set to ones to ensure no return.
        "bx    r0\n"                          // Branch to reset handler of bootloader.

        "isr_abort: \n"
                                              // R4 contains ones from line above. We be popped as R12 when exiting ISR (Cleaning up the registers).
        "mov   r5, r4\n"                      // Fill with ones before jumping to reset handling. Will be popped as LR when exiting ISR. Ensures no return to application.
        "mov   r6, r0\n"                      // Move address of reset handler to R6. Will be popped as PC when exiting ISR. Ensures the reset handler will be executed when exist ISR.
        "movs  r7, #0x21\n"                   // Move MSB reset value of xPSR to R7. Will be popped as xPSR when exiting ISR. xPSR is 0x21000000 thus MSB is 0x21.
        "rev   r7, r7\n"                      // Reverse byte order to put 0x21 as MSB.
        "push  {r4-r7}\n"                     // Push everything to new stack to allow interrupt handler to fetch it on exiting the ISR.

        "movs  r4, #0x00\n"                   // Fill with zeros before jumping to reset handling. We be popped as R0 when exiting ISR (Cleaning up of the registers).
        "movs  r5, #0x00\n"                   // Fill with zeros before jumping to reset handling. We be popped as R1 when exiting ISR (Cleaning up of the registers).
        "movs  r6, #0x00\n"                   // Fill with zeros before jumping to reset handling. We be popped as R2 when exiting ISR (Cleaning up of the registers).
        "movs  r7, #0x00\n"                   // Fill with zeros before jumping to reset handling. We be popped as R3 when exiting ISR (Cleaning up of the registers).
        "push  {r4-r7}\n"                     // Push zeros (R4-R7) to stack to prepare for exiting the interrupt routine.

        "movs  r0, #0x06\n"                   // Load 0x06 into R6 to prepare for exec return command.
        "mvns  r0, r0\n"                      // Invert 0x06 to obtain EXEC_RETURN, 0xFFFFFFF9.
        "bx    r0\n"                          // No return - Handler mode will be exited. Stack will be popped and execution will continue in reset handler initializing other application.
        :: "r" (start_addr)                   // Argument list for the IAR assembly. start_addr is %0.
        :  "r0", "r4", "r5", "r6", "r7");     // List of register maintained manually.
}
#else
#error Compiler not supported.
#endif


void bootloader_util_app_start(uint32_t start_addr)
{

	uint32_t const gpregret = NRF_POWER->GPREGRET;

	bool _sd_inited = (gpregret == DFU_MAGIC_OTA_APPJUM);

	if (!_sd_inited)
	{
		// sd_mbr_command_t com = {.command = SD_MBR_COMMAND_INIT_SD};
  		// sd_mbr_command(&com);
	}

	// sd_softdevice_disable();

	// Disable all interrupts
  NVIC->ICER[0]=0xFFFFFFFF;
  NVIC->ICPR[0]=0xFFFFFFFF;
#if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
  NVIC->ICER[1]=0xFFFFFFFF;
  NVIC->ICPR[1]=0xFFFFFFFF;
#endif

	uint32_t app_addr;
	app_addr = SD_SIZE_GET(MBR_SIZE);

	*(uint32_t *)(0x20000000) = app_addr;

    bootloader_util_reset(app_addr);
}




typedef void (*app_entry_t)(void);

void jump_to_application(uint32_t app_addr)
{


    // Disable interrupts
    __disable_irq();

	SCB->VTOR = app_addr;

	uint32_t reset_handler = *(volatile uint32_t *)(app_addr );

    // Set Main Stack Pointer to the application's stack
    __set_MSP(*(volatile uint32_t *)app_addr);

    // Jump to application's reset handler
    ((void (*)(void))reset_handler)();
}



void cleanup_arm_nvic(void) {
	/* Allow any pending interrupts to be recognized */
	__ISB();
	__disable_irq();

	/* Disable NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
	}
	/* Clear pending NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICPR); i++) {
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
}


static void do_boot()
{
    struct arm_vector_table *vt;


    /* The beginning of the image is the ARM vector table, containing
     * the initial stack pointer address and the reset vector
     * consecutively. Manually set the stack pointer and jump into the
     * reset vector
     */
    int rc;
    const struct flash_area *fap;
    static uint32_t dst[2];



//     rc = flash_area_open(FIXED_PARTITION_ID(MCUBOOT_PARTITION), &fap);
//     assert(rc == 0);

//     rc = flash_area_read(fap, 0 , dst, sizeof(dst));

//     assert(rc == 0);
// #ifndef CONFIG_ASSERT
//     /* Enter a lock up as asserts are disabled */
//     if (rc != 0) {
//         while (1);
//     }
// #endif

//     flash_area_close(fap);

    vt = (struct arm_vector_table *)dst;

    // vt->msp=0x2000AD00;
    // vt->reset=0x0002A5C9;

    vt->msp=0x2000AD00;
    vt->reset=0x0002A5C9;

    



    if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
        sys_clock_disable();
    }

#ifdef CONFIG_USB_DEVICE_STACK
    /* Disable the USB to prevent it from firing interrupts */
    usb_disable();
#endif

#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
    cleanup_arm_nvic(); /* cleanup NVIC registers */

    #if defined(CONFIG_BOOT_DISABLE_CACHES)
        /* Flush and disable instruction/data caches before chain-loading the application */
        (void)sys_cache_instr_flush_all();
        (void)sys_cache_data_flush_all();
        sys_cache_instr_disable();
        sys_cache_data_disable();
    #endif

    #if CONFIG_CPU_HAS_ARM_MPU || CONFIG_CPU_HAS_NXP_MPU
        z_arm_clear_arm_mpu_config();
    #endif

    #if defined(CONFIG_BUILTIN_STACK_GUARD) && \
        defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
        /* Reset limit registers to avoid inflicting stack overflow on image
        * being booted.
        */
        __set_PSPLIM(0);
        __set_MSPLIM(0);
    #endif
#else
    irq_lock();
#endif /* CONFIG_MCUBOOT_CLEANUP_ARM_CORE */



#ifdef CONFIG_BOOT_INTR_VEC_RELOC
    #if defined(CONFIG_SW_VECTOR_RELAY)
        _vector_table_pointer = vt;
        #ifdef CONFIG_CPU_CORTEX_M_HAS_VTOR
            SCB->VTOR = (uint32_t)__vector_relay_table;
        #endif
    #elif defined(CONFIG_CPU_CORTEX_M_HAS_VTOR)
        SCB->VTOR = (uint32_t)vt;
    #endif /* CONFIG_SW_VECTOR_RELAY */
#else /* CONFIG_BOOT_INTR_VEC_RELOC */

    #if defined(CONFIG_CPU_CORTEX_M_HAS_VTOR) && defined(CONFIG_SW_VECTOR_RELAY)
        _vector_table_pointer = _vector_start;
        SCB->VTOR = (uint32_t)__vector_relay_table;
    #endif
#endif /* CONFIG_BOOT_INTR_VEC_RELOC */

    __set_MSP(vt->msp);
#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
    __set_CONTROL(0x00); /* application will configures core on its own */
    __ISB();
#endif
    ((void (*)(void))vt->reset)();
}


#define DFU_DBL_RESET_MEM               0x20007F7C
uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

int main(void)
{
	int ret;


    ret = usb_enable(NULL);

    k_msleep(3000);

	// if(NRF_POWER->GPREGRET == 0x57)
	// {
	// 	
	// 	if (ret != 0) {
	// 		LOG_ERR("Failed to enable USB");
	// 		return 0;
	// 	}
	// 	LOG_INF("This device supports USB DFU class.\n");
	// }
	//else

		// NRF_POWER->GPREGRET = 0x57;
		// bootloader_util_app_start(NEW_APP_ADDRESS);
    while(1)
    {
        // jump_to_application(NEW_APP_ADDRESS);
        do_boot();
        // bootloader_util_app_start(NEW_APP_ADDRESS);
        k_msleep(1000);
    }
		
		// 



	return 0;
}
