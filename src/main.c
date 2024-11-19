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
#define NEW_APP_ADDRESS 0x27000  

#define CONFIG_MCUBOOT_CLEANUP_ARM_CORE     1
#define CONFIG_CPU_HAS_ARM_MPU              1
#define CONFIG_MCUBOOT_CLEANUP_ARM_CORE     1



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
uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);




struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

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
    // int rc;
    // const struct flash_area *fap;
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

    vt->msp=0x2000B040;
    vt->reset=0x000295CD;

    



    if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
        sys_clock_disable();
    }

#ifdef CONFIG_USB_DEVICE_STACK
    /* Disable the USB to prevent it from firing interrupts */
    usb_disable();
#endif

#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
    // cleanup_arm_nvic(); /* cleanup NVIC registers */
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



int main(void)
{

    bool const reason_reset_pin = (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk) ? true : false;


    if( NRF_POWER->GPREGRET== DFU_MAGIC_UF2_RESET)
    {	
        NRF_POWER->GPREGRET=0;	
        usb_enable(NULL);
        while(1)
        {

        }
        
    }
    else
    {
        if (reason_reset_pin) 
        {
            if(* dbl_reset_mem == DFU_DBL_RESET_MAGIC)
            {	
                (*dbl_reset_mem) = 0;
                usb_enable(NULL);	
                while(1)
                {

                }                
            }
            else
            {
                // Register our first reset for double reset detection
                (*dbl_reset_mem) = DFU_DBL_RESET_MAGIC;
                
                // if RST is pressed during this delay (double reset)--> if will enter dfu
                k_msleep(DFU_DBL_RESET_DELAY);
                
                (*dbl_reset_mem) = 0;
                // printf("Booting to app");
                do_boot();
            }
        }
        else
        {            
            do_boot();
        }
    }
	return 0;
}
