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

#include <zephyr/storage/flash_map.h>


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
#define DFU_MAGIC_MCUMGR_RESET          0x48
#define DFU_MAGIC_SKIP                  0x6d

#define DFU_DBL_RESET_MAGIC_1            0x5A1AD5      // SALADS
#define DFU_DBL_RESET_MAGIC_2            0x5A5A1A      // SASALA
#define DFU_DBL_RESET_APP               0x4ee5677e
#define DFU_DBL_RESET_DELAY             300
#define DFU_DBL_RESET_MEM               0x20007F7C


uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

uint8_t do_boot(void);
uint8_t led_blink_status=0;
struct k_thread my_thread_data;
K_THREAD_STACK_DEFINE(my_stack_area, 1024);

#if defined(CONFIG_BOARD_NAYA_DONGLE)  || defined(CONFIG_BOARD_XIAO_BLE)
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
void led_blink(void *, void *, void *)
{
    uint16_t delay=1000;
    uint32_t start=k_uptime_get_32();
    uint32_t st=0;
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_blue,GPIO_OUTPUT_INACTIVE);   
    
    bool value=0;
    while (1) {
        gpio_pin_toggle_dt(&led_blue);
        gpio_pin_toggle_dt(&led_red);
        // gpio_pin_set(led_blue.port,led_blue.pin,value);
        // gpio_pin_set(led_red.port,led_red.pin,!value);
        value=!value;
        if(led_blink_status == 0) delay = 700;
        else delay =100;
        
        k_msleep(delay);

        if(k_uptime_get_32() - start > 10000) do_boot();
    }
}
#else
void led_blink(void *, void *, void *)
{

    uint32_t start=k_uptime_get_32();

    bool value=0;
    while (1) {
        k_msleep(1000);
        if(k_uptime_get_32() - start > 10000) do_boot();
    }
}

#endif

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




uint8_t do_boot(void)
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
    
    rc = flash_area_open(FIXED_PARTITION_ID(MCUBOOT_PARTITION), &fap);
    assert(rc == 0);

    rc = flash_area_read(fap, 0 , dst, sizeof(dst));

    assert(rc == 0);
#ifndef CONFIG_ASSERT
    /* Enter a lock up as asserts are disabled */
    if (rc != 0) {
        while (1);
    }
#endif

    flash_area_close(fap);


    if( dst[0] == 0xFFFFFFFF) return 0;

    vt = (struct arm_vector_table *)dst;

#if defined(CONFIG_BOARD_NAYA_DONGLE)  || defined(CONFIG_BOARD_XIAO_BLE)
    gpio_pin_configure_dt(&led_blue, GPIO_DISCONNECTED);
    gpio_pin_configure_dt(&led_red, GPIO_DISCONNECTED);
#endif

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

    return 0;
}


int main(void)
{

    bool const reason_reset_pin = (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk) ? true : false;

    if( NRF_POWER->GPREGRET== DFU_MAGIC_UF2_RESET)
    {	
        NRF_POWER->GPREGRET=0;	                
    }
    else
    {
        if (reason_reset_pin) 
        {
            if(* dbl_reset_mem == DFU_DBL_RESET_MAGIC_2)
            {	
                (*dbl_reset_mem) = 0;
                //Magic value for MCUBoot to detect entering recovery mode
                NRF_POWER->GPREGRET=DFU_MAGIC_MCUMGR_RESET;
                //Boot to MCUboot
                do_boot();               	       
            }
            else if(* dbl_reset_mem == DFU_DBL_RESET_MAGIC_1)
            {
                // Register our second reset for tripple reset detection
                (*dbl_reset_mem) = DFU_DBL_RESET_MAGIC_2;
                
                // if RST is pressed during this delay (tripple reset)--> if will enter MCUMGR
                k_msleep(DFU_DBL_RESET_DELAY);
                
                (*dbl_reset_mem) = 0;
            }
            else
            {
                // Register our first reset for double reset detection
                (*dbl_reset_mem) = DFU_DBL_RESET_MAGIC_1;
                
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

    usb_enable(NULL);


    k_tid_t my_tid =k_thread_create(&my_thread_data, my_stack_area,
                                 K_THREAD_STACK_SIZEOF(my_stack_area),
                                 led_blink,
                                 NULL, NULL, NULL,
                                 91, 0, K_NO_WAIT);


	return 0;
}
