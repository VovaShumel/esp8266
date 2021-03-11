/******************************************************************************
 * PV` FileName: user_main.c
 *******************************************************************************/

#include "user_config.h"
#include "bios.h"
#include "sdk/add_func.h"
#include "hw/esp8266.h"
#include "user_interface.h"
#include "tcp_srv_conn.h"
#include "flash_eep.h"
#include "wifi.h"
#include "hw/spi_register.h"
#include "sdk/rom2ram.h"
#include "web_iohw.h"
#include "tcp2uart.h"
#include "webfs.h"

#ifdef USE_WEB
#include "web_srv.h"
#endif

#ifdef USE_NETBIOS
#include "netbios.h"
#endif

#ifdef USE_SNTP
#include "sntp.h"
#endif

//#ifdef USE_MODBUS
//#include "modbustcp.h"
#ifdef USE_RS485DRV
#include "driver/rs485drv.h"
#include "mdbtab.h"
#endif
//#endif

#ifdef USE_WEB
extern void web_fini(const uint8 * fname);
static const uint8 sysinifname[] ICACHE_RODATA_ATTR = "protect/init.ini";
#endif

#ifdef SHUMEL
os_timer_t fito_timer;

void on_fito_timer(void);

void run_fito_timer(uint32 tsec)
{
	ets_timer_disarm(&fito_timer);
	ets_timer_setfn(&fito_timer, (os_timer_func_t *)on_fito_timer, NULL);
	ets_timer_arm_new(&fito_timer, tsec*1000, 0, 1); // На секунды
}

void on_fito_timer(void)
{
//	if (GPIO_OUT & (1 << OPTO_OUT))
//		GPIO_OUT_W1TC = 1 << OPTO_OUT;
//	else
//		GPIO_OUT_W1TS = 1 << OPTO_OUT;

	u32 fitotime = read_user_const(3);

	u8 fitoStartMin = fitotime & 0xFF;
	u8 fitoStartHour = (fitotime >> 8) & 0xFF;
	u8 fitoEndMin = (fitotime >> 16) & 0xFF;
	u8 fitoEndHour = (fitotime >> 24) & 0xFF;

	time_t curTime = (get_sntp_time() + 3 * 3600) / 60; // now it is minutes
	u8 curMin = curTime % 60;
	curTime /= 60; // now it is hours
	u8 curHour = curTime % 24;

	if ((fitoStartHour <= curHour) && (curHour < fitoEndHour))
		GPIO_OUT_W1TS = 1 << OPTO_OUT;
	else
		GPIO_OUT_W1TC = 1 << OPTO_OUT;

	os_printf("Fito start time: %d:%d\n", fitoStartHour, fitoStartMin);
	os_printf("Fito end time: %d:%d\n", fitoEndHour, fitoEndMin);

	os_printf("Current time: %d:%d\n", curHour, curMin);

	run_fito_timer(2);
}
#endif

void ICACHE_FLASH_ATTR init_done_cb(void)
{
#if (DEBUGSOO > 0)
    os_printf("\nSDK Init - Ok\nCurrent 'heap' size: %d bytes\n", system_get_free_heap_size());
#endif
#ifdef USE_WEB
	web_fini(sysinifname);
#endif
	switch(system_get_rst_info()->reason) {
	case REASON_SOFT_RESTART:
	case REASON_DEEP_SLEEP_AWAKE:
		break;
	default:
		New_WiFi_config(WIFI_MASK_ALL);
		break;
	}
#ifdef USE_RS485DRV
	rs485_drv_start();
	init_mdbtab();
#endif

#ifdef SHUMEL
	GPIO_ENABLE_W1TS = 1 << OPTO_OUT;
	on_fito_timer();
#endif
}

#ifdef SHUMEL
	uint32 fitoTime;
#endif

extern uint32 _lit4_start[]; // addr start BSS in IRAM
extern uint32 _lit4_end[]; // addr end BSS in IRAM
/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void ICACHE_FLASH_ATTR user_init(void) {
	sys_read_cfg();
	if(!syscfg.cfg.b.debug_print_enable) system_set_os_print(0);
	GPIO0_MUX = VAL_MUX_GPIO0_SDK_DEF;
	GPIO4_MUX = VAL_MUX_GPIO4_SDK_DEF;
	GPIO5_MUX = VAL_MUX_GPIO5_SDK_DEF;
	GPIO12_MUX = VAL_MUX_GPIO12_SDK_DEF;
	GPIO13_MUX = VAL_MUX_GPIO13_SDK_DEF;
	GPIO14_MUX = VAL_MUX_GPIO14_SDK_DEF;
	GPIO15_MUX = VAL_MUX_GPIO15_SDK_DEF;
#ifndef SHUMEL // чтобы можно было выводить отладочную инфу в UART0 REFACT сделать нормально
	uarts_init();
#endif
	system_timer_reinit();
#if (DEBUGSOO > 0 && defined(USE_WEB))
	os_printf("\nSimple WEB version: " WEB_SVERSION "\n");
#endif
	if(syscfg.cfg.b.pin_clear_cfg_enable) test_pin_clr_wifi_config();
	set_cpu_clk(); // select cpu frequency 80 or 160 MHz
#ifdef USE_GDBSTUB
extern void gdbstub_init(void);
	gdbstub_init();
#endif
#if DEBUGSOO > 0
	if(eraminfo.size > 0) os_printf("Found free IRAM: base: %p, size: %d bytes\n", eraminfo.base,  eraminfo.size);
	os_printf("System memory:\n");
    system_print_meminfo();
    os_printf("bssi  : 0x%x ~ 0x%x, len: %d\n", &_lit4_start, &_lit4_end, (uint32)(&_lit4_end) - (uint32)(&_lit4_start));
    os_printf("free  : 0x%x ~ 0x%x, len: %d\n", (uint32)(&_lit4_end), (uint32)(eraminfo.base) + eraminfo.size, (uint32)(eraminfo.base) + eraminfo.size - (uint32)(&_lit4_end));

    os_printf("Start 'heap' size: %d bytes\n", system_get_free_heap_size());
#endif
#if DEBUGSOO > 0
	os_printf("Set CPU CLK: %u MHz\n", ets_get_cpu_frequency());
#endif
	Setup_WiFi();
	WEBFSInit(); // файловая система

	#ifdef SHUMEL
		fitoTime = read_user_const(3);
	#endif

	system_deep_sleep_set_option(0);
	system_init_done_cb(init_done_cb);
}

#if DEF_SDK_VERSION >= 2000
/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    return 256 - 5; // всегда 'песочница' для SDK в первых 512k Flash
}
#endif // DEF_SDK_VERSION >= 2000
