#ifndef _user_config_h_
#define _user_config_h_

#include "sdk/sdk_config.h"

#define SYS_VERSION "0.6.4"

#define SHUMEL
#define OPTO_OUT 5

#define WEB_DEFAULT_SOFTAP_IP	DEFAULT_SOFTAP_IP // ip 192.168.4.1
#define WEB_DEFAULT_SOFTAP_MASK	DEFAULT_SOFTAP_MASK // mask 255.255.255.0
#define WEB_DEFAULT_SOFTAP_GW	DEFAULT_SOFTAP_IP // gw 192.168.4.1
#define WEB_DEFAULT_STATION_IP    0x3201A8C0 // ip 192.168.1.50
#define WEB_DEFAULT_STATION_MASK  0x00FFFFFF // mask 255.255.255.0
#define WEB_DEFAULT_STATION_GW    0x0101A8C0 // gw 192.168.1.1

#define PROGECT_NUMBER 0
// 0 -> проект "TCP2UART"
// 1 -> проект MODBUS-"RS485"
// 2 -> пустой web
// 3 — Таймер фитолампы
// 4 — опрос платы с датчиками температуры, влажности, CO2
// 5 - тест программирования STM8
// 6 - тест программирования PIC16F18324
// 7 - тест работы с многоцветными светодиодами
// 8 - весы улья
// 9 - туалетное занято
#if PROGECT_NUMBER == 0 // TCP2UART
	#define USE_WEB			80 	// включить в трансляцию порт Web, если =0 - по умолчанию выключен
	#define WEBSOCKET_ENA  		// включить WEBSOCKET
	#define USE_TCP2UART 12345 	// включить в трансляцию драйвер TCP2UART, номер порта по умолчанию (=0 - отключен)
	#define USE_UART_AJAX 		// включить в трансляцию ~uart_data~ (для чудаков :) )
	#define USE_CPU_SPEED  160 	// установить частоту CPU по умолчанию 80 или 160 MHz
	#define USE_NETBIOS		 1 	// включить в трансляцию драйвер NETBIOS, если =0 - по умолчанию выключен.
	#define USE_SNTP		 1 	// включить в трансляцию драйвер SNTP, если =0 - по умолчанию выключен, = 1 - по умолчанию включен.
	#define USE_MODBUS	   502 	// включить в трансляцию Modbus TCP, если =0 - по умолчанию выключен
	#define MDB_ID_ESP 		50  // номер устройства ESP на шине modbus
	#define USE_CAPTDNS		 0	// включить в трансляцию NDS отвечающий на всё запросы клиента при соединении к AP модуля
								// указанием на данный WebHttp (http://aesp8266/), если =0 - по умолчанию выключен
	#if USE_MAX_IRAM == 48
		#define USE_OVERLAY 16384 // включить в трансляцию возможность работы с оверлеями (максимальный размер кода оверлея)
	#else
		#if DEF_SDK_VERSION >=2000
			#define USE_OVERLAY 5850 // включить в трансляцию возможность работы с оверлеями (максимальный размер кода оверлея)
		#else
			#define USE_OVERLAY 7424 // включить в трансляцию возможность работы с оверлеями (максимальный размер кода оверлея)
		#endif
	#endif
#elif PROGECT_NUMBER == 1 // MODBUS-RS-485
	#define USE_WEB			80
	#define WEBSOCKET_ENA
	#define USE_CPU_SPEED  160
	#define USE_NETBIOS		 1
	#define USE_SNTP		 1
	#ifndef USE_TCP2UART
		#define USE_RS485DRV		// использовать RS-485 драйвер
		#define MDB_RS485_MASTER 	// Modbus RTU RS-485 master & slave
	#endif
	#define USE_MODBUS	   502
	#define MDB_ID_ESP 		50
	#define USE_CAPTDNS		 0

	#if USE_MAX_IRAM == 48
		#define USE_OVERLAY 16384
	#else
		#if DEF_SDK_VERSION >=2000
			#define USE_OVERLAY 5850
		#else
			#define USE_OVERLAY 7424
		#endif
	#endif
#elif PROGECT_NUMBER == 2 // WEB+WEBSOCEK
	#define USE_WEB		80
//#define WEBSOCKET_ENA
//#define USE_NETBIOS	1
	#define USE_SNTP	1
#elif PROGECT_NUMBER == 4 // опрос платы с датчиками температуры, влажности, CO2

#elif PROGECT_NUMBER == 5 // тест программирования STM8

#elif PROGECT_NUMBER == 6 // тест программирования PIC16F18324

#elif PROGECT_NUMBER == 7 // тест работы с многоцветными светодиодами

#elif PROGECT_NUMBER == 8 // весы улья

#elif PROGECT_NUMBER == 9 // туалетное занято

#endif // PROGECT_NAME

#if DEBUGSOO > 0
#if !(defined(USE_RS485DRV) || defined(USE_TCP2UART))
// #define USE_GDBSTUB // UDK пока не поддерживает GDB
#endif
#endif

#endif // _user_config_h_
