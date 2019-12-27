/*
 * Domuino.h
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#ifndef DOMUINO_H_
#define DOMUINO_H_

#include "Arduino.h"
#include <avr/wdt.h>
#include <crc16.h>
#include <FreeMemory.h>
#include <EmonLib.h>
#include <DHT.h>
//#include <SSD1306Ascii.h>
//#include <SSD1306AsciiAvrI2c.h>
#include <SSD1306AsciiWire.h>


/*
 * LCD settings
 */
#define I2C_ADDRESS 0x3C
//SSD1306AsciiAvrI2c oled;
SSD1306AsciiWire oled;

const uint8_t* fonts[] = {
		font5x7,
		lcdnums14x24
};

/*
 * Communication settings
 */
#define DATA_MAX_SIZE 12
#define BROADCAST 255
#define HEADER {0x08, 0x70}

struct Payload {
    uint8_t code;
    uint8_t data[DATA_MAX_SIZE] = { 0 };
};

struct Packet {
	uint8_t header[2] = HEADER;
	uint16_t source;
	uint16_t dest;
    Payload payload;
    uint16_t crc;
};

struct Item {
	unsigned long timeout;
	Packet packet;
};

#define PACKET_SIZE sizeof(Packet) - 2
#define QUEUE_MAX_SIZE 6

Item queue[QUEUE_MAX_SIZE];
int8_t queue_idx = -1;

#define NET_BAUDRATE 38400
#define _BYTE_DURATION_ 10000000 / NET_BAUDRATE // 8 bit + xon + xoff = 10 bit - express microseconds

/*

                  _MSG_DURATION_
    	   +-------------------------+
		   |    					 |
0----------+    					 +----------END
           !									!
		   SLOT * _MSG_DURATION_                NET_MAX_NODES * _MSG_DURATION_

*/

#define SLOT_EE 0x197 // NET SLOT
#define ID_EE_ADDRESS  0x198

#define NET_MSG_SIZE (sizeof(Packet) + 5) // 5 = 2 bytes header, 1 byte size, 2 bytes CRC
#define NET_MSG_DURATION (_BYTE_DURATION_ * NET_MSG_SIZE) / 1000 + 1 // Conversion to millis
#define NET_SLOT NET_MSG_DURATION
#define NET_MAX_NODES 32
#define BUS_ENABLE 2
#define MAX_RETRY 3
#define NET_PACKET_TIMEOUT (unsigned long)50UL // (NET_MSG_DURATION) 	// milliseconds
#define PACKET_QUERY 0
#define PACKET_ANSWER 1
#define PACKET_LIFETIME (unsigned long)900	// milliseconds
#define PACKET_DELAY_TRANSACTION (unsigned long)5 //milliseconds

/*
 * Commands settings
 */
// QUERY: COMMAND > COMMAND_PATTERN
// ANSWER: COMMAND <= COMMAND_PATTERN

#define COMMAND_PATTERN  0x80
// SYSTEM
#define C_ACK			0x7e
#define C_START 		0x80
#define C_PING 			0x81
#define C_PROGRAM 		0x82
#define C_STANDBY       0x83
#define C_RUN	 		0x84
#define C_SETID			0x85
#define C_CONFIG 		0x88
#define C_HUB 			0x89
#define C_MEM 			0x90
#define C_LCDCLEAR		0x91
#define C_LCDPRINT		0x92
#define C_LCDWRITE		0x93
#define C_LCDINIT		0x94

// DEVICE
#define C_HBT 			0x9f
#define C_DHT 			0xA0
#define C_EMS 			0xA1
#define C_SWITCH 		0xA3
#define C_BINARY_OUT 	0xA2
#define C_LIGHT 		0xA4
#define C_PIR 			0xA5
#define C_LUX 			0xA6

uint8_t commands[] = {
		C_HBT,
		C_LUX,
		C_PIR,
		C_DHT,
		C_EMS,
		C_SWITCH
};

/*
 * Timing settings in milliseconds
 */
struct Timeout {
	uint8_t code;
	uint8_t retry;
	unsigned long timer;
	unsigned long value;
	Timeout():code(0), retry(0), timer(millis()), value(0) {};
};

// **** ATTENZIONE !! L'ORDINE DI DICHIARAZIONE DEVE ESSERE QUELLO DI COMMANDS ****
#define EE_BASE 0x10
#define HBT_TIMING 10000UL  // heartbeat
#define EE_HBT EE_BASE
#define LUX_TIMEOUT 10000UL
#define EE_LUX EE_BASE+1
#define PIR_TIMEOUT 5000UL
#define EE_PIR EE_BASE+2
#define DHT_TIMEOUT 2000UL
#define EE_DHT EE_BASE+3
#define EMS_TIMEOUT 10000UL
#define EE_EMS EE_BASE+4
#define SWITCH_TIMEOUT 1000UL
#define EE_SWITCH EE_BASE+5

/*
 * EMON settings
 */
#define EMON_CURRENT 20.73
#define EMON_VOLTAGE 229.0
#define NUM_EMS 2
struct Payload_ems {
	double value[NUM_EMS];
} ems_state;
EnergyMonitor energy[NUM_EMS];

/*
 * DHT settings
 */
#define DHT_PIN 13
struct Payload_dht {
	int16_t temperature;
	int16_t humidity;
} dht_state;
DHT dht_sensor;

/*
 * PIR settings
 */
#define PIR_IN 12
int pir_state;

/*
 * LUX settings
 */
#define LUX_IN PIN_A1
int lux_state;

/*
 * TOUCH settings
 */
#define NUMSWITCH 3
#define SWITCH_BASE_PIN 3 // Base pin, others will be Base+1, Base+2 and so on

/*
 * LIGHT settings
 */
#define NUMLIGHT 11
#define LIGHT_BASE_PIN 3 // Base pin, others will be Base+1, Base+2 and so on
uint8_t lightbuff[NUMLIGHT];


uint16_t hub_node;

#define RUN 1
#define PROGRAM 2
#define STANDBY 3

uint8_t state;
uint16_t get_id(); // get address
void set_id(uint16_t); // set address
void showsplash();
uint8_t refresh_sensor(uint8_t code);
uint8_t exec_command(Packet *packet);
void prepare_packet(uint8_t code, Packet *packet, uint8_t mode);
void display_info();
void start_bootloader();
uint8_t enqueue(Item *item);
uint8_t enqueue(Packet *packet);
uint8_t dequeue(Item *queue);
uint8_t push(Packet *packet);
void flushinputbuffer();
uint8_t receive(Packet* packet);
uint8_t send(Packet* pkt);

#endif /* DOMUINO_H_ */
