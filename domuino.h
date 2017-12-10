/*
 * Domuino.h
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#ifndef DOMUINO_H_
#define DOMUINO_H_

#include "Arduino.h"
#include <crc16.h>
#include <FreeMemory.h>
#include <EmonLib.h>
#include <DHT.h>
#include <SSD1306Ascii.h>
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
#define BAUDRATE 38400
#define MAX_DATA_SIZE 12

struct Payload {
    uint8_t code;
    uint8_t data[MAX_DATA_SIZE];
};

struct Packet {
	uint16_t source;
	uint16_t dest;
    Payload payload;
};

const unsigned char header[3] = {0x08, 0x70, sizeof(Packet)};

#define MAX_QUEUE_SIZE 6

struct Item {
	uint8_t retry;
	Packet packet;
};

Item queue[MAX_QUEUE_SIZE];

#define BUS_ENABLE 2
#define PACKET_TIMEOUT 100UL 	// milliseconds
#define NUM_PACKET 1
#define MAX_BUFFER_SIZE (NUM_PACKET * (sizeof(Packet) + 5)) // 5 = 2 bytes header, 1 byte size, 2 bytes CRC
#define MAX_RETRY 3

/*
 * Commands settings
 */
// QUERY: COMMAND > COMMAND_PATTERN
// ANSWER: COMMAND <= COMMAND_PATTERN
#define COMMAND_PATTERN  0x80
// SYSTEM
#define C_START 		0x80
#define C_PING 			0x81
#define C_RESET 		0x82
#define C_STANDBY 		0x83
#define C_RUN	 		0x84
#define C_CONFIG 		0x88
#define C_HUB 			0x89
#define C_MEM 			0x90
#define C_LCDCLEAR		0x91
#define C_LCDPRINT		0x92
#define C_LCDWRITE		0x93
#define C_HBT 			0x9f
// DEVICE
#define C_DHT 			0xA0
#define C_EMS 			0xA1
#define C_BINARY_OUT 	0xA2
#define C_SWITCH 		0xA3
#define C_LIGHT 		0xA4
#define C_PIR 			0xA5
#define C_LUX 			0xA6

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

#define DHT_TIMEOUT 2000UL
#define PIR_TIMEOUT 5000UL
#define LUX_TIMEOUT 10000UL
#define EMS_TIMEOUT 10000UL
#define HB_TIMING 10000UL  // heartbeat

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
#define NUMTOUCH 2
#define TOUCH_PIN 3 // Base pin, others will be Base+1, Base+2 and so on

uint16_t hub_node;

#define RUN 0
#define STANDBY 1
#define PROGRAM 2

uint8_t state;
uint16_t get_id();
void showsplash();
uint8_t refresh_sensor(uint8_t code);
uint8_t prepare_packet(uint8_t code, Packet *packet);
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
