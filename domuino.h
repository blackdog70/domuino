/*
 * Domuino.h
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#ifndef DOMUINO_H_
#define DOMUINO_H_

#include "Arduino.h"
//#include "settings.h"
#include <avr/wdt.h>
#include "crc16.h"
#include "EEPROM.h"
#include "FreeMemory.h"
#include "EmonLib.h"
//#include <Domuino.h>
#include <DHT.h>

/*
 * Communication settings
 */

#define MAX_DATA_SIZE 10

struct Payload {
    uint8_t code;
    uint8_t data[MAX_DATA_SIZE];
};

struct Packet {
	uint8_t source;
	uint8_t dest;
    Payload payload;
};

#define NODE_ID 2
#define BUS_ENABLE 2
#define PACKET_TIMEOUT 100UL 	// milliseconds
#define NUM_PACKET 1
#define MAX_BUFFER_SIZE (NUM_PACKET * (sizeof(Packet) + 5)) // 5 = 2 bytes header, 1 byte size, 2 bytes CRC

/*
 * Commands settings
 */
// QUERY: COMMAND > COMMAND_PATTERN
// ANSWER: COMMAND <= COMMAND_PATTERN
#define COMMAND_PATTERN  0x80
// SYSTEM
#define C_START 		0x80
#define C_PING 		0x81
#define C_RESET 		0x82
#define C_CONFIG 		0x88
#define C_HUB 		0x89
#define C_MEM 		0x90
#define C_HBT 		0x9f
// DEVICE
#define C_DHT 		0xA0
#define C_EMS 		0xA1
#define C_BINARY_OUT 	0xA2
#define C_SWITCH 		0xA3
#define C_LIGHT 		0xA4
#define C_PIR 		0xA5
#define C_LUX 		0xA6

/*
 * IO settings
 */
#define DHT_PIN 9
#define PIR_IN 10
#define LUX_IN A1

/*
 * Timing settings in milliseconds
 */
struct timeout {
	unsigned long timer;
	unsigned long value;
	timeout():timer(millis()), value(0) {};
};

#define DHT_TIMEOUT 2000UL
#define PIR_TIMEOUT 5000UL
#define LUX_TIMEOUT 10000UL
#define EMS_TIMEOUT 10000UL
#define HB_TIMING 10000UL  // heartbeat

/*
 * Emon settings
 */
#define EMON_CURRENT 20.73
#define EMON_VOLTAGE 229.0
#define NUM_EMS 2

EnergyMonitor energy[NUM_EMS];
DHT dht_sensor;

int pir_state;

void start_bootloader();
void push(const char dest, Payload *payload);
void flushinputbuffer();
uint8_t read(Packet* packet);
uint8_t write(Packet* pkt);

#endif /* DOMUINO_H_ */
