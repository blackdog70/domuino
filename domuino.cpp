/*
 * Domuino.cpp
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#include "domuino.h"

timeout dht_timeout;
timeout pir_timeout;
timeout lux_timeout;
timeout ems_timeout;

void setup()
{
	/* INIT BUS */
	Serial.begin(19200);
	pinMode(BUS_ENABLE, OUTPUT);

	/* INIT IO PINS */
	pinMode(DHT_PIN, INPUT);
	pinMode(PIR_IN, INPUT);
	pinMode(LUX_IN, INPUT);
	dht_sensor.setup(DHT_PIN);

	/* INITIAL SENSOR STATE */
	pir_state = LOW;
//
//	/* INIT TIMEOUT */
//	pir_timeout.value = PIR_TIMEOUT;
//	lux_timeout.value = LUX_TIMEOUT;
//	dht_timeout.value = DHT_TIMEOUT;
//
//	/* INIT SENSORS CHANNELS */
//	node.channel(update_pir, &pir_timeout);
//	node.channel(update_lux, &lux_timeout);
//	node.channel(update_dht, &dht_timeout);
}

void loop()
{
	Packet packet;

	if(read(&packet)) {
//		Serial.print("Source=");
//		Serial.println(packet.source);
//		Serial.print("Dest=");
//		Serial.println(packet.dest);
//		Serial.print("Code=");
//		Serial.println(packet.payload.code, HEX);
//		Serial.print("Data=");
//		Serial.write((const char *)packet.payload.data, 10);
		if (packet.dest != NODE_ID) {
			/* If packet is not for this node wait for the same time of
			 * a packet timeout, this will give the time to complete the
			 * current transaction between the strangers nodes
			 */
			flushinputbuffer();
			delay(PACKET_TIMEOUT);
		} else {
			/* Wait 1ms before reply to give time for transmission
			 * direction change
			 */
			delay(10);
			packet.source = NODE_ID;
			packet.dest = 1;
			memset(packet.payload.data, 0, sizeof(packet.payload.data));
			switch(packet.payload.code) {
				case C_CONFIG: {
					switch(packet.payload.data[0]) {
						case C_LUX:
							lux_timeout.value = packet.payload.data[1] * 1000UL;
						case C_PIR:
							pir_timeout.value = packet.payload.data[1] * 1000UL;
						case C_DHT:
							dht_timeout.value = packet.payload.data[1] * 1000UL;
						case C_EMS:
							ems_timeout.value = packet.payload.data[1] * 1000UL;
					}
					break;
				}
				case C_MEM: {
					int free = freeMemory();
					memcpy(packet.payload.data, &free, sizeof(int));
					break;
				}
				case C_LUX: {
					int lux = analogRead(LUX_IN);
					memcpy(packet.payload.data, &lux, sizeof(int));
					break;
				}
				case C_PIR: {
					int pir = digitalRead(PIR_IN);
					memcpy(packet.payload.data, &pir, sizeof(int));
					break;
				}
				case C_DHT: {
					struct Payload_dht {
						uint16_t temperature;
						uint16_t humidity;
					} dht;
					dht.temperature = dht_sensor.getTemperature();
					dht.humidity = dht_sensor.getHumidity();
					memcpy(packet.payload.data, &dht, sizeof(Payload_dht));
					break;
				}
				case C_EMS: {
					struct Payload_ems {
						double value[NUM_EMS];
					} ems;
					for(uint8_t i = 0; i < NUM_EMS; i++) {
						double power = energy[i].calcIrms(1480) * EMON_VOLTAGE;

						if (power == NAN) {
							power = -1.0;
						} else if (power <= 600.0) {
							power *= 0.95;
						}
						ems.value[i] = power;
					}
					memcpy(packet.payload.data, &ems, sizeof(Payload_ems));
					break;
				}
			}
			write(&packet);
		}
	}
}

void start_bootloader()
{
	EEPROM.write(0, 0); // Set start bootloader
	wdt_reset();
	wdt_enable(WDTO_15MS);
	exit (1);  			// loop forever
}

void flushinputbuffer() {
	while(Serial.available() > 0)
		Serial.read();
}

void push(const char dest, Payload *payload) {
	Packet packet;
	unsigned long timeout;

	packet.source = NODE_ID;
	packet.dest = dest;
	memcpy((uint8_t*)&packet.payload, (uint8_t*)payload, sizeof(Payload));
	write(&packet);
	timeout = millis();
	while(!read(&packet) && (millis() - timeout) < PACKET_TIMEOUT);
	if(packet.dest == NODE_ID) {
		/* TODO: da completare con interprete pacchetto */
		flushinputbuffer();
	} else {
		delay(PACKET_TIMEOUT);
	}
}

uint8_t write(Packet* pkt) {
	if(Serial.availableForWrite() < (int)sizeof(pkt->payload.data)) {
		Serial.print("Full buffer");
		return 0;
	}

	/*
	 * There are 2 delay of 10 us approx a char at 19200
	 * to wait for bus enable and disable to be sure for
	 * a complete transmission
	 */
	uint8_t size = sizeof(Packet);
	uint16_t chksum = ModRTU_CRC((char*)pkt, size);

	digitalWrite(BUS_ENABLE, HIGH);          			// 485 write mode
	delayMicroseconds(50);
	Serial.write(0x08);
	Serial.write(0x70);
	Serial.write(size);
	Serial.write((unsigned char *)pkt, size);
	Serial.write((char*)&chksum, 2);
//	while (!(UCSR0A & _BV(TXC0)));						// wait for complete transmission
	Serial.flush();										// wait for complete transmission
	delayMicroseconds(50);
	digitalWrite(BUS_ENABLE, LOW);						// 485 read mode
	return 1;
}

uint8_t read(Packet* packet) {
	int ch;
	char buffer[MAX_BUFFER_SIZE];

	// Wait for Header
	do {
		if (Serial.available() < 3)
			return 0;
		ch = Serial.read();
	} while(ch != 0x08);
	if (Serial.read() != 0x70)
		return 0;

	uint16_t size = Serial.read();

//	Serial.print("Wait packet");
	// Wait for packet
	Serial.setTimeout(PACKET_TIMEOUT);
	if ((size > sizeof(Packet)) || (Serial.readBytes(buffer, size) < size)) {
		flushinputbuffer();
		return 0;
	}

//	Serial.print("Wait CRC");
	// Wait for CRC
	uint16_t chksum;
	if (Serial.readBytes((char *)&chksum, 2) < 2) {
		flushinputbuffer();
		return 0;
	}
	// Check CRC
	if (chksum!=ModRTU_CRC(buffer, size)) {
//		Serial.print("CRC Error=");
//		Serial.print(chksum, HEX);
		flushinputbuffer();
		return 0;
	}

	memcpy(packet, buffer, size);
	return 1;
}
