/*
 * Domuino.cpp
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#include "domuino.h"

Timeout timeout[5];

void setup()
{
	/* INIT LCD */
	lcd.begin();
	lcd.clear();

	/* INIT BUS */
	Serial.begin(19200);
	pinMode(BUS_ENABLE, OUTPUT);
	hub_node = 1;	// Set to default hub

	/* INIT IO PINS */
	pinMode(DHT_PIN, INPUT);
	pinMode(PIR_IN, INPUT);
	pinMode(LUX_IN, INPUT);
	dht_sensor.setup(DHT_PIN);

	/* INITIAL SENSOR STATE */
	pir_state = LOW;

//	/* INIT TIMEOUT */
	timeout[0].code = C_HBT;
	timeout[1].code = C_LUX;
	timeout[2].code = C_PIR;
	timeout[3].code = C_DHT;
	timeout[4].code = C_EMS;

//	/* INIT SENSORS CHANNELS */
//	node.channel(update_pir, &pir_timeout);
//	node.channel(update_lux, &lux_timeout);
//	node.channel(update_dht, &dht_timeout);
}

void loop()
{
	Packet packet;

	struct Payload_dht {
					uint16_t temperature;
					uint16_t humidity;
				} dht;
	dht.temperature = dht_sensor.getTemperature();
	dht.humidity = dht_sensor.getHumidity();
	lcd.setCursor(0, 0);
	lcd.setFontSize(FONT_SIZE_SMALL);
	lcd.print("    22/01/17 09:57");
	lcd.setCursor(0, 2);
	lcd.setFontSize(FONT_SIZE_MEDIUM);
	lcd.print("TEMP");
	lcd.setCursor(55, 2);
	lcd.setFontSize(FONT_SIZE_XLARGE);
	lcd.printFloat(dht.temperature / 10.0);
	lcd.setFontSize(FONT_SIZE_MEDIUM);
	lcd.setCursor(115, 3);
	lcd.print('C');
	lcd.setCursor(0, 5);
	lcd.setFontSize(FONT_SIZE_MEDIUM);
	lcd.print("HUM");
	lcd.setCursor(55, 5);
	lcd.setFontSize(FONT_SIZE_XLARGE);
	lcd.printFloat(dht.humidity / 10.0);
	lcd.setCursor(115, 6);
	lcd.setFontSize(FONT_SIZE_MEDIUM);
	lcd.print('%');
	delay(2000);

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
			switch(packet.payload.code) {
				case C_CONFIG: {
					switch(packet.payload.data[0]) {
						case C_HBT:
							timeout[0].value = packet.payload.data[1] * 1000UL;
						case C_LUX:
							timeout[1].value = packet.payload.data[1] * 1000UL;
						case C_PIR:
							timeout[2].value = packet.payload.data[1] * 1000UL;
						case C_DHT:
							timeout[3].value = packet.payload.data[1] * 1000UL;
						case C_EMS:
							timeout[4].value = packet.payload.data[1] * 1000UL;
					}
					break;
				}
				case C_RESET: {
					start_bootloader();
					break;
				}
				default:
					prepare_packet(packet.payload.code, &packet);
			}
			write(&packet);
		}
	} else {
		for (uint8_t i = 0; i < (sizeof(timeout) / sizeof(Timeout)); i++) {
			if ((timeout[i].code) &&
					(timeout[i].value) &&
					((millis() - timeout[i].timer) > timeout[i].value)) {
				if(prepare_packet(timeout[i].code, &packet) &&
						(push(&packet) || (timeout[i].retry++ > MAX_PUSH_RETRY))) {
					timeout[i].timer = millis();
					timeout[i].retry = 0;
				}
			}
		}
	}
}

uint8_t prepare_packet(uint8_t code, Packet *packet) {
	packet->source = NODE_ID;
	packet->dest = hub_node;
	packet->payload.code = code;
	memset(packet->payload.data, 0, sizeof(packet->payload.data));
	switch(code) {
		case C_MEM: {
			int free = freeMemory();
			memcpy(packet->payload.data, &free, sizeof(int));
			break;
		}
		case C_LUX: {
			int lux = analogRead(LUX_IN);
			memcpy(packet->payload.data, &lux, sizeof(int));
			break;
		}
		case C_PIR: {
			int pir = digitalRead(PIR_IN);
			memcpy(packet->payload.data, &pir, sizeof(int));
			break;
		}
		case C_DHT: {
			struct Payload_dht {
				uint16_t temperature;
				uint16_t humidity;
			} dht;
			dht.temperature = dht_sensor.getTemperature();
			dht.humidity = dht_sensor.getHumidity();
			memcpy(packet->payload.data, &dht, sizeof(Payload_dht));
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
			memcpy(packet->payload.data, &ems, sizeof(Payload_ems));
			break;
		}
		default:
			return 0;
	}
	return 1;
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

uint8_t push(Packet *packet) {
	unsigned long timeout;

	write(packet);
	timeout = millis();
	while(!read(packet) && (millis() - timeout) < PACKET_TIMEOUT);
	if(packet->dest == NODE_ID) {
		/* TODO: da completare con interprete pacchetto */
	} else {
		delay(PACKET_TIMEOUT);
		flushinputbuffer();
		return 0;
	}
	return 1;
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
