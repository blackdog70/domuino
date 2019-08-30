/*
 * Domuino.cpp
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#include "domuino.h"
#include "logo.h"

Timeout push_timeout[6];
uint16_t node_id;
char running = 0;

#define NUMTIMEOUT (sizeof(push_timeout) / sizeof(Timeout))

void setup()
{
	/* INIT BUS */
	Serial.begin(BAUDRATE);
	pinMode(BUS_ENABLE, OUTPUT);
	hub_node = 1;	// Set to default hub

	/* INIT LCD */
	//oled.begin(&Adafruit128x64, I2C_ADDRESS, false);
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.set400kHz();

/*  PRIMA DI RIABILITARE LA VISUALIZZAZIONE DEL LOGO
 *  CONSIDERARE CHE E' STATO DISATTIVATO PER EVITARE PROBLEMI
 *  CON LA VISUALIZZAZIONE DELLE STRINGHE ALTE 3 BYTES
 *  I PROBLEMI ERANO DIVERSI AD OGNI PROGRAMMAZIONE A SEGUITO DI MODIFICHE AL CODICE
 *  APPARENTEMENTE INSIGNIFICANTI, ES AGGIUNGENDO O TOGLIENDO UN INCREMENTO DI UNA VARIABILE
 *  E NON SONO RIUSCITO A CAPIRE SE IL PROBLEMA E' DOVUTO AL SOFTWARE O ALLA FLASH DEL
 *  CHIP CHE PER QUALCHE MOTIVO NON SI PROGRAMMA CORRETTAMENTE (NON HO
 *  ESEGUITO IL VERIFY DEL DOWNLOAD), IN EFFETTI RESTANDO AL DI SOTTO DI UNA CERTA
 *  SIZE DEL CODICE IL PROBLEMA NON SI VERIFICA
 */
	oled.clear();
//	showsplash();

	/* INIT IO PINS */
	pinMode(DHT_PIN, INPUT);
	pinMode(PIR_IN, INPUT);
	pinMode(LUX_IN, INPUT);
	dht_sensor.setup(DHT_PIN);
	for(uint8_t i=0; i<NUMSWITCH; i++)
		pinMode(SWITCH_BASE_PIN + i, INPUT);
	for(uint8_t i=0; i<NUMLIGHT; i++) {
		pinMode(LIGHT_BASE_PIN + i, OUTPUT);
		digitalWrite(LIGHT_BASE_PIN + i, LOW);
	}


	/* INIT QUEUE */
	memset(&queue, 0, sizeof(queue));

	/* INITIAL SENSOR STATE */
	pir_state = LOW;

	/* INIT TIMEOUT CODES
	 * TIMEOUTS are in milliseconds.
	 * TIMEOUT is used to push repeatedly in automatic the result of a command without a specific request.
	 * for example if C_DHT timeout will be 10000, DOMUINO will push the values of temperature and humidity every 10 seconds.
	 */
	memset(&push_timeout, 0, sizeof(push_timeout));

	push_timeout[0].code = C_HBT;
	push_timeout[0].value = eeprom_read_byte((uint8_t*)EE_HBT) * 1000UL;
	push_timeout[1].code = C_LUX;
	push_timeout[1].value = eeprom_read_byte((uint8_t*)EE_LUX) * 1000UL;
	push_timeout[2].code = C_PIR;
	push_timeout[2].value = eeprom_read_byte((uint8_t*)EE_PIR) * 1000UL;
	push_timeout[3].code = C_DHT;
	push_timeout[3].value = eeprom_read_byte((uint8_t*)EE_DHT) * 1000UL;
	push_timeout[4].code = C_EMS;
	push_timeout[4].value = eeprom_read_byte((uint8_t*)EE_EMS) * 1000UL;
	push_timeout[5].code = C_SWITCH;
	push_timeout[5].value = eeprom_read_byte((uint8_t*)EE_SWITCH) * 1000UL;

	/* READ NODE ID FROM FLASH */
	node_id = get_id();

	state = RUN;
}

void loop()
{
	Packet packet;

	if(receive(&packet)) {
			/* Wait 10ms before reply to give time for transmission
			 * direction change
			 */
			delay(10);
			if(exec_command(&packet)) {
				prepare_packet(packet.payload.code, &packet);
				//TODO: If the packet will be not received there will be another request from source and this should be not a good thing sometimes
				send(&packet);
			}
	} else {
		switch (state) {
			case RUN: {
				Item item;

				if(dequeue(&item)) {
					if (!push(&item.packet) && item.retry++ <= MAX_RETRY) {
						enqueue(&item);
					}
				} else {
					for (uint8_t i = 0; i < NUMTIMEOUT; i++) {
						if ((push_timeout[i].code) &&
								(push_timeout[i].value) &&
								((millis() - push_timeout[i].timer) > push_timeout[i].value)) {
							prepare_packet(push_timeout[i].code, &packet);
							if(exec_command(&packet) && enqueue(&packet)) {
								push_timeout[i].timer = millis();
							}
						}
					}
				}
				break;
			}
			case STANDBY:
				break;
			case PROGRAM: {
				start_bootloader();
				break;
			}
		}
	}
}

void prepare_packet(uint8_t code, Packet *packet) {
	packet->source = node_id;
	packet->dest = hub_node;
	packet->payload.code = code;
	memset(packet->payload.data, 0, MAX_DATA_SIZE);
}

uint8_t exec_command(Packet *packet) {
	switch(packet->payload.code) {
		case C_CONFIG: {
			switch(packet->payload.data[0]) {
				case C_HBT: {
					unsigned long timeout = (unsigned long)packet->payload.data[1];
					if (timeout > 0) {
						push_timeout[0].value = timeout * 1000UL;
						eeprom_update_byte((uint8_t*)EE_HBT, packet->payload.data[1]);
					}
					break;
				}
				case C_LUX: {
					push_timeout[1].value = (unsigned long)packet->payload.data[1] * 1000UL;
					eeprom_update_byte((uint8_t*)EE_LUX, packet->payload.data[1]);
					break;
				}
				case C_PIR: {
					push_timeout[2].value = (unsigned long)packet->payload.data[1] * 1000UL;
					eeprom_update_byte((uint8_t*)EE_PIR, packet->payload.data[1]);
					break;
				}
				case C_DHT: {
					push_timeout[3].value = (unsigned long)packet->payload.data[1] * 1000UL;
					eeprom_update_byte((uint8_t*)EE_DHT, packet->payload.data[1]);
					break;
				}
				case C_EMS: {
					push_timeout[4].value = (unsigned long)packet->payload.data[1] * 1000UL;
					eeprom_update_byte((uint8_t*)EE_EMS, packet->payload.data[1]);
					break;
				}
				case C_SWITCH: {
					push_timeout[5].value = (unsigned long)packet->payload.data[1] * 1000UL;
					eeprom_update_byte((uint8_t*)EE_SWITCH, packet->payload.data[1]);
					break;
				}
			}
			break;
		}
		case C_LCDCLEAR: {
			oled.clear();
			break;
		}
		case C_LCDPRINT: {
			oled.setFont(fonts[packet->payload.data[2]]);
			oled.setCursor(packet->payload.data[1], packet->payload.data[0]);
			oled.print((const char*)&packet->payload.data[3]);
			break;
		}
		case C_LCDWRITE: {
			oled.setCursor(packet->payload.data[1], packet->payload.data[0]);
			for (uint8_t c = 0; c <= packet->payload.data[2] - 1; c++) {
				oled.ssd1306WriteRamBuf(packet->payload.data[3 + c]);
			}
			break;
		}
		case C_PROGRAM: {
			state = PROGRAM;
			break;
		}
		case C_STANDBY: {
			state = STANDBY;
			break;
		}
		case C_RUN: {
			state = RUN;
			break;
		}
		case C_SETID: {
			uint16_t new_value = packet->payload.data[1] * 256 + packet->payload.data[0];
			set_id(new_value);
			break;
		}
		case C_HBT: {
			break;
		}
		case C_MEM: {
			int free = freeMemory();

			memcpy(packet->payload.data, &free, sizeof(int));
			break;
		}
		case C_LUX: {
			lux_state = analogRead(LUX_IN);

			memcpy(packet->payload.data, &lux_state, sizeof(lux_state));
			break;
		}
		case C_PIR: {
			int new_state = digitalRead(PIR_IN);

			if (new_state == pir_state)
				return 0;

			pir_state = new_state;
			memcpy(packet->payload.data, &pir_state, sizeof(pir_state));
			break;
		}
		case C_DHT: {
			dht_state.temperature = dht_sensor.getTemperature();
			dht_state.humidity = dht_sensor.getHumidity();

			memcpy(packet->payload.data, &dht_state, sizeof(Payload_dht));
			break;
		}
		case C_EMS: {
			for(uint8_t i = 0; i < NUM_EMS; i++) {
				double power = energy[i].calcIrms(1480) * EMON_VOLTAGE;

				if (power == NAN) {
					power = -1.0;
				} else if (power <= 600.0) {
					power *= 0.95;
				}
				ems_state.value[i] = power;
			}

			memcpy(packet->payload.data, &ems_state, sizeof(Payload_ems));
			break;
		}
		case C_LIGHT: {
			// Payload value are 0:unchange 1:toggle
			for(uint8_t i=0; i < NUMLIGHT; i++) {
				lightbuff[i] = packet->payload.data[i] ^ lightbuff[i];
				digitalWrite(LIGHT_BASE_PIN + i, lightbuff[i]);
			}

			memcpy(packet->payload.data, &lightbuff, sizeof(lightbuff));
			break;
		}
		case C_SWITCH: {
			uint8_t digin[NUMSWITCH];
			uint8_t keypressed = 0;

			memset(&digin, 0, sizeof(digin));
			for(uint8_t i=0; i < NUMSWITCH; i++) {
				digin[i] = digitalRead(SWITCH_BASE_PIN + i);
				keypressed += digin[i];
			}

			if (!keypressed)
				return 0;

			memcpy(packet->payload.data, &digin, sizeof(digin));
			break;
		}
		default:
			return 0;
	}
	return 1;
}

void start_bootloader() {
	wdt_disable(); /* Verificare se serve disabilitare il watchdog */
	MCUSR = 0;
	cli();
	noInterrupts();
	asm volatile ("ijmp" ::"z" (0x3e00));
}

uint16_t get_id() {
	return eeprom_read_word((uint16_t*)ID_EE_ADDRESS);
}

void set_id(uint16_t new_value) {
	eeprom_update_word((uint16_t*)ID_EE_ADDRESS, new_value);
}

void showsplash() {
	uint8_t rows = *logo64x64;
	uint8_t cols = *(logo64x64 + 1);

	uint8_t *char_idx = (uint8_t*)logo64x64 + 2;

	for (uint8_t r = 0; r <= rows - 1; r++) {
		oled.setCursor(32, r);
		for (uint8_t c = 0; c <= cols - 1; c++) {
			oled.ssd1306WriteRamBuf(pgm_read_byte(char_idx++));
		}
	}
	delay(2000);
	oled.clear();
}

void flushinputbuffer() {
	while(Serial.available() > 0)
		Serial.read();
}

uint8_t enqueue(Item *item) {
	/* Do not enqueue if there is already a packet with the same code and dest */
	for (uint8_t i=0; i<MAX_QUEUE_SIZE; i++) {
		if (queue[i].packet.payload.code == item->packet.payload.code &&
				queue[i].packet.dest == item->packet.dest) {
			return 2;
		}
	}

	/* Enqueue if there is space available */
	for (uint8_t i=0; i<MAX_QUEUE_SIZE; i++) {
		if (queue[i].packet.payload.code == 0) {
			memcpy(&queue[i], item, sizeof(Item));
			return 1;
		}
	}
	return 0;
}

uint8_t enqueue(Packet *packet) {
	Item item;

	memcpy(&item.packet, packet, sizeof(Packet));
	return enqueue(&item);
}

uint8_t dequeue(Item *item) {
	/* Enqueue if there is something */
	for (uint8_t i=0; i<MAX_QUEUE_SIZE; i++) {
		if (queue[i].packet.payload.code != 0) {
			memcpy(item, &queue[i], sizeof(Item));
			memset(&queue[i], 0, sizeof(Item));
			return 1;
		}
	}
	return 0;
}

uint8_t push(Packet *packet) {
	unsigned long timeout;
	uint8_t received;

	if(send(packet)) {
		timeout = millis();
		while(!(received=receive(packet)) && ((millis() - timeout) < PACKET_TIMEOUT));
		if(!received)
			return 0;
	} else
		return 0;

	return 1;
}

uint8_t send(Packet* pkt) {
	if((Serial.availableForWrite() < (int)sizeof(Packet)) || Serial.available())
		return 0;

	/*
	 * There are 2 delay of 10 us approx a char at 19200
	 * to wait for bus enable and disable to be sure for
	 * a complete transmission
	 */
	digitalWrite(BUS_ENABLE, HIGH);          			// 485 write mode
//	delayMicroseconds(10);
	Serial.write(header, sizeof(header));
	Serial.write((unsigned char *)pkt, sizeof(Packet));

	uint16_t chksum = ModRTU_CRC((char*)pkt, sizeof(Packet));

	Serial.write((char*)&chksum, 2);
	Serial.flush();										// wait for complete transmission
//	delayMicroseconds(10);
	digitalWrite(BUS_ENABLE, LOW);						// 485 read mode
	return 1;
}

char timeRead() {
	unsigned long timeout;
	int c;

	timeout = millis();
	do {
		c = Serial.read();
		if (c >= 0)
			return c;
	} while((millis() - timeout) < PACKET_TIMEOUT);

	return -1;
}

uint8_t receive(Packet* packet) {
	char c;

	// Wait for Header
	if(!Serial.available())
		return 0;

	do {
		c = timeRead();
	} while (c >= 0 && c != header[0]);

	if (c != header[0])
		return 0;

	do {
		c = timeRead();
	} while (c >= 0 && c == header[0]);

	if (c != header[1])
		return 0;

//	uint8_t looking_for_header = 0;
//
//		// Wait for Header
//		while(Serial.available()) {
//			if (Serial.read() == header[0]) {
//				looking_for_header = 1;
//				break;
//			}
//		}
//
//		if (!looking_for_header)
//			return 0;
//
//		unsigned long timeout;
//
//		timeout = millis();
//		while((Serial.available() < 2) && ((millis() - timeout) < PACKET_TIMEOUT) );
//
//		if (Serial.available() < 2)
//			return 0;
//
//		if (Serial.read() != header[1])
//			return 0;

	uint16_t size = timeRead();
	char buffer[MAX_BUFFER_SIZE];

	// Wait for packet
	Serial.setTimeout(PACKET_TIMEOUT);
	if ((size > sizeof(Packet)) || (Serial.readBytes(buffer, size) < size))
		return 0;

	// Wait for CRC
	uint16_t chksum;

	if (Serial.readBytes((char *)&chksum, 2) < 2)
		return 0;

	// Check CRC
	if (chksum!=ModRTU_CRC(buffer, size))
		return 0;

	memcpy(packet, buffer, size);
	if(packet->dest!=node_id)
		return 0;

	return 1;
}
