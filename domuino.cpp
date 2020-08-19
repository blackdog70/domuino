/*
 * Domuino.cpp
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#include "domuino.h"
#include "logo.h"

Timeout push_timeout[sizeof(commands)/sizeof(command_type)];
uint16_t node_id;
uint16_t version = 11;
char running = 0;
char numswitch = 0;
char numlight = 0;

#define NUMTIMEOUT (sizeof(push_timeout) / sizeof(Timeout))

void setup()
{
	/* INIT BUS */
	Serial.begin(NET_BAUDRATE);
	Serial.setTimeout(NET_PACKET_TIMEOUT);

	pinMode(BUS_ENABLE, OUTPUT);
	hub_node = 1;	// Set to default hub

	/* INIT IO PINS */
	pinMode(DHT_PIN, INPUT);
	pinMode(PIR_IN, INPUT);
	pinMode(LUX_IN, INPUT);
	dht_sensor.setup(DHT_PIN);
	numswitch = eeprom_read_byte((uint8_t*)EE_SWITCH);
	numlight =  eeprom_read_byte((uint8_t*)EE_LIGHT);
	for(uint8_t i=0; i<numswitch; i++)
		pinMode(SWITCH_BASE_PIN + i, INPUT);
	for(uint8_t i=0; i<numlight; i++) {
		pinMode(SWITCH_BASE_PIN + numswitch + i, OUTPUT);
		digitalWrite(SWITCH_BASE_PIN + numswitch + i, LOW);
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

	for(uint8_t i=0; i<sizeof(commands)/sizeof(command_type); i++) {
		switch(commands[0].type) {
			case TIMER_COMMAND: {
				push_timeout[i].code = commands[i].command;
				push_timeout[i].value = eeprom_read_byte((uint8_t*)EE_BASE+i) * 1000UL;
				break;
			}
			case TRIGGER_COMMAND: {
				push_timeout[i].code = commands[i].command;
				push_timeout[i].value = SWITCH_TIMEOUT;
				break;
			}
		}
	}

	// If is set then init LCD
	if (eeprom_read_byte((uint8_t*)EE_LCD)) {
		//oled.begin(&Adafruit128x64, I2C_ADDRESS, false);
		oled.begin(&Adafruit128x64, I2C_ADDRESS);
		//oled.set400kHz();

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
		//showsplash();
	    oled.setFont(fonts[0]);
		oled.setCursor(25, 3);
		oled.print("Initializing...");
		delay(2000);
	    oled.clear();
	}

	/* READ NODE ID FROM FLASH */
	node_id = get_id();

	state = RUN;
}

void loop()
{
	Packet packet;

	if(receive(&packet) && exec_command(&packet)) {
		send(&packet);
		// If the last was a config command then restart to apply the new settings
		if (packet.payload.code == C_CONFIG)
			RESET();
	} else {
		switch (state) {
			case RUN: {
				Item item;

				if(dequeue(&item)) {
					if (!push(&item.packet) && ((millis() - item.timeout) < PACKET_LIFETIME)) {
						enqueue(&item);
					}
				} else {
					for (uint8_t i = 0; i < NUMTIMEOUT; i++) {
						if ((push_timeout[i].code) &&
								(push_timeout[i].value) &&
								((millis() - push_timeout[i].timer) > push_timeout[i].value)) {
							packet.payload.code = push_timeout[i].code;
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

uint8_t exec_command(Packet *packet) {
	packet->source = node_id;
	packet->dest = hub_node;

	switch(packet->payload.code) {
		case C_CONFIG: {
			for(uint8_t i=0; i< sizeof(commands)/sizeof(command_type); i++) {
				if(packet->payload.data[0] == commands[i].command)
					eeprom_update_byte((uint8_t*)EE_BASE+i, packet->payload.data[1]);
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
			for(uint8_t i=0; i < numlight; i++) {
				lightbuff[i] = packet->payload.data[i] ^ lightbuff[i];
				digitalWrite(SWITCH_BASE_PIN + numswitch + i, lightbuff[i]);
			}

			memcpy(packet->payload.data, &lightbuff, sizeof(lightbuff));
			break;
		}
		case C_SWITCH: {
			uint8_t digin[numswitch];
			uint8_t keypressed = 0;

			memset(&digin, 0, sizeof(digin));
			for(uint8_t i=0; i < numswitch; i++) {
				digin[i] = digitalRead(SWITCH_BASE_PIN + i);
				keypressed += digin[i];
			}

			if (!keypressed)
				return 0;

			memcpy(packet->payload.data, &digin, sizeof(digin));
			break;
		}
		case C_VERSION: {
			memcpy(packet->payload.data, &version, sizeof(int));
			break;
		}
		default:
			return 0;
	}

	packet->crc = ModRTU_CRC((char*)packet, PACKET_SIZE); // avoid crc in calc
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
	node_id = new_value;
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
	if (queue_idx < QUEUE_MAX_SIZE - 2) { // -2 because I reserve a queue slot for instant reply
		queue_idx++;
		memcpy(&queue[queue_idx], item, sizeof(Item));
		return 1;
	}

	return 0;
}

uint8_t enqueue(Packet *packet) {
	Item item;

	memcpy(&item.packet, packet, sizeof(Packet));
	item.timeout = millis();
	return enqueue(&item);
}

uint8_t dequeue(Item *item) {
	if (queue_idx >= 0) {
		memcpy(item, &queue[queue_idx], sizeof(Item));
		queue_idx--;
		return 1;
	}

	return 0;
}

uint8_t push(Packet *packet) {
	unsigned long timeout;
	uint8_t received;

	if(send(packet)) {
		timeout = millis();
		while(!(received=receive(packet)) && ((millis() - timeout) < NET_PACKET_TIMEOUT));
		if(!received) {
			delayMicroseconds(node_id * _BYTE_DURATION_ + _BYTE_DURATION_);
			return 0;
		}
	} else
		return 0;

	return 1;
}

uint8_t send(Packet* packet) {
	if((Serial.availableForWrite() < (int)sizeof(Packet)) || Serial.available())
		return 0;

	/*
	 * There are 2 delay
	 * to wait for bus enable and disable to be sure for
	 * a complete transmission
	 */
	digitalWrite(BUS_ENABLE, HIGH);          			// 485 write mode
	delayMicroseconds(10);

	Serial.write((unsigned char *)packet, sizeof(Packet));
	Serial.flush();										// wait for complete transmission

	delayMicroseconds(10);
	digitalWrite(BUS_ENABLE, LOW);						// 485 read mode
	return 1;
}

uint8_t receive(Packet* packet) {
	if(!Serial.find((char*)&packet->header, sizeof(packet->header)) or					// Wait for Header
			(Serial.readBytes((char*)&packet->source, PACKET_SIZE) < PACKET_SIZE) or	// Wait for packet
			(packet->dest!=node_id and packet->dest!=BROADCAST)	or						// Check destination
			(packet->crc!=ModRTU_CRC((char*)packet, PACKET_SIZE)) 						// Check CRC
	)
		return 0;

	return 1;
}
