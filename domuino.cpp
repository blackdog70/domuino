/*
 * Domuino.cpp
 *
 *  Created on: May 25, 2016
 *      Author: sebastiano
 */

#include "domuino.h"

Timeout push_timeout[6];
int info_type = 0;
uint16_t node_id;
uint8_t running = 0;

#define NUMTIMEOUT (sizeof(push_timeout) / sizeof(Timeout))

void setup()
{
	/* INIT BUS */
	Serial.begin(19200);
	pinMode(BUS_ENABLE, OUTPUT);
	hub_node = 1;	// Set to default hub

	/* INIT LCD */
	oled.begin(&Adafruit128x64, I2C_ADDRESS, false);

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
//	oledDraw(32, 0, (uint8_t*)hass64x64);
//	delay(3000);
	oled.clear();

	/* INIT IO PINS */
	pinMode(DHT_PIN, INPUT);
	pinMode(PIR_IN, INPUT);
	pinMode(LUX_IN, INPUT);
	dht_sensor.setup(DHT_PIN);

	/* INITIAL SENSOR STATE */
	pir_state = LOW;

	/* INIT TOUCH */
//	for(uint8_t i=0; i < NUMTOUCH; i++)
//		calibrate_touch(i);

//	/* INIT TIMEOUT */
	push_timeout[0].code = C_HBT;
//	push_timeout[0].value = 2;
	push_timeout[1].code = C_LUX;
//	push_timeout[1].value = 30000;
	push_timeout[2].code = C_PIR;
//	push_timeout[2].value = 1;
	push_timeout[3].code = C_DHT;
	push_timeout[3].value = 10000;
	push_timeout[4].code = C_EMS;
//	push_timeout[4].value = 1;
	push_timeout[5].code = C_SWITCH;
	push_timeout[5].value = 500;

	/* READ NODE ID FROM FLASH */
	node_id = get_id();

	/* SEND START */
	Packet packet;

	prepare_packet(C_START, &packet);
	enqueue(&packet);
}

void loop()
{
	Packet packet;
	Item item;

	if(receive(&packet)) {
//		Serial.print("Source=");
//		Serial.println(packet.source);
//		Serial.print("Dest=");
//		Serial.println(packet.dest);
//		Serial.print("Code=");
//		Serial.println(packet.payload.code, HEX);
//		Serial.print("Data=");
//		Serial.write((const char *)packet.payload.data, sizeof(packet.payload.data));
		if (packet.dest != node_id) {
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
							push_timeout[0].value = packet.payload.data[1] * 1000UL;
						case C_LUX:
							push_timeout[1].value = packet.payload.data[1] * 1000UL;
						case C_PIR:
							push_timeout[2].value = packet.payload.data[1] * 1000UL;
						case C_DHT:
							push_timeout[3].value = packet.payload.data[1] * 1000UL;
						case C_EMS:
							push_timeout[4].value = packet.payload.data[1] * 1000UL;
						case C_SWITCH:
							push_timeout[4].value = packet.payload.data[1] * 1000UL;
					}
					break;
				}
				default:
					prepare_packet(packet.payload.code, &packet);
			}
			send(&packet);
			if(packet.payload.code == C_RESET)
				start_bootloader();
		}
	} else {
		for (uint8_t i = 0; i < NUMTIMEOUT; i++) {
			if ((push_timeout[i].code) &&
					(push_timeout[i].value) &&
					((millis() - push_timeout[i].timer) > push_timeout[i].value)) {
				if(prepare_packet(push_timeout[i].code, &packet) && enqueue(&packet)) {
					push_timeout[i].timer = millis();
				}
			}
		}
		while (dequeue(&item)) {
			if (!push(&item.packet) && item.retry++ <= MAX_RETRY) {
				enqueue(&item);
			}
		}
	}
	display_info();
}

void display_info()
{
	char tmp[20];

	oled.setFont(font5x7);
	oled.setCursor(40, 0);
	if(running)
		oled.print(".DOMUINO.");
	else
		oled.print(" DOMUINO ");
	running = 1 - running;

	oled.setCursor(5,  3);
	oled.print("TEMP");
	oled.setCursor(119,  3);
	oled.print("C");
	oled.setCursor(5,  6);
	oled.print("HUM");
	oled.setCursor(119,  6);
	oled.print("%");
	oled.setFont(lcdnums14x24);
	oled.setCursor(40,  2);
	oled.print(dht_state.temperature / 10.0);
	oled.setCursor(40,  5);
	oled.print(dht_state.humidity / 10.0);


//	if (touch[0].state) {
//		if(++info_type > 2)
//			info_type = 0;
//		clearScr();
//	} else if (touch[1].state) {
//		if(--info_type < 0)
//			info_type = 2;
//		clearScr();
//	}
//
//	current_font = (uint8_t*)SmallFont;
//	if(running)
//		oledWriteString(40, 0, ".DOMUINO.");
//	else
//		oledWriteString(40, 0, " DOMUINO ");
//	running = 1 - running;
//
//	switch (info_type) {
//		case 0:
//			oledWriteString(5, 3, "TEMP");
//			oledWriteString(119, 3, "C");
//			oledWriteString(5, 6, "HUM");
//			oledWriteString(119, 6, "%");
////			current_font = (uint8_t*)digits14x24;
////			dtostrf(dht_state.temperature / 10.0, 5, 1, tmp);
//			oledWriteString(40, 2, "41.2");
////			dtostrf(dht_state.humidity / 10.0, 5, 1, tmp);
//			oledWriteString(40, 5, "60.3");
//			break;
//		case 1: {
//			oledWriteString(0, 3, "LUX");
//			current_font = (uint8_t*)digits14x24;
//			dtostrf(lux_state, 4, 0, tmp);
//			oledWriteString(40, 2, tmp);
//			break;
//		}
//		case 2:
//			oledWriteString(0, 3, "PIR");
//			current_font = (uint8_t*)digits14x24;
//			dtostrf(pir_state, 2, 0 , tmp);
//			oledWriteString(40, 2, tmp);
//			break;
//		case 3:
////			oledWriteString(0, 2, "Touch");
////			current_font = (uint8_t*)digits14x24;
////			for (int i=0; i < NUMTOUCH; i++) {
////				sprintf(tmp, "%i\0", touch[i].state);
////				oledWriteString(30 + (16 * i), 2, tmp);
////			}
//			break;
//		default:
//			break;
//	}
}

//void calibrate_touch(int i) {
//    touch[i].base_value = touch_sensor.readRaw(TOUCH_PIN + i, TOUCHSAMPLES);    //create reference for off state
//}

uint8_t prepare_packet(uint8_t code, Packet *packet) {
	packet->source = node_id;
	packet->dest = hub_node;
	packet->payload.code = code;
	memset(packet->payload.data, 0, MAX_DATA_SIZE);
	switch(code) {
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
		case C_SWITCH: {
			uint8_t switches[6] = {0, 0, 0 ,0 ,0, 0};
			uint8_t keypressed;

			for(uint8_t i=0; i < NUMTOUCH; i++) {
				switches[i] = digitalRead(TOUCH_PIN + i);
				keypressed += switches[i];
			}

			if (!keypressed)
				return 0;

			memcpy(packet->payload.data, &switches, sizeof(switches));
			break;
		}
		case C_START: {
			break;
		}
		default:
			return 0;
	}
	return 1;
}

void start_bootloader()
{
	for(uint8_t i=0; i < 7; i++) {
		digitalWrite(BUS_ENABLE, HIGH);
		delay(200);
		digitalWrite(BUS_ENABLE, LOW);
		delay(50);
	}

	MCUSR = 0;
	cli();
	noInterrupts();
	asm volatile ("ijmp" ::"z" (0x3D00));
}

uint16_t get_id() {
	/* READ NODE ID FROM FLASH */
	uint16_t address = 0x3ffc;
	uint8_t ch;
	uint16_t id;

	__asm__ ("lpm %0,Z+\n" : "=r" (ch) , "=z" (address) : "1" (address));
	id = ch;

	__asm__ ("lpm %0,Z+\n" : "=r" (ch) , "=z" (address) : "1" (address));
	id += 256 * ch;

	return id;
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

	send(packet);
	timeout = millis();
	while(!receive(packet) && (millis() - timeout) < PACKET_TIMEOUT);
	if(packet->dest == node_id) {
		/* TODO: da completare con interprete pacchetto */
	} else {
		delay(PACKET_TIMEOUT);
		flushinputbuffer();
		return 0;
	}
	return 1;
}

uint8_t send(Packet* pkt) {
	if(Serial.availableForWrite() < (int)sizeof(Packet)) {
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

uint8_t receive(Packet* packet) {
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
