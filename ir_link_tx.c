/*
 * rc5_link_tx.c
 *
 *  Created on: 11 mar 2022
 *      Author: Mariusz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ir_link_tx.h"

#define IR_LINK_TX_FRAME_DATA_BITS	32
#define IR_LINK_TX_TCNT				(255 - IR_LINK_TX_BURST_LENGTH)
#define IR_LINK_TX_TCNT_ZERO		(255 - IR_LINK_TX_BURST_LENGTH)
#define IR_LINK_TX_TCNT_ONE			(255 - IR_LINK_TX_BURST_LENGTH * 2)


enum IR_LINK_TX_STATUS{
	IR_LINK_STATUS_EMPTY,
	IR_LINK_TX_STATUS_PREPARE,
	IR_LINK_STATUS_READY
};

static volatile uint8_t ir_link_tx_status = IR_LINK_STATUS_EMPTY;
static volatile uint32_t ir_link_tx_data;

void ir_link_tx_init(void){
	// T0 	- PD4
	// OC1B - PB2
	DDRB |= (1<<PB2) | (1<<PB1);

	// carrier 36kHz
	TCCR1A |= (1<<COM1A0);
	TCCR1B |= (1<<WGM12) | (1<<CS10);
	OCR1A = 111; // carrier for T0
	OCR1B = 111; // carrier out

	// carrier periods counter
	TCCR0 |= (1<<CS02) | (1<<CS01);
}

static uint8_t ir_link_tx_CRC8(uint8_t* bytes, uint8_t size){
    const uint8_t generator = 0x1D;
    uint8_t crc = 0; // start with 0 so first byte can be 'xored' in

    for(uint8_t i = 0; i < size; i++){
        crc ^= bytes[i]; // XOR-in the next input byte

        for(uint8_t j = 0; j < 8; j++){
            if((crc & 0x80) != 0){
                crc = (uint8_t)((crc << 1) ^ generator);
            }
            else{
                crc <<= 1;
            }
        }
    }

    return crc;
}

void ir_link_tx_emit(ir_link_tx_payload* payload){
	if(ir_link_tx_status == IR_LINK_STATUS_EMPTY){
		uint8_t data[4];

		data[0] = ((payload->address & 0x03) << 2) | (payload->data0to2bit & 0x03);
		data[1] = payload->data0to8bit1;
		data[2] = payload->data0to8bit2;
		data[3] = ir_link_tx_CRC8((uint8_t*)&data, 3);

	    data[0] = ((data[0] << 2) & 0x3C) | ((data[1] >> 6) & 0x03);
	    data[1] = ((data[1] << 2) & 0xFC) | ((data[2] >> 6) & 0x03);
	    data[2] = ((data[2] << 2) & 0xFC) | ((data[3] >> 6) & 0x03);
	    data[3] = ((data[3] << 2) & 0xFC) | 0x03;

		ir_link_tx_data = 	((uint32_t)data[0] << 24) |
							((uint32_t)data[1] << 16) |
							((uint32_t)data[2] << 8) |
							(uint32_t)data[3];

		TCNT1 = 0;					// reset carrier generator
		TCNT0 = IR_LINK_TX_TCNT; 	// delay
		TIMSK |= (1<<TOIE0);

		ir_link_tx_status = IR_LINK_TX_STATUS_PREPARE;
	}
}

ISR(TIMER0_OVF_vect){
    static uint8_t emitted_bits;
	static uint8_t emit_cnt;

	if(ir_link_tx_status == IR_LINK_TX_STATUS_PREPARE){
		emitted_bits = IR_LINK_TX_FRAME_DATA_BITS;
		emit_cnt = 0;
		ir_link_tx_status = IR_LINK_STATUS_READY;
	}

	if(ir_link_tx_status == IR_LINK_STATUS_READY){
		emit_cnt++;

		if((emit_cnt & 0x01) && emitted_bits){
			if(ir_link_tx_data & ((uint32_t)1<<(emitted_bits - 1))){
                TCCR1A |= (1<<COM1B0);
                TCNT0 = IR_LINK_TX_TCNT_ONE;
			}
			else{
                TCCR1A |= (1<<COM1B0);
                TCNT0 = IR_LINK_TX_TCNT_ZERO;
			}
		}
		else{
			TCCR1A &= ~(1<<COM1B0);

			if(emitted_bits){
				emitted_bits--;

				TCNT0 = IR_LINK_TX_TCNT_ZERO;
			}
			else{
				TIMSK &= ~(1<<TOIE0);
				ir_link_tx_status = IR_LINK_STATUS_EMPTY;
			}
		}
	}
}
