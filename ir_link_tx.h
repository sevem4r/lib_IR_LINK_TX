/*
 * rc5_link_tx.h
 *
 *  Created on: 23 sty 2019
 *      Author: Mariusz
 */

#ifndef RC5_LINK_TX_RC5_LINK_TX_H_
#define RC5_LINK_TX_RC5_LINK_TX_H_

#include <inttypes.h>

// Connect T0(PD4) to OC1A(PB1)
// Output: OC1B(PB2)

// |------------- 100ms ---------------|
// |--- frame < 30ms ---|
// 		   				|--- > 70ms ---|
//						|- rx 1us ovf -|

#define IR_LINK_TX_BURST_LENGTH		15

typedef struct{
	uint8_t address;
	uint8_t data0to2bit;
	uint8_t data0to8bit1;
	uint8_t data0to8bit2;
}ir_link_tx_payload;

void ir_link_tx_init(void);
void ir_link_tx_emit(ir_link_tx_payload* payload);

#endif /* RC5_LINK_TX_RC5_LINK_TX_H_ */
