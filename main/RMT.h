/*
 * RMT.h
 *
 *  Created on: 20 de nov de 2017
 *      Author: Ewerton L. de Sousa
 */

#ifndef MAIN_RMT_H_
#define MAIN_RMT_H_

void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us);

int nec_build_items(int channel, rmt_item32_t* item, int item_num, uint16_t addr, uint16_t cmd_data);

//debug
void imprimir_ITEM(rmt_item32_t* item, int num_de_bits);

void RMT_TX();
void RMT_TX_TV();

void TX_INICIALIZACAO();
rmt_item32_t* RMT_BUTTON_INIT();
void RMT_BUTTON_SEND(rmt_item32_t* item);

void RX_INICIALIZACAO();
void RMT_RX(void *pvParameter);
void RMT_BUTTON_RECEIVE(rmt_item32_t* item);


#endif /* MAIN_RMT_H_ */
