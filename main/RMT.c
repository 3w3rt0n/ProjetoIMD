/*
 * RMT.c
 *
 *  Created on: 20 de nov de 2017
 *      Author: Ewerton L. de Sousa
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"


#define RMT_RX_ACTIVE_LEVEL  1   /*!< Data bit is active high for self test mode */
#define RMT_TX_CARRIER_EN    1   /*!< Disable carrier for self test mode  */

#define RMT_TX_CHANNEL    0     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  15		/*!< 18 GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    1     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  18     /*!< 19 GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  100    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  4000   /*9500!< RMT receiver timeout value(us) */

#define NEC_HEADER_HIGH_US    9125  //7300*1,25                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     2175  //1740*1,25                       /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US   646   //517*1,25
#define NEC_BIT_ONE_LOW_US    1587  //1270*1,25
#define NEC_BIT_ZERO_HIGH_US  646   //517*1,25                     /*!< NEC protocol data bit 0: positive 0.56ms */ //Valores a serem testados
#define NEC_BIT_ZERO_LOW_US   487   //390*1,25
#define NEC_BIT_END           647   //518*1,25
#define NEC_MARGIN			  10


//Oled
typedef struct{
	char texto[20];
	int x;
	int y;
    int c;
}TEXTO;

uint16_t IRsignal[] =
{
// ON, OFF (in 1,25's of microseconds)
 4550, 4500,	//00
  545, 1690,	//01
  545, 1690,	//02
  545, 1690,	//03
  545, 570,		//04
  545, 570,		//05
  545, 570,		//06
  545, 570,		//07
  545, 570,		//08
  545, 1690,	//09
  545, 1690,	//10
  545, 1690,	//11
  545, 570,		//12
  545, 570,		//13
  545, 570,		//14
  545, 570,		//15
  545, 570,		//16
  545, 570,		//17
  545, 1690,	//18
  545, 570,		//19
  545, 570,		//20
  545, 570,		//21
  545, 570,     //22
  545, 570,     //23
  545, 570,     //24
  545, 1690,    //25
  545, 570,     //26
  545, 1690,    //27
  545, 1690,    //28
  545, 1690,    //29
  545, 1690,    //30
  545, 1690,    //31
  545, 1690,    //32
  545, 1690		//33
};



/*
 * @brief Build register value of waveform for NEC one data bit
 */
static inline void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}


/*
 * @brief RMT transmitter initialization
 */
void TX_INICIALIZACAO()
{
    rmt_config_t rmt_tx;
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1; //1 block = 62 entradas de 32bits
    rmt_tx.clk_div = 80;   // divisor = 100 => 80.000.000 / 100 = 800k => 1.25 msec
    rmt_tx.tx_config.loop_en = 0; //false
    rmt_tx.tx_config.carrier_duty_percent = 33;
    rmt_tx.tx_config.carrier_freq_hz = 37900; //38000
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

/*
 * @brief RMT receiver initialization
 */
void RX_INICIALIZACAO()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = 80;
    rmt_rx.mem_block_num = 1; // 1 - original
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 200;
    rmt_rx.rx_config.idle_threshold = 9000;
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);

}

void imprimir_ITEM(rmt_item32_t* item, int num_de_bits){

	rmt_item32_t* item_temp = item;
	for(int i=0; i<num_de_bits; i++){
		printf("[%d] %d - %dus \n", i, item_temp->level0, item_temp->duration0);
		printf("[%d] %d - %dus \n", i, item_temp->level1, item_temp->duration1);
		item_temp++;
	}
}


void RMT_TX_TV(void *pvParameter){

	int nec_tx_num = RMT_TX_DATA_NUM;
	size_t size = (sizeof(rmt_item32_t) * 120 * nec_tx_num);
	//each item represent a cycle of waveform.

	TX_INICIALIZACAO();

	while(1) {

		rmt_item32_t* item = (rmt_item32_t*) malloc(size);
		memset((void*) item, 0, size);
		rmt_item32_t* item_temp = item;

		for(int i = 0; i < 68; i=i+2){

			 item_temp->level0 = 1;
			 item_temp->duration0 = IRsignal[i];
			 printf( "[%2d] - %d4us | ", i, IRsignal[i]);
			 //item_temp->duration0 = IRsignal[i] / 10 * RMT_TICK_10_US;
			 //printf( "[%2d] - %d4us | ", i, IRsignal[i] / 10 * RMT_TICK_10_US);
			 item_temp->level1 = 0;
			 item_temp->duration1 = IRsignal[i+1];
			 printf( "%d4us \n",IRsignal[i+1]);
			 //item_temp->duration1 = IRsignal[i+1] / 10 * RMT_TICK_10_US;
			 //printf( "%d4us \n",IRsignal[i+1] / 10 * RMT_TICK_10_US);

			item_temp++;
		}


		//imprimir_ITEM(item,60); //Debug verificar dados enviados pelo IR

		rmt_write_items(RMT_TX_CHANNEL, item, 68, true);
		rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);


		printf( "FIM do envio. \n");
		free(item);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

rmt_item32_t* RMT_BUTTON_INIT(){
	size_t size = (sizeof(rmt_item32_t) * 120 * 100);
	//each item represent a cycle of waveform.

	TX_INICIALIZACAO();

	rmt_item32_t* item = (rmt_item32_t*) malloc(size);
	memset((void*) item, 0, size);

	for(int i = 1; i < 69; i=i+2){

		 (item+i)->level0 = 1;
		 (item+i)->duration0 = IRsignal[i];
		 //printf( "[%2d] - %d4us | ", i, IRsignal[i]);
		 (item+i)->level1 = 0;
		 (item+i)->duration1 = IRsignal[i+1];
		 //printf( "%d4us \n",IRsignal[i+1]);

	}

	return item;
}


/*original
rmt_item32_t* RMT_BUTTON_INIT(){
	size_t size = (sizeof(rmt_item32_t) * 120 * 100);
	//each item represent a cycle of waveform.

	TX_INICIALIZACAO();

	rmt_item32_t* item = (rmt_item32_t*) malloc(size);
	memset((void*) item, 0, size);
	rmt_item32_t* item_temp = item;

	for(int i = 0; i < 68; i=i+2){

		 item_temp->level0 = 1;
		 item_temp->duration0 = IRsignal[i];
		 //printf( "[%2d] - %d4us | ", i, IRsignal[i]);
		 item_temp->level1 = 0;
		 item_temp->duration1 = IRsignal[i+1];
		 //printf( "%d4us \n",IRsignal[i+1]);

		 item_temp++;
	}

	return item;
}
*/

void RMT_BUTTON_SEND(rmt_item32_t* item){
	rmt_write_items(RMT_TX_CHANNEL, item, 68, true);
	rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
	printf("RMT ENVIADO!\n");
}

void RMT_BUTTON_RECEIVE(rmt_item32_t* item){

	RingbufHandle_t rb = NULL;

	rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
	rmt_rx_start(RMT_RX_CHANNEL, 1);

	while(rb) {
		size_t rx_size = 0;
		item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
		xRingbufferPrintInfo(rb);
		if(item) {
			printf( "RMT RECEBIDO!\n");
			printf("       [HIGH]   [LOW]\n");
			for(int i=0; i<33; i++){
				printf("[%2d] - %4dus ", i, (item+i)->duration0);
				printf("| %4dus \n", (item+i)->duration1);
			}

			vRingbufferReturnItem(rb, (void*) item);
		} else {
			break;
		}
	}

}

/*void RMT_RX(void *pvParameter){

	 RX_INICIALIZACAO();

	 char dados[35]= {0};

	 //Configuração da fila
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	TEXTO texto;
	xQueueHandle xQueue = (xQueueHandle)pvParameter;

	 while(1){

		RingbufHandle_t rb = NULL;

		int channel = RMT_RX_CHANNEL;
		rmt_get_ringbuf_handle(channel, &rb);
		rmt_rx_start(channel, 1);

		while(rb) {
			size_t rx_size = 0;
			//try to receive data from ringbuffer.
			//RMT driver will push all the data it receives to its ringbuffer.
			//We just need to parse the value and return the spaces of ringbuffer.
			rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
			rmt_item32_t* item_temp = item;
			xRingbufferPrintInfo(rb);
			if(item) {
				printf( "Dados recebidos: \n");
				printf("       [HIGH]   [LOW]\n");
				for(int i=0; i<67; i++){
					printf("[%2d] - %dus ", i, item->duration0);
					printf("| %dus \n", item->duration1);
					item++;
				}

				//sprintf(texto.texto, "RX: %s ", dados);
				//texto.x = 2;
				//texto.y = 15;
				//texto.c = 0;

				//xStatus = xQueueSendToFront( xQueue, &texto, xTicksToWait );
				//if( xStatus == pdPASS ) {
				//	printf( "Enviado para o display. \n" );
				//}

				//after parsing the data, return spaces to ringbuffer.
				vRingbufferReturnItem(rb, (void*) item_temp);
			} else {
				//xRingbufferPrintInfo(rb);
				break;
			}
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);

}*/
