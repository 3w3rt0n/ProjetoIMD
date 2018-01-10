/*
 * Main.c
 *
 *  Created on: 20 de nov de 2017
 *      Author: Ewerton L. de Sousa
 */

/* Projeto de automação de sala de aula


*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
//FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
//ESP
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
//DRIVER
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
//ADC
#include "driver/adc.h"
//Touch_PAD
#include "driver/touch_pad.h"
//OUTROS
#include "soc/rmt_reg.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
//RMT
#include "driver/rmt.h"

//Include do projeto
#include "RMT.h"			//Infrevermelho
#include "DHT22.h"			//Sensor DHT22

//OLED u8g2
#include <u8g2.h>
#include "u8g2_esp32_hal.h"

typedef struct{
	char texto[20];
	int x;
	int y;
    int c;
}TEXTO;

//Fila para enviar texto para o display
xQueueHandle xQueue;

// SDA - GPIO21
#define PIN_SDA 5

// SCL - GPIO22
#define PIN_SCL 4

//Led's
#define LED_SEND 25
#define LED_PROG 26

//Button's
#define btnLIGAR 23
#define btnAUMENTAR 22
#define btnDIMINUIR 21

//INFRARED RECEPTOR
#define receptor 18

//INFRARED EMISSOR
#define transmissor 15

//Items do IR
rmt_item32_t* IR1;
rmt_item32_t* IR2;
rmt_item32_t* IR3;

/*
 * @Brief Display do OLED WEMOS
 *
 * SDA: 5
 * SCL: 4
 *
 */
void display_OLED_u8g2(void *pvParameter){

	u8g2_t u8g2;

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	//u8g2_t u8g2; // a structure which will contain all the data for one display
	//u8g2_Setup_ssd1306_i2c_128x32_univision_f(
	u8g2_Setup_ssd1306_128x64_noname_f(
		&u8g2,
		U8G2_R0, //U8G2_R2 inverte 180 graus
		//u8x8_byte_sw_i2c,
		u8g2_esp32_msg_i2c_cb,
		u8g2_esp32_msg_i2c_and_delay_cb);  // init u8g2 structure

	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_ClearBuffer(&u8g2);

	u8g2_DrawFrame(&u8g2, 2, 15, 124, 35);

	u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	u8g2_DrawStr(&u8g2, 10,40,"Iniciando...");
	u8g2_SendBuffer(&u8g2);


	vTaskDelay( 500 / portTICK_RATE_MS );
	u8g2_ClearBuffer(&u8g2);
	u8g2_SendBuffer(&u8g2);
	//Configuração da fila
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	TEXTO texto;

	while(1){
		xStatus = xQueueReceive( xQueue, &texto, xTicksToWait );

		if(xStatus == pdPASS){
			if(texto.c){
				u8g2_ClearBuffer(&u8g2);
			}
			u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
			u8g2_DrawStr(&u8g2, texto.x, texto.y, texto.texto);
			u8g2_SendBuffer(&u8g2);
		}
		vTaskDelay( 100 / portTICK_RATE_MS );
	}

	vTaskDelete(NULL);

}

/*
 * @brief Tarefa para leitura do touch pad
 *
 *
 */
void ler_Touch_PAD(void *pvParameter){

	//LED Pino 16

	//Configuração da çeitura da fila
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	TEXTO texto;

	while(1){
		//Touch_pad
		touch_pad_init();
		uint16_t touch_value;
		ESP_ERROR_CHECK(touch_pad_read(TOUCH_PAD_NUM6, &touch_value));
		ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM6, touch_value/2));

		touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);
		touch_pad_filter_start(10);
		//int touch_value;
		//touch_pad_read_filtered(TOUCH_PAD_NUM6, &touch_value);
		printf( "Touch: %d \n", touch_value );

		sprintf(texto.texto, "T0:%4d  ", touch_value);
		texto.x = 2;
		texto.y = 62;
		texto.c = 0;

		xStatus = xQueueSendToFront( xQueue, &texto, xTicksToWait );
		if( xStatus == pdPASS ) {
			printf( "Enviado para o display. \n" );
		}

	    touch_pad_filter_stop();
		touch_pad_deinit();
		vTaskDelay( 2000 / portTICK_RATE_MS );

	}

	vTaskDelete(NULL);

	/*touch_pad_read(TOUCH_PAD_NUM0, &touch_value);
	touch_pad_read(TOUCH_PAD_NUM5, &touch_value);
	touch_pad_read(TOUCH_PAD_NUM6, &touch_value);*/



}

/*
 * @brief Tarefa para ler o ADC (LDR)
 *
 */
void ler_ADC(void *pvParameter){

	//ADC - LDR
	int read_raw;
	adc1_config_channel_atten(ADC1_GPIO39_CHANNEL, ADC_ATTEN_0db );

	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	TEXTO texto;

	while(1){

		read_raw = adc1_get_raw(ADC1_GPIO39_CHANNEL);

		printf( "LDR: %d \n", read_raw );

		sprintf(texto.texto, "LDR:%dL  ", read_raw);
		texto.x = 2;
		texto.y = 45;
		texto.c = 0;

		xQueueSendToFront( xQueue, &texto, xTicksToWait );
		vTaskDelay( 2050 / portTICK_RATE_MS );
	}

	vTaskDelete(NULL);

}


/*
 * @brief Tarefa responsável pela leitura da temperatura e umidade do ar, no sensor DHT22
 *
 *  	 __________
 *      |          |	VCC: 3.3 ~ 6V
 *      |   DHT    |
 *      |   22     |
 *      |__________|
 *      |  |   |   |
 *     VCC NC DATA GND
 *
 */
void DHT_task(void *pvParameter)
{
	//DHT
	setDHTgpio( 16 );
	printf( "Starting DHT Task\n\n");

	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	TEXTO texto;


	while(1) {

		printf("=== Reading DHT ===\n" );
		int ret = readDHT();

		errorHandler(ret);

		printf( "Umid: %2.1f %% \n", getHumidity() );
		printf( "Temp: %2.1f *C \n", getTemperature() );

		sprintf(texto.texto, "HR:%2.2f%%  ", getHumidity());
		texto.x = 2;
		texto.y = 15;
		texto.c = 0;

		xQueueSendToFront( xQueue, &texto, xTicksToWait );

		sprintf(texto.texto, "Tem:%2.1f*C  ", getTemperature());
		texto.x = 2;
		texto.y = 30;
		texto.c = 0;

		xQueueSendToFront( xQueue, &texto, xTicksToWait );

		// -- wait at least 2 sec before reading again ------------
		// The interval of whole process must be beyond 2 seconds !!
		vTaskDelay( 2000 / portTICK_RATE_MS );
	}

	vTaskDelete(NULL);

}


/*
 *
 * @Brief Tarefa para debug do funcionamento do ESP32
 *
 */
void blink_task(void *pvParameter)
{

    gpio_pad_select_gpio(LED_SEND);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_SEND, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
    	printf("LED LOW \n");
        gpio_set_level(LED_SEND, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        printf("LED HIGH \n");
        gpio_set_level(LED_SEND, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
 *
 * @Brief Button
 *
 */
void gpio_task(void *pvParameter)
{
	gpio_set_direction(LED_SEND, GPIO_MODE_OUTPUT);
	gpio_set_direction(LED_PROG, GPIO_MODE_OUTPUT);
	gpio_set_direction(btnLIGAR, GPIO_MODE_INPUT);
	gpio_set_direction(btnAUMENTAR, GPIO_MODE_INPUT);
	gpio_set_direction(btnDIMINUIR, GPIO_MODE_INPUT);
	gpio_set_level(LED_SEND, 0);
	gpio_set_level(LED_PROG, 0);

	int cLIGAR = 0, cAUMENTAR = 0, cDIMINUIR = 0;

	while(1) {

		//btnLigar
		if(gpio_get_level(btnLIGAR)){
			gpio_set_level(LED_SEND, 0);
			gpio_set_level(LED_PROG, 0);
			cLIGAR = 0;
		}else{
			if(cLIGAR==0){
				RMT_BUTTON_SEND(IR1);
				gpio_set_level(LED_SEND, 1);
				cLIGAR++;
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}else if(cLIGAR>5){
				gpio_set_level(LED_SEND, 0);
				gpio_set_level(LED_PROG, 1);
				RMT_BUTTON_RECEIVE(IR1);
				cLIGAR++;
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}else{
				cLIGAR++;
			}
		}

		//btnDiminuir
		if(gpio_get_level(btnDIMINUIR)){

			if(cDIMINUIR==0){
				gpio_set_level(LED_SEND, 1);
				RMT_BUTTON_SEND(IR2);
				cDIMINUIR++;
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}else if(cDIMINUIR>5){
				gpio_set_level(LED_SEND, 0);
				gpio_set_level(LED_PROG, 1);
				RMT_BUTTON_RECEIVE(IR2);
				cDIMINUIR++;
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}else{
				cDIMINUIR++;
			}


		}else{
			gpio_set_level(LED_SEND, 0);
			gpio_set_level(LED_PROG, 0);
			cDIMINUIR = 0;
		}

		//btnAUMENTAR
		if(gpio_get_level(btnAUMENTAR)){
			if(cAUMENTAR==0){
				gpio_set_level(LED_SEND, 1);
				RMT_BUTTON_SEND(IR3);
				cAUMENTAR++;
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}else if(cAUMENTAR>5){
				gpio_set_level(LED_SEND, 0);
				gpio_set_level(LED_PROG, 1);
				RMT_BUTTON_RECEIVE(IR3);
				cAUMENTAR++;
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}else{
				cAUMENTAR++;
			}
		}else{
			gpio_set_level(LED_SEND, 0);
			gpio_set_level(LED_PROG, 0);
			cAUMENTAR = 0;
		}


		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void app_main()
{
	size_t size = (sizeof(rmt_item32_t) * 120 * 100);
	IR1 = (rmt_item32_t*) malloc(size);
	memset((void*) IR1, 0, size);
	IR2 = (rmt_item32_t*) malloc(size);
	memset((void*) IR2, 0, size);
	IR3 = (rmt_item32_t*) malloc(size);
	memset((void*) IR3, 0, size);

	TX_INICIALIZACAO();
	RX_INICIALIZACAO();

	//Fila para enviar texto para o display
	//xQueue = xQueueCreate(10, sizeof(TEXTO));


	//*****************************
	// RMT
	//*****************************
    //xTaskCreate(blink_task, "blink_task", 2048, NULL, 1, NULL);
    //++++xTaskCreate(RMT_TX_TV, "RMT_TX_TV", 2048, NULL, 1, NULL);
    //xTaskCreate(RMT_RX, "RMT_RX", 2048, xQueue, 1, NULL);
	//#############################
	// Fim RMT
	//#############################


    //*****************************
    // Button
    //*****************************
    xTaskCreate(&gpio_task, "gpio_task", 2048, NULL, 5, NULL);
    //#############################
    // Fim Button
    //#############################

	//*****************************
	// Display OLED - U8G2
	//*****************************
	///xTaskCreate(&display_OLED_u8g2, "u8g2", 2048, NULL, 6, NULL);
	///xTaskCreatePinnedToCore(&display_OLED_u8g2, "u8g2", 2048, NULL, 6, NULL, 0);
	//#############################
	// Fim Display OLED
	//#############################

	//*****************************
	// DHT22
	//*****************************
	///nvs_flash_init();
	///vTaskDelay( 1000 / portTICK_RATE_MS );
	///xTaskCreate( DHT_task, "DHT_task", 2048, NULL, 5, NULL );
	//#############################
	// Fim DHT22
	//#############################

	//*****************************
	//Ler ADC (LDR)
	//*****************************
	///xTaskCreate( ler_ADC, "ler_ADC_task", 2048, NULL, 5, NULL );
	//#############################
	//Fim ADC
	//#############################

	//*****************************
	//Ler Touch Pad
	// Incompatibilidade com a biblioteca u8g2 - 28/11, verificar novas versões no futuro
	//*****************************
	//xTaskCreate( ler_Touch_PAD, "ler_Touch_PAD", 2048, NULL, 2, NULL );
	//xTaskCreatePinnedToCore(&display_OLED_u8g2, "u8g2", 2048, NULL, 6, NULL, 1);
	//#############################
	//Fim Touch Pad
	//#############################


}

