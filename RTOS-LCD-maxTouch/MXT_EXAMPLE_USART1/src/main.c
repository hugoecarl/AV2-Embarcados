/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "termometro.h"
#include "ar.h"
#include "soneca.h"
#include "tfont.h"
#include "digital521.h"


#define YEAR        2019
#define MOUNTH      1
#define DAY         1
#define WEEK        1
#define HOUR        13
#define MINUTE      43
#define SECOND      34


//butao 2 oled
#define EBUT1_PIO PIOA //pause  Ext 4 PA19 PA = 10
#define EBUT1_PIO_ID 10
#define EBUT1_PIO_IDX 19
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)
//butao 3 oled
#define EBUT2_PIO PIOC //sei la EXT 3 PC31
#define EBUT2_PIO_ID 12 // piod ID
#define EBUT2_PIO_IDX 31
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)

#define PIO_PWM_0 PIOA
#define ID_PIO_PWM_0 ID_PIOA
#define MASK_PIN_PWM_0 (1 << 0)

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;
uint duty = 0;


/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)
#define AFEC_CHANNEL_POT_SENSOR 0
volatile uint32_t pot_ul_value;
volatile bool g1_is_conversion_done = false;


/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;
uint32_t hour, minuto,seg;
	
/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)  

#define TASK_LCD_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_SEM_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_SEM_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_SLE_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_SLE_STACK_PRIORITY        (tskIDLE_PRIORITY)



typedef struct {
  uint x;
  uint y;
} touchData;

QueueHandle_t xQueueTouch;
/** Semaforo a ser usado pela task led */
SemaphoreHandle_t xSemaphorepwmdown;
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphoresleep;
SemaphoreHandle_t xSemaphorepwmup;




void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}



void draw_tela_inicial_logos(tImage ter, tImage ar, tImage son){
	
	ili9488_draw_pixmap(40,
	380,
	54,
	79,
	ter.data);
	
	ili9488_draw_pixmap(20,
	300,
	88,
	70,
	ar.data);
	
	ili9488_draw_pixmap(230,
	20,
	69,
	69,
	son.data);

	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(350, 290, 0, 280);
	

}

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */

static void AFEC_pot_callback(void)                     
{
	pot_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static void but_p_freq_callback(void){
	printf("AEE\n");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphorepwmdown, &xHigherPriorityTaskWoken);
}
static void but_m_freq_callback(void){
	printf("AOO\n");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphorepwmup, &xHigherPriorityTaskWoken);
}




extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/


static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}

void io_init(void)
{
	// configura botoes do oled
	pmc_enable_periph_clk(EBUT1_PIO_ID);
	pmc_enable_periph_clk(EBUT2_PIO_ID);
	
	// configura botoes do oled como input;
	pio_configure(EBUT1_PIO, PIO_INPUT, EBUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(EBUT2_PIO, PIO_INPUT, EBUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(EBUT1_PIO,
	EBUT1_PIO_ID,
	EBUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_p_freq_callback);
	
	pio_handler_set(EBUT2_PIO,
	EBUT2_PIO_ID,
	EBUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_m_freq_callback);
	
	// Ativa interrup??o
	pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
	pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)

	NVIC_EnableIRQ(EBUT1_PIO_ID);
	NVIC_SetPriority(EBUT1_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(EBUT2_PIO_ID);
	NVIC_SetPriority(EBUT2_PIO_ID, 4); // Prioridade 4
}


void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	draw_tela_inicial_logos(termometro, ar, soneca);
}


uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
	if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
		if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
			//draw_button(1);
		} else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
			//draw_button(0);
		}
	}
}




void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
	/* USART tx buffer initialized to 0 */
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;
  
  /* first touch only */
  uint first = 0;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {

		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = convert_axis_system_x(touch_event.y);
      *y = convert_axis_system_y(touch_event.x);
      first = 1;
    }
    
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  	struct mxt_device device; /* Device data container */
  	mxt_init(&device);       	/* Initialize the mXT touch device */
    touchData touch;          /* touch queue data type*/
    
  	while (true) {  
		  /* Check for any pending messages and run message handler if any
		   * message is found in the queue */
		  
		  if (mxt_is_message_pending(&device)) {
		  	mxt_handler(&device, &touch.x, &touch.y);
        xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      }
     vTaskDelay(100);
	}
}


void task_semaf(void){
	
	
	const TickType_t xDelay = 4000 / portTICK_PERIOD_MS;
	
	while(1){
		printf("Inicio convers�o \n");
		afec_start_software_conversion(AFEC0);
		vTaskDelay(xDelay);
	}
	
}


void task_lcd(void){	
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphorepwmup = xSemaphoreCreateBinary();
	xSemaphorepwmdown = xSemaphoreCreateBinary();
	
	
	configure_lcd();
	draw_screen();
	touchData touch;
	io_init();
	
	char hnum[3];
	char hora[3];
	char minu[3];
	char dutystr[3];
	
  
	if (xSemaphore == NULL)
	printf("falha em criar o semaforo \n");
  
  while (true) {  
	 printf("Entrou \n");
	 
	 rtc_get_time(RTC, &hour, &minuto, &seg);
	 ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	 ili9488_draw_filled_rectangle(90, 200, 230, 150);
	 ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	 
	 itoa(hour, hora, 10);
	 font_draw_text(&digital52, hora, 100, 150, 1);
	 itoa(minuto, minu, 10);
	 font_draw_text(&digital52, minu, 180, 150, 1);
	 font_draw_text(&digital52, ":", 150, 150, 1);
	
	 
	 if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
		 //int temp = 100*pot_ul_value/4095;
		 int temp = 100*pot_ul_value/2870;
		 printf("Temperatura %d\n", temp);
		 ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		 ili9488_draw_filled_rectangle(120,500,250,400);
		 itoa(temp, hnum, 10);
		 font_draw_text(&digital52, hnum, 130, 400, 1);	 
	 }
	 
	 	if ( xSemaphoreTake(xSemaphorepwmup, ( TickType_t ) 500) == pdTRUE){
		 	
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		 	ili9488_draw_filled_rectangle(120,370,300,295);
		 	if (duty < 100){
				duty+=10;
			 }
		 	
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 100-duty);
		 	itoa(duty, dutystr, 10);
		 	font_draw_text(&digital52, dutystr, 150, 310, 1);
			 font_draw_text(&digital52, "%", 230, 310, 1);

	 	}
	 	if ( xSemaphoreTake(xSemaphorepwmdown, ( TickType_t ) 500) == pdTRUE){
		 	
			 ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		 	ili9488_draw_filled_rectangle(120,370,300,295);
		 	if (duty > 0){
			 	duty-=10;
		 	}
		 	
			 pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 100-duty);
		 	itoa(duty, dutystr, 10);
		 	font_draw_text(&digital52, dutystr, 150, 310, 1);
			 font_draw_text(&digital52, "%", 230, 310, 1);

	 	}
	 
	 if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
       update_screen(touch.x, touch.y);
       printf("x:%d y:%d\n", touch.x, touch.y);
	   if(touch.x >= 218 && touch.x <= 320) {
		  if(touch.y >= 0 && touch.y <= 82) {
			  printf("Botaooooo\n");
			  font_draw_text(&digital52, "sleep", 1, 50, 1);
			  xSemaphoreGiveFromISR(xSemaphoresleep, NULL);
			  
		  }
	 }
	 }     
  }	 
}

void task_sleep(void){
	xSemaphoresleep = xSemaphoreCreateBinary();
	const TickType_t xDelay = 30000 / portTICK_PERIOD_MS;
	int i = 0;
	
	while(1){
		if( xSemaphoreTake(xSemaphoresleep, ( TickType_t ) 500) == pdTRUE && i < 1){
		i++;
		vTaskDelay(xDelay);
		font_draw_text(&digital52, "off", 1, 1, 1);
		}
	
	}
	
}

static void config_POT(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_pot_callback, 5);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/
	//struct afec_temp_sensor_config afec_pot_sensor_cfg;

	//afec_temp_sensor_get_config_defaults(&afec_pot_sensor_cfg);
	//afec_temp_sensor_set_config(AFEC0, &afec_pot_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}

// Inicializa botao SW0 do kit com interrupcao





/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};
	
	
	

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	
	
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	
	RTC_init();
	config_POT();
	PWM0_init(0, duty);
	
	pmc_enable_periph_clk(ID_PIO_PWM_0);
	pio_set_peripheral(PIO_PWM_0, PIO_PERIPH_A, MASK_PIN_PWM_0 );
	  

		
  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
	    /* Create task to handler LCD */
  if (xTaskCreate(task_semaf, "semaf", TASK_SEM_STACK_SIZE, NULL, TASK_SEM_STACK_PRIORITY, NULL) != pdPASS) {
	printf("Failed to create test semaf task\r\n");
	    }
		
	  if (xTaskCreate(task_sleep, "sleep", TASK_SLE_STACK_SIZE, NULL, TASK_SLE_STACK_PRIORITY, NULL) != pdPASS) {
		  printf("Failed to create test SLEEP task\r\n");
	  }



  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){
	
	
  }


	return 0;
}
