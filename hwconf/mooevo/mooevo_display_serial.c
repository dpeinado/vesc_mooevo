/*
	Copyright 2021 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2021 Maximiliano Cordoba	mcordoba@powerdesigns.ca

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "conf_general.h"
#include "hw.h"
#include "mooevo_display_serial.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include <string.h>
//#include "../../applications/app_URBAN_DISPLAY.h"
#include "../../applications/app_0_Mooevo.h"
#include "comm_can.h"
#include "datatypes.h"

#define CMD_HEAD			0x55
#define MOOEVO_DISPLAY_BAUD	9600
#define MAX_TXBUFF 7
#define MAX_RXBUFF 5
#define MOOEVO_RX_BUFFER_SIZE 20
#define MOOEVO_TX_BUFFER_SIZE 28

typedef struct {
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	unsigned char data[MOOEVO_RX_BUFFER_SIZE];
	unsigned char tx[MOOEVO_TX_BUFFER_SIZE];
} mooevo_serial_buffer_t;


//tipo_estado_vehiculo *estado_vehiculo;
DisplayCommParameters *miCommParameters;

static volatile bool mooevo_display_thread_is_running = false;
static volatile bool mooevo_display_uart_is_running = false;

/* UART driver configuration structure */
static SerialConfig uart_cfg = {
                                MOOEVO_DISPLAY_BAUD,
                                0,
                                USART_CR2_LINEN,
                                0
};

static mooevo_serial_buffer_t serial_buffer;

// Threads
static THD_WORKING_AREA(mooevo_display_process_thread_wa, 1024);
static THD_FUNCTION(mooevo_display_process_thread, arg);

static uint8_t mooevo_checksum(uint8_t *buf, uint8_t len);
static void mooevo_serial_send_packet(unsigned char *data, unsigned int len);
static void mooevo_serial_display_byte_process(unsigned char byte);
static void mooevo_serial_display_check_rx(void);

void mooevo_display_serial_start(void) {
  //estado_vehiculo = get_estado_vehiculo();
  miCommParameters = get_display_parameters();
  if (!mooevo_display_thread_is_running) {
      chThdCreateStatic(mooevo_display_process_thread_wa, sizeof(mooevo_display_process_thread_wa),
              NORMALPRIO, mooevo_display_process_thread, NULL);
      mooevo_display_thread_is_running = true;
  }
  serial_buffer.rd_ptr = 0;
  serial_buffer.wr_ptr = 0;

  palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
          PAL_STM32_OSPEED_HIGHEST |
          PAL_STM32_PUDR_PULLUP);
  palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
          PAL_STM32_OSPEED_HIGHEST |
          PAL_STM32_PUDR_PULLUP);

  sdStart(&HW_UART_DEV, &uart_cfg);
  mooevo_display_uart_is_running = true;
}

static uint8_t mooevo_checksum(uint8_t *buf, uint8_t len) {
  uint8_t sum = 0;
  for(int i = 0; i<len; i++) {
      sum += buf[i];
  }
  return sum;
}

static void mooevo_serial_send_packet(unsigned char *data, unsigned int len) {
	if (mooevo_display_uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static void mooevo_serial_display_byte_process(unsigned char byte) {
	//append new byte to the buffer.
	serial_buffer.data[serial_buffer.wr_ptr] = byte;
	serial_buffer.wr_ptr++;

	uint8_t rd_ptr = serial_buffer.rd_ptr;	//read pointer to try at different start addresses
	// process with at least 2 bytes available to read
	while( (serial_buffer.wr_ptr - rd_ptr ) > 1) {
      if(serial_buffer.data[rd_ptr] == CMD_HEAD) {
          // start byte found
        if( (serial_buffer.wr_ptr - rd_ptr) < MAX_RXBUFF)
          return;
        uint8_t checksum_addr;
        checksum_addr = rd_ptr + MAX_RXBUFF - 1;
        if(checksum_addr <= serial_buffer.wr_ptr) {	//check the checksum has been received
          // check sum
          if( serial_buffer.data[checksum_addr] == mooevo_checksum(serial_buffer.data + rd_ptr + 1, 3) ) {
        	miCommParameters->modo = (serial_buffer.data[rd_ptr+1] >> 4) & 0xFF;
        	miCommParameters->reversa = (serial_buffer.data[rd_ptr+3] & 0x01);
            serial_buffer.rd_ptr = rd_ptr + 4;	//mark bytes as read
            // enviar paquete al display con info de miCommParameters
            serial_buffer.tx[0] = CMD_HEAD;
            serial_buffer.tx[1] = 0x00; // error de momento no env�o errores
            serial_buffer.tx[2] = miCommParameters->velocidad & 0xFF;
            serial_buffer.tx[3] = miCommParameters->velocidad >> 8;
            serial_buffer.tx[4] = 100;
            serial_buffer.tx[5] = miCommParameters->intensidad*68.0/120.0;
            serial_buffer.tx[6] = mooevo_checksum(serial_buffer.tx+1, MAX_TXBUFF-2);
            mooevo_serial_send_packet(serial_buffer.tx, MAX_TXBUFF);
          }
        }
      }
      rd_ptr++;
	}
	if(serial_buffer.rd_ptr > 0) {
		memmove(serial_buffer.data, serial_buffer.data + serial_buffer.rd_ptr, MOOEVO_RX_BUFFER_SIZE - serial_buffer.rd_ptr);
		serial_buffer.wr_ptr -= serial_buffer.rd_ptr;
		serial_buffer.rd_ptr = 0;
	}
	if(serial_buffer.wr_ptr == (MOOEVO_RX_BUFFER_SIZE - 1) ) {
		//shift buffer to the left discarding the oldest byte
		memmove(serial_buffer.data,serial_buffer.data + 1, MOOEVO_RX_BUFFER_SIZE - 1);
		serial_buffer.wr_ptr -= 1;
	}
}

static void mooevo_serial_display_check_rx(void){
  bool rx = true;
  while (rx) {
    rx = false;
    if (mooevo_display_uart_is_running) {
      msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_INFINITE);
      if (res != MSG_TIMEOUT) {
        mooevo_serial_display_byte_process(res);
        rx = true;
      }
    }
  }
}

static THD_FUNCTION(mooevo_display_process_thread, arg) {
  (void)arg;
  chRegSetThreadName("Mooevo serial display");

  chThdSleepMilliseconds(500);
  for(;;) {
    chThdSleepMilliseconds(100);
    mooevo_serial_display_check_rx();
  }
}
