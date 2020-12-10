/*
 * decode.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Chris Baldwin
 */

#define VERBOSE 1

#include <stdlib.h>
#include <stdint.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"

#ifndef DECODE_H_
#define DECODE_H_

#define BUF_SIZE 4
#define INDEX_MASK 0xFE
#define INDEX_OFFSET 1
#define GPIO_VALUE_MASK 0x01

#define SUCCESS false
#define FAILURE true
#define INPUT true
#define OUTPUT false

typedef enum {ACK, STATUS, CLEAR_STATUS, SET_GPIO, GET_GPIO, GET_RF, SEND_RF, SEND_RF_BUF, OVERFLOW = 0xFF} op_t;
typedef enum {NONE = 0x0, RESET = 0x1, RF_RECEIVED = 0x2, BUTTON_PRESS = 0x4, ERROR = 0x8} op_status_t;

#define SPI_MSG_LENGTH  (20)
#define SLAVE_ACK       0xA5

#define SET_BUFFER 0xFF

/*
 * Arduino wants to send GPIO/RF--
 *  Makes packet
 *     Opcode = SET_GPIO/SEND_RF
 *     Packet = GPIO [1..0], Packet Buffer [Length..Data]
 *  Writes packet to SPI
 *
 *  Arduino wants data (STATUS, GET_GPIO, GET_RF)
 *   Makes a packet
 *     Opcode = STATUS/GET_GPIO/GET_RF
 *   Writes packet to SPI & expects read data on next transfer after opcode is sent
 *     Issues is the length of RF data is unknown
 *       Head data with length of data?
 *     Needs time to write to the buffer
 *     Point SPI_TX to different buffers? -- Probably not smart
 *   TO-IMPLEMENT::
 *   Writes opcode & ends transfer which tells MCU to set the TX buffer to STATUS, GPIO_VAL, RF_RX_BUF[length..data].
 *   On next transfer, the Arduino will read the length of the data and pull all the data from the buffer.
 *
 *   Arduino sends GET_RF plus a null packet
 *           gets  length 1 plus status
 *
 *   Arduino sends STATUS plus null packets
 *           gets  data length plus data
 *
 *
 */

typedef struct decode_struct{
    unsigned char tx_buf[SPI_MSG_LENGTH];
    unsigned char tx_len;
} decode_struct_t;

decode_struct_t init_decode_struct();

decode_struct_t decode(uint8_t buf[SPI_MSG_LENGTH], uint32_t buf_length);

bool set_status(op_status_t status_p);

bool enable_gpio(uint8_t index, bool i_o);

bool init_spi(SPI_Handle* slaveSpi_p, SPI_Transaction *transaction_p, unsigned char *slaveRxBuffer_p, unsigned char *slaveTxBuffer_p, Display_Handle *display_p);

bool init_rf(Display_Handle *display_p);

void *rxThread(void *arg0);
//void init_tx();

void *txThread(void *arg0);

#endif /* DECODE_H_ */
