/*
 * decode.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Chris Baldwin
 */

#include "decode.h"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <ti/devices/cc13x0/driverlib/i2c.h>

#include "RFQueue.h"
#include <ti/drivers/rf/RF.h>
#include <unistd.h>
#include "smartrf_settings/smartrf_settings.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

#include <semaphore.h>

/* Example/Board Header files */
#include "Board.h"

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      12
#define PACKET_INTERVAL     500000  /* Set packet interval to 500000us or 500ms */

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];

SPI_Handle      slaveSpi;
SPI_Transaction *transaction;

unsigned char *slaveRxBuffer;
unsigned char *slaveTxBuffer;
unsigned char gpio_read = 0;
unsigned char status = NONE;

static Display_Handle display;

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

sem_t txSem;
sem_t rxSem;

static uint8_t tx_packet[PAYLOAD_LENGTH];
static uint8_t rx_packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1];
uint32_t rx_length = 0;

/* Callback function for RX */
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/* Sends the data in tx_packet array */
void send_tx(uint8_t packet_length);

decode_struct_t init_decode_struct(){
    decode_struct_t decode_ret;
#if 0
    int i;
    for(i=0; i<SPI_MSG_LENGTH; i++){
        decode_ret.tx_buf[i] = 0;
    }
#endif
    memset((void *) decode_ret.tx_buf, 0, SPI_MSG_LENGTH);
    decode_ret.tx_len = 2;
    return decode_ret;
}

decode_struct_t decode(uint8_t buf[], uint32_t buf_length){
    decode_struct_t decode_ret = init_decode_struct();
    switch(buf[0]){

        case ACK: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: ACK\n");
#endif
//            set_status(NONE);
            decode_ret.tx_buf[0] = 0x01;
            decode_ret.tx_buf[1] = SLAVE_ACK;
            decode_ret.tx_len = 2;
            return decode_ret;
        }
        case STATUS: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: STATUS\n");
#endif
            decode_ret.tx_buf[0] = 0x01;
            decode_ret.tx_buf[1] = status;
            decode_ret.tx_len = 2;
            return decode_ret;
        }
        case CLEAR_STATUS: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: STATUS\n");
#endif
            decode_ret.tx_buf[0] = 0x01;
            decode_ret.tx_buf[1] = SLAVE_ACK;
            decode_ret.tx_len = 2;
            return decode_ret;
         }
        case SET_GPIO: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: SET_GPIO\n");
#endif
            uint8_t index;
            uint8_t value;
            index = (buf[1] & INDEX_MASK) >> INDEX_OFFSET;
            value = buf[1] & GPIO_VALUE_MASK;
            enable_gpio(index, OUTPUT);
            GPIO_write(index, value);
            return decode_ret;
        }
        case GET_GPIO: {
#if VERBOSE
            /* Get the GPIO Value */
            Display_printf(display, 0, 0, "Received opcode: GET_GPIO\n");
#endif
            gpio_read = GPIO_read( (uint_fast8_t) buf[1] );
            /* Set SPI transaction to status buffer */
            decode_ret.tx_buf[0] = 0x01;
            decode_ret.tx_buf[1] = gpio_read;
            decode_ret.tx_len = 2;
            return decode_ret;
        }
        case GET_RF: {
            int i;
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: GET_RF\n");
#if 1
            Display_printf(display, 0, 0, "Rx Length %d\r\nRx Packet", rx_length);
            for(i=0; i<rx_length; i++){
                Display_printf(display, 0, 0, "%x", rx_packet[i]);
            }
#endif
            Display_printf(display, 0, 0, "Moving to Buf");
#endif
            for(i=0; (i<rx_length) && (i<SPI_MSG_LENGTH-1) && (status & RF_RECEIVED); i++){
                decode_ret.tx_buf[i+1] = rx_packet[i];
#if VERBOSE
            Display_printf(display, 0, 0, "%x", rx_packet[i]);
#endif
            }
            if(rx_length > SPI_MSG_LENGTH-1){
                decode_ret.tx_len = SPI_MSG_LENGTH;
                decode_ret.tx_buf[0] = SPI_MSG_LENGTH-1;
#if VERBOSE
            Display_printf(display, 0, 0, "Length greater than SPI %d", decode_ret.tx_len);
#endif
            } else if( (rx_length) && (status & RF_RECEIVED)){
                decode_ret.tx_len = rx_length+1;
                decode_ret.tx_buf[0] = rx_length;
#if VERBOSE
            Display_printf(display, 0, 0, "Length less than SPI %d", decode_ret.tx_len);
#endif
            } else {
                decode_ret.tx_buf[0] = 0;
                decode_ret.tx_buf[1] = 0;
                decode_ret.tx_len = 2;
#if VERBOSE
            Display_printf(display, 0, 0, "Length zero %d", rx_length);
#endif
            }

            status = status & ~RF_RECEIVED;
            return decode_ret;
        }
        case SEND_RF: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: SEND_RF\n");
#endif
            /* Copy packet from buf */
//            decode_ret.tx_len = buf[1] > PAYLOAD_LENGTH ? PAYLOAD_LENGTH : buf[1] > buf_length ? buf_length-1 : buf[1];
            decode_ret.tx_len = buf[1] + 1;
            return decode_ret;
        }
        case SEND_RF_BUF: {
#if VERBOSE
            Display_printf(display, 0, 0, "Received opcode: SEND_RF_BUF\n");
#endif
            memcpy(tx_packet, buf+1, buf_length-1);
#if VERBOSE
            Display_printf(display, 0, 0, "Copy Complete; Entering send_tx\n");

#endif
            send_tx(buf_length-1);
            status = status & ~RF_RECEIVED;
            decode_ret.tx_len = 2;
            decode_ret.tx_buf[0] = 0x01;
            decode_ret.tx_buf[1] = SLAVE_ACK;
            return decode_ret;
        }
        case OVERFLOW: {
#if VERBOSE
            Display_printf(display, 0, 0, "Overflow Event\n");
#endif
            decode_ret.tx_len = 3;
            return decode_ret;
        }
        default: {
            Display_printf(display, 0, 0, "Received opcode: UNKNOWN\n");
            status = ERROR;
            return decode_ret;
        }
    }

}

bool set_status(op_status_t status_p){
    status = status_p;
    return SUCCESS;
}

uint32_t gpio_enable_mask = 0;
uint32_t gpio_exclude_mask = (1 << Board_SPI_MISO) | (1 << Board_SPI_MOSI) |
                             (1 << Board_SPI_CLK) | (1 << Board_SPI_SS) |
                             (1 << Board_UART_RX) | (1 << Board_UART_TX) |
                             (1 << Board_UART_CTS) | (1 << Board_UART_RTS);


bool enable_gpio(uint8_t index, bool i_o){
    if( gpio_exclude_mask & (1 << index) ){
        return FAILURE;
    }
    if( gpio_enable_mask & (1 << index) ){
        return SUCCESS;
    }
    if(i_o == INPUT){
        GPIO_setConfig(index, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_NONE);
        return SUCCESS;
    } else {
        GPIO_setConfig(index, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_HIGH);
        return SUCCESS;
    }

}

bool init_spi(SPI_Handle* slaveSpi_p, SPI_Transaction *transaction_p, unsigned char *slaveRxBuffer_p, unsigned char *slaveTxBuffer_p, Display_Handle *display_p){
    slaveSpi = *slaveSpi_p;
    transaction = transaction_p;
    slaveRxBuffer = slaveRxBuffer_p;
    slaveTxBuffer = slaveTxBuffer_p;
    display = *display_p;
    return SUCCESS;
}

bool init_rf(Display_Handle *display_p){
    display = *display_p;
    return SUCCESS;
}

void *txThread(void *arg0)
{
#if VERBOSE
    Display_printf(display, 0, 0, "Initializing RF Tx\n");
#endif
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = (uint8_t*) tx_packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    Display_printf(display, 0, 0, "Initializing txSem\n");
    status = sem_init(&txSem, 0, 0);
    if (status != 0) {
        Display_printf(display, 0, 0, "Error creating txSem\n");

        while(1);
    }

    while(1){

        sem_wait(&txSem);
#if VERBOSE
        Display_printf(display, 0, 0, "Cancelling Rx command\n");
#endif
        RF_Stat cancel_status = RF_cancelCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 1);
#if VERBOSE
        Display_printf(display, 0, 0, "Rx Cancel Status %d\n", cancel_status);
#endif

#if 0
#if VERBOSE
        Display_printf(display, 0, 0, "Closing RF\n");
#endif
        RF_close(rfHandle);
#if VERBOSE
        Display_printf(display, 0, 0, "Reopening RF\n");
#endif
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

#if VERBOSE
        Display_printf(display, 0, 0, "Setting Frequency\n");
#endif
        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
#endif


        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

#if VERBOSE
        Display_printf(display, 0, 0, "Running Tx command\n");
#endif
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);
#if VERBOSE
        Display_printf(display, 0, 0, "terminationReason %x\n", terminationReason);
#endif
        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

#if VERBOSE
        Display_printf(display, 0, 0, "Getting Tx command status\n");
#endif
        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;

#if VERBOSE
        Display_printf(display, 0, 0, "cmdStatus %x\n", cmdStatus);
#endif
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

    sem_post(&rxSem);
    }

}

void *rxThread(void *arg0)
{
#if VERBOSE
    Display_printf(display, 0, 0, "Initializing RF Rx\n");
#endif
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
#if VERBOSE
        Display_printf(display, 0, 0, "\n----------Failed to allocate RFQueue----------\n");
#endif
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

#if VERBOSE
    Display_printf(display, 0, 0, "Opening RF\n");
#endif

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

#if VERBOSE
    Display_printf(display, 0, 0, "Running Frequency Set command\n");
#endif


    Display_printf(display, 0, 0, "Initializing rxSem\n");
    status = sem_init(&rxSem, 0, 0);
    if (status != 0) {
        Display_printf(display, 0, 0, "Error creating rxSem\n");

        while(1);
    }

    while(1){

#if VERBOSE
        Display_printf(display, 0, 0, "Running Rx command\n");
#endif
        /* Enter RX mode and stay forever in RX */
#if 1

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                   RF_PriorityNormal, &callback,
                                                   RF_EventRxEntryDone);

#if VERBOSE
        Display_printf(display, 0, 0, "terminationReason %x\n", terminationReason);
#endif
        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
                // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
    //            while(1);
                break;
        }

#if VERBOSE
        Display_printf(display, 0, 0, "Getting Rx command status\n");
#endif
        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;

#if VERBOSE
        Display_printf(display, 0, 0, "cmdStatus %x\n", cmdStatus);
#endif

        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet received with CRC OK
                break;
            case PROP_DONE_RXERR:
                // Packet received with CRC error
                break;
            case PROP_DONE_RXTIMEOUT:
                // Observed end trigger while in sync search
                break;
            case PROP_DONE_BREAK:
                // Observed end trigger while receiving packet when the command is
                // configured with endType set to 1
                break;
            case PROP_DONE_ENDED:
                // Received packet after having observed the end trigger; if the
                // command is configured with endType set to 0, the end trigger
                // will not terminate an ongoing reception
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP after command started and, if sync found,
                // packet is received
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT after command started
                break;
            case PROP_ERROR_RXBUF:
                // No RX buffer large enough for the received data available at
                // the start of a packet
                break;
            case PROP_ERROR_RXFULL:
                // Out of RX buffer space during reception in a partial read
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_RXOVF:
                // RX overflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
    //            while(1);
                break;
        }

#else
        RF_CmdHandle Rx_Cmd_Handle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, RF_EventRxEntryDone);
#endif

        sem_wait(&rxSem);
    }
}

void send_tx(uint8_t packet_length){

#if VERBOSE
    Display_printf(display, 0, 0, "Assigning cmdPropTx to proper value\n");
#endif
    RF_cmdPropTx.pktLen = packet_length;
    RF_cmdPropTx.pPkt = (uint8_t*) tx_packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

#if 1
    int j;
    Display_printf(display, 0, 0, "Shield sending RF: Length %x\r\nPacket: ", packet_length);
    for(j=0; j<packet_length; j++){
        Display_printf(display, 0, 0, "%x ", tx_packet[j]);
    }
#endif

#if VERBOSE
    Display_printf(display, 0, 0, "Running Tx command\n");
#endif
    /* Send packet */
    sem_post(&txSem);

#if VERBOSE
    Display_printf(display, 0, 0, "Finished send_tx\n");
#endif
}


void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {

#if VERBOSE
        Display_printf(display, 0, 0, "Starting RF rx callback\n");
#endif
        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        GPIO_toggle(Board_GPIO_LED1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(rx_packet, packetDataPointer, (packetLength + 1));
        rx_length = packetLength;
        status = RF_RECEIVED;
        RFQueue_nextEntry();
    }
}
