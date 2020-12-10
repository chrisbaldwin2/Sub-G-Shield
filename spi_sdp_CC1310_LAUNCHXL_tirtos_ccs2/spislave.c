/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * spislave.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Chris Baldwin
 */

/*
 *  ======== spislave.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#define BOARD_SHIELD 0
#if BOARD_SHIELD
#include "Board_Shield.h"
#else
#include "Board.h"
#endif

#include "decode.h"

#define THREADSTACKSIZE (1024)

#define MAX_LOOP        (10000)

#define COUNT 0

static Display_Handle display;

unsigned char slaveRxBuffer[SPI_MSG_LENGTH];
unsigned char slaveTxBuffer[SPI_MSG_LENGTH];

/* Semaphore to block slave until transfer is complete */
sem_t slaveSem;

/*
 *  ======== transferCompleteFxn ========
 *  Callback function for SPI_transfer().
 */
void transferCompleteFxn(SPI_Handle handle, SPI_Transaction *transaction)
{
    sem_post(&slaveSem);
}

/*
 * ======== slaveThread ========
 *  Slave SPI sends a message to master while simultaneously receiving a
 *  message from the master.
 */
void *slaveThread(void *arg0)
{
    SPI_Handle      slaveSpi;
    SPI_Params      spiParams;
    SPI_Transaction transaction;
//    uint32_t        i;
    uint32_t        j;
    bool            transferOK;
    int32_t         status;

    /*
     * Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY are GPIO pins connected
     * between the master & slave.  These pins are used to synchronize
     * the master & slave applications via a small 'handshake'.  The pins
     * are later used to synchronize transfers & ensure the master will not
     * start a transfer until the slave is ready.  These pins behave
     * differently between spimaster & spislave examples:
     *
     * spislave example:
     *     * Board_SPI_MASTER_READY is configured as an input pin.  During the
     *       'handshake' this pin is read & a high value will indicate the
     *       master is ready to run the application.  Afterwards, the pin is
     *       read to determine if the master has already opened its SPI pins.
     *       The master will pull this pin low when it has opened its SPI.
     *
     *     * Board_SPI_SLAVE_READY is configured as an output pin.  During the
     *       'handshake' this pin is changed from low to high output.  This
     *       notifies the master the slave is ready to run the application.
     *       Afterwards, the pin is used by the slave to notify the master it
     *       is ready for a transfer.  When ready for a transfer, this pin will
     *       be pulled low.
     *
     * Below we set Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY initial
     * conditions for the 'handshake'.
     */
    

    /*
     * Handshake - Set Board_SPI_SLAVE_READY high to indicate slave is ready
     * to run.  Wait for Board_SPI_MASTER_READY to be high.
     * 
     * --Removed due to input buffering issue. Replaced with ACK structure
     */
//    Display_printf(display, 0, 0, "Initializing SPI Handshake\n");
//    GPIO_write(Board_SPI_SLAVE_READY, 0);
//    while (GPIO_read(Board_SPI_MASTER_READY) == 1) {
//        Display_printf(display, 0, 0, "Waiting on Master: %d %x\r", Board_SPI_MASTER_READY, GPIO_read(Board_SPI_MASTER_READY));
//    }
//    GPIO_write(Board_SPI_SLAVE_READY, 1);

    /*
     * Create synchronization semaphore; this semaphore will block the slave
     * until a transfer is complete.  The slave is configured in callback mode
     * to allow us to configure the SPI transfer & then notify the master the
     * slave is ready.  However, we must still wait for the current transfer
     * to be complete before setting up the next.  Thus, we wait on slaveSem;
     * once the transfer is complete the callback function will unblock the
     * slave.
     */
    Display_printf(display, 0, 0, "Initializing slaveSem\n");
    status = sem_init(&slaveSem, 0, 0);
    if (status != 0) {
        Display_printf(display, 0, 0, "Error creating slaveSem\n");

        while(1);
    }

    /*
     * Wait until master SPI is open.  When the master is configuring SPI pins
     * the clock may toggle from low to high (or high to low depending on
     * polarity).  If using 3-pin SPI & the slave has been opened before the
     * master, clock transitions may cause the slave to shift bits out assuming
     * it is an actual transfer.  We can prevent this behavior by opening the
     * master first & then opening the slave.
     */

    /*
     * Open SPI as slave in callback mode; callback mode is used to allow us to
     * configure the transfer & then set Board_SPI_SLAVE_READY high.
     */

    Display_printf(display, 0, 0, "Initializing slave Spi\n");
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.mode = SPI_SLAVE;
    spiParams.transferCallbackFxn = transferCompleteFxn;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.bitRate=1000000;
    slaveSpi = SPI_open(Board_SPI_SLAVE, &spiParams);
    if (slaveSpi == NULL) {
        Display_printf(display, 0, 0, "Error initializing slave SPI\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "Slave SPI initialized\n");
    }

    init_spi(&slaveSpi, &transaction, (unsigned char *) slaveRxBuffer, (unsigned char *) slaveTxBuffer, &display);
    Display_printf(display, 0, 0, "SPI pointers passed\n");

    unsigned char count = 0;
    decode_struct_t decode_packet = init_decode_struct();

    while(1) {
        /* Initialize slave SPI transaction structure */
        
        memset((void *) slaveRxBuffer, 0, SPI_MSG_LENGTH);

        transaction.rxBuf = (void *) slaveRxBuffer;/* Copy message to transmit buffer */
#if COUNT
        slaveTxBuffer[0] = 0x01;
        slaveTxBuffer[1] = count;
        transaction.count = 2;
#else
        transaction.count = decode_packet.tx_len;
        if(decode_packet.tx_len == 2){
            slaveTxBuffer[0] = decode_packet.tx_buf[0];
            slaveTxBuffer[1] = decode_packet.tx_buf[1];
            Display_printf(display, 0, 0, "Slave to send: Length %d\r\nPacket: %x %x", decode_packet.tx_len, slaveTxBuffer[0], slaveTxBuffer[1]);
        }
        else{
            Display_printf(display, 0, 0, "Slave to send: Length %d\r\nPacket: ", decode_packet.tx_len);
            
            for(j=0; j<decode_packet.tx_len; j++){
                slaveTxBuffer[j] = decode_packet.tx_buf[j];
                Display_printf(display, 0, 0, "%x ", slaveTxBuffer[j]);
            }
            Display_printf(display, 0, 0, "\n");
        }
#endif

        transaction.txBuf = (void *) slaveTxBuffer;
        count++;



        /*
         * Setup SPI transfer; Board_SPI_SLAVE_READY will be set to notify
         * master the slave is ready.
         */
        Display_printf(display, 0, 0, "Initializing SPI transfer");
        transferOK = SPI_transfer(slaveSpi, &transaction);
        if (transferOK) {
            

            /* Wait until transfer has completed */
#if VERBOSE
            Display_printf(display, 0, 0, "Slave ready. Waiting for SPI");
#endif
            sem_wait(&slaveSem);

            /*
             * Drive Board_SPI_SLAVE_READY high to indicate slave is not ready
             * for another transfer yet.
             */

            GPIO_toggle(Board_GPIO_LED0);

#if 1
            if(decode_packet.tx_len != 2){
                Display_printf(display, 0, 0, "Slave received: ");
                for(j=0; j<decode_packet.tx_len; j++){
                    Display_printf(display, 0, 0, "%x ", slaveRxBuffer[j]);
                }
                
            } else {
                Display_printf(display, 0, 0, "Slave received: %x %x", slaveRxBuffer[0], slaveRxBuffer[1]);
            }
#endif

            decode_packet = decode(slaveRxBuffer, decode_packet.tx_len);

#if VERBOSE
            if(decode_packet.tx_len != 2){
                Display_printf(display, 0, 0, "Decode returned: ");
                for(j=0; j<decode_packet.tx_len; j++){
                    Display_printf(display, 0, 0, "%x ", decode_packet.tx_buf[j]);
                }
                
            } else {
                Display_printf(display, 0, 0, "Decode returned: %x %x", decode_packet.tx_buf[0], decode_packet.tx_buf[1]);
            }
#endif

        }
        else {
            Display_printf(display, 0, 0, "Unsuccessful slave SPI transfer");
            SPI_close(slaveSpi);
            sleep(1);
            slaveSpi = SPI_open(Board_SPI_SLAVE, &spiParams);
            if (slaveSpi == NULL) {
                Display_printf(display, 0, 0, "Error initializing slave SPI\n");
                while (1);
            }
            else {
                Display_printf(display, 0, 0, "Slave SPI initialized\n");
            }
            init_spi(&slaveSpi, &transaction, (unsigned char *) slaveRxBuffer, (unsigned char *) slaveTxBuffer, &display);
        }
    }

}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_t           thread1;
    pthread_t           thread2;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();

    /* Configure the LED pins */
    enable_gpio(Board_GPIO_LED0, OUTPUT);
    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    Display_printf(display, 0, 0, "---------Starting the SPI Shield---------");
    Display_printf(display, 0, 0, "Look at the Board.h file for connections");

    /* Create spi thread */
#if 1
    /* Create application thread */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        Display_printf(display, 0, 0, "Spi thread detached state failed\n");
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        Display_printf(display, 0, 0, "Spi thread set stack size failed\n");
        while (1);
    }
    /* Create slave thread */
    priParam.sched_priority = 3;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, slaveThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        Display_printf(display, 0, 0, "Spi thread create failed\n");
        while (1);
    }

#endif
    /* Create rx thread */
#if 1
    init_rf(&display);

    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        Display_printf(display, 0, 0, "Rx thread detached state failed");
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE/2);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        Display_printf(display, 0, 0, "Rx thread set stack size failed\n");
        while (1);
    }

    priParam.sched_priority = 2;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread1, &attrs, rxThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        Display_printf(display, 0, 0, "Rx thread create failed\n");
        while (1);
    }

//    while(true);
#endif
    /* Create tx thread */
#if 1
    init_rf(&display);

    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        Display_printf(display, 0, 0, "Tx thread detached state failed");
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE/2);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        Display_printf(display, 0, 0, "Tx thread set stack size failed\n");
        while (1);
    }

    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread2, &attrs, txThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        Display_printf(display, 0, 0, "Tx thread create failed\n");
        while (1);
    }
#endif

    return (NULL);
}
