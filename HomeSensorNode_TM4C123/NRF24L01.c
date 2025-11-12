/**
 * main.c
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "spi0.h"
#include "wait.h"
#include "NRF24L01.h"

#define GREEN_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define GREEN_LED_MASK 8

char string[100];
//uint8_t address[5] = {0xA0, 0xB0, 0xA0, 0xB0, 0xA0};
uint8_t address[5] = {0x52, 0x78, 0x41, 0x41, 0x41};
nrf24lo1 sndPacket;

void blinkLed(int times)
{
    int i;
    for (i = 0; i < times; i++)
    {
        GREEN_LED = 1;  // Toggle PF1 (Red LED)
        waitMicrosecond(100000);
        GREEN_LED = 0;
        waitMicrosecond(100000);
    }
}

uint8_t readRegister(uint8_t reg)
{
    uint8_t result;

    setPinValue(CS, 0); // Begin SPI transaction

    writeSpi0Data(R_REGISTER | (reg & 0x1F)); // Send register read command
    while (SSI0_SR_R & SSI_SR_BSY);          // Wait for SPI
    readSpi0Data();                          // Dummy read (status)

    writeSpi0Data(NOP);                      // Send NOP to receive register value
    while (SSI0_SR_R & SSI_SR_BSY);          // Wait for SPI
    result = readSpi0Data();                 // Actual register value

    setPinValue(CS, 1);   // End SPI transaction

    return result;
}

void writeRegisterMulti(uint8_t reg, uint8_t* data, uint8_t len)
{
    uint8_t i;
    setPinValue(CS, 0);
    writeSpi0Data(W_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++)
        writeSpi0Data(data[i]);
    setPinValue(CS, 1);
}

void writeRegister(uint8_t reg, uint8_t value)
{
    setPinValue(CS, 0);
    writeSpi0Data(W_REGISTER | (reg & 0x1F));
    writeSpi0Data(value);
    setPinValue(CS, 1);
}

void writePayload(uint8_t* data, uint8_t len)
{
    int i;
    setPinValue(CS, 0);
    writeSpi0Data(W_TX_PAYLOAD);
    for (i = 0; i < len; i++)
        writeSpi0Data(data[i]);
    setPinValue(CS, 1);
}

uint8_t crc8_ccitt(const uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t j;
    uint8_t crc = 0x00;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    //return crc;
    return 0xAA;
}

void readPayload(uint8_t* buffer, uint8_t len)
{
    int i;
    setPinValue(CS, 0);
    writeSpi0Data(0x61);  // R_RX_PAYLOAD
    while (SSI0_SR_R & SSI_SR_BSY);
    for (i = 0; i < len; i++)
    {
        writeSpi0Data(0);
        while (SSI0_SR_R & SSI_SR_BSY);
        buffer[i] = readSpi0Data();
    }
    setPinValue(CS, 1);
}

void flushRx()
{
    setPinValue(CS, 0);
    writeSpi0Data(0xE2);  // FLUSH_TX command
    setPinValue(CS, 1);
}

void enableReceiver()
{
    setPinValue(CE, 0);
    writeRegister(CONFIG, 0x03);   // PWR_UP + PRIM_RX=1 => RX mode
    writeRegister(STATUS, 0x70);
    waitMicrosecond(150);         // ~130us startup for RX mode
    setPinValue(CE, 1);           // CE high to start listening
}

void nrf24_init()
{
    writeRegisterMulti(TX_ADDR, address, 5);
    writeRegisterMulti(RX_ADDR_P0, address, 5);
    //setPinValue(CE, 0);
    //writeRegister(CONFIG, 0x0E);        // Power up, CRC on, PRIM_RX=0 (TX)
    writeRegister(CONFIG, 0x0E);
    //writeRegister(EN_AA, 0x01);         // Auto ack on pipe 0
    writeRegister(EN_AA, 0x00);         // Auto ack on pipe 0
    writeRegister(EN_RXADDR, 0x01);     // Enable data pipe 0
    //writeRegister(SETUP_RETR, 0x2F);    // 750us delay, 15 retries
    writeRegister(SETUP_RETR, 0x00);
    //writeRegister(RF_CH, 10);           // Channel 10
    writeRegister(RF_CH, 76);           // Channel 76
    writeRegister(RF_SETUP, 0x06);      // 1 Mbps, 0 dBm
    //writeRegister(RF_SETUP, 0x27);
    writeRegister(STATUS, 0x70);        // Clear IRQs
    //writeRegister(STATUS, 0x40);        // Clear IRQs
    writeRegister(RX_PW_P0, 32);        // Fixed payload size
    writeRegister(DYNPD, 0x00);  // Disable dynamic payloads
    writeRegister(FEATURE, 0x00); // Disable features (like ACKs) just in case

    // Set TX and RX address for pipe 0
    //writeRegisterMulti(TX_ADDR, address, 5);
    //writeRegisterMulti(RX_ADDR_P0, address, 5);

    waitMicrosecond(1500);
    //setPinValue(CE, 1);
}

void enableTransmitter()
{
    setPinValue(CE, 0);
    writeRegister(CONFIG, 0x0E);  // PWR_UP + PRIM_RX=0
    //writeRegister(STATUS, 0x40);  // Clear IRQs
    waitMicrosecond(150);
}

void flushTx()
{
    setPinValue(CS, 0);
    writeSpi0Data(0xE1);  // FLUSH_TX command
    setPinValue(CS, 1);
}

void sendPacket(nrf24lo1* pkt)
{
    //uint8_t i;
    //pkt->checksum = computeChecksum(pkt);
    //printHex8(pkt->checksum);
    pkt->data[28-2] = 0; // reserved byte 0
    pkt->data[28-1] = 0; // reserved byte 1

    // Set checksum field temporarily to 0
    pkt->checksum = 0;
    // Calculate CRC over entire 32 bytes
    pkt->checksum = crc8_ccitt((uint8_t*)pkt, 32);
    //printHex8(pkt->checksum);
    writeRegister(STATUS, 0x70);  // Clear TX_DS, MAX_RT, RX_DR
    //writeRegister(STATUS, 0x40);
    //flushTx();
    //enableTransmitter();
    flushTx(); // Always flush before send
    //writeRegister(STATUS, 0x70); // Clear status flags

    //writeRegister(CONFIG, 0x02); // Force TX mode (PRIM_RX=0), PWR_UP
    //writeRegister(CONFIG, 0x0E);
    waitMicrosecond(2000);      // Wait for power-up

    setPinValue(CE, 0);
    //writeRegister(STATUS, 0x70);  // Clear TX_DS, MAX_RT, RX_DR
    writePayload((uint8_t*)pkt, 32);
    setPinValue(CE, 1);

    waitMicrosecond(15);

    setPinValue(CE, 0);          // Must go low after TX
    waitMicrosecond(130);        // Allow some time for TX to complete

    //memcpy(&lastSentPacket, pkt, sizeof(nrf24lo1));  // Save last sent packet
}

uint8_t computeChecksum(nrf24lo1* pkt)
{
    uint8_t i;
    uint8_t checksum = 0;
    checksum += pkt->type;
    checksum += pkt->dataLength;
    for (i = 0; i < pkt->dataLength; i++)
        checksum += pkt->data[i];
    return checksum;
}

void serverJoin(void)
{
    int i;
    //enableReceiver();
    if (readRegister(STATUS) & (1 << 6))  // RX_DR
    {
        writeRegister(STATUS, (1 << 6)); // Clear RX_DR flag

        uint8_t rxBuf[32];
        readPayload(rxBuf, 32);

        nrf24lo1* pkt = (nrf24lo1*)(rxBuf+7);

        //enableTransmitter();

        putsUart0("Received Packet:\n");
        putsUart0("Frame ID: ");
        //printHex8(pkt->frame_id);
        putsUart0("\nType: ");
        //printHex8(pkt->type);
        putsUart0("\nData Length: ");
        //printHex8(pkt->dataLength);
        putsUart0("\n");

        for (i = 0; i < pkt->dataLength; i++)
        {
            putsUart0("Data[");
            //printHex8(i);
            putsUart0("]: ");
            //printHex8(pkt->data[i]);
            putsUart0("\n");
        }
        //enableTransmitter();

       if(pkt->type == 0x04) {
           enableTransmitter();
           putsUart0("SYNC packet received!\n");
           sndPacket.frame_id = 0x00;
           sndPacket.type = 0x02;
           sndPacket.dataLength = 0x01;
           sndPacket.data[0] = 0x81;
           sndPacket.checksum =  crc8_ccitt((uint8_t*)&sndPacket, 32);
           sendPacket(&sndPacket);
           putsUart0("Sending JOIN_REQUEST\n");
       }
       else {
           putsUart0("Waiting for SYNC packet\n");
       }

      // waitMicrosecond(2000000);
       enableReceiver();

       if(pkt->type == 0x03) {
       putsUart0("JOIN_RESPONSE received!\n");
       putsUart0("Frame ID: ");
       //printHex8(pkt->frame_id);
       putsUart0("\nType: ");
       //printHex8(pkt->type);
       putsUart0("\nData Length: ");
       //printHex8(pkt->dataLength);
       putsUart0("\n");

       for (i = 0; i < pkt->dataLength; i++)
       {
           putsUart0("Data[");
           //printHex8(i);
           putsUart0("]: ");
           //printHex8(pkt->data[i]);
           putsUart0("\n");
       }
      }
       enableTransmitter();
       //writeRegister(STATUS, (1 << 6)); // Clear RX_DR flag
       sndPacket.frame_id = pkt->frame_id;
       sndPacket.type = 0x0B;
       sndPacket.dataLength = 0x01;
       sndPacket.data[0] = 0x54;
       sndPacket.checksum =  crc8_ccitt((uint8_t*)&sndPacket, 32);
       sendPacket(&sndPacket);

    }
}


void GPIOPortB_Handler(void)
{
    GPIO_PORTB_ICR_R = IRQ_MASK;  // Clear PB2 interrupt flag

    uint8_t i;
    uint8_t status = readRegister(STATUS);
    putsUart0("Status: ");
    //printHex8(status);

    if (status & (1 << 6)) // RX_DR: Packet received
    {
        putsUart0("IRQ: Packet Received\n");

        uint8_t buffer[32];
        readPayload(buffer, 32);
        nrf24lo1 *recPacket = (nrf24lo1*)(buffer + 7);

        uint8_t computedChecksum = 0;
       for (i = 1; i < recPacket->dataLength + 4; i++) {
           computedChecksum += ((uint8_t*)recPacket)[i];
       }

       if (computedChecksum != recPacket->checksum)
       {
           putsUart0("Invalid message\n");
           return;
       }
       else
       {
         if(recPacket->type == 0x04) {
           putsUart0("SYNC packet received\n");
           putsUart0("Received Packet Info:\n ");
           putsUart0("Frame ID: ");
           //printHex8(recPacket->frame_id);
           putsUart0("Type: ");
           //printHex8(recPacket->type);
           putsUart0("Data Length: ");
           //printHex8(recPacket->dataLength);
           for(i = 0; i < recPacket->dataLength; i++) {
            putsUart0("Data: ");
            //printHex8(recPacket->data[i]);
           }
           //enableTransmitter();
         }
           //nrf24lo1 sndPkt;
           //sndPkt.type = 0x02;
           //sndPkt.frame_id = pkt->frame_id;
           //sndPkt.dataLength = 1;
           //sndPkt.data[0] = 0x02;
           //sendPacket(&sndPkt);
       }

        putsUart0("Received Packet Info:\n ");
        putsUart0("Frame ID: ");
        //printHex8(recPacket->frame_id);
        putsUart0("Type: ");
        //printHex8(recPacket->type);
        putsUart0("Data Length: ");
        //printHex8(recPacket->dataLength);
        for(i = 0; i < recPacket->dataLength; i++) {
            putsUart0("Data: ");
            //printHex8(recPacket->data[i]);
        }

        // You can process your packet here (cast to struct, etc.)
        // Save last received
       //memcpy(&lastReceivedPacket, recPacket, sizeof(nrf24lo1));
        writeRegister(STATUS, (1 << 6)); // Clear RX_DR
    }

    uint8_t fifo_status = readRegister(FIFO_STATUS);
    putsUart0("FIFO: ");
    //printHex8(fifo_status);  // Should show TX_EMPTY = 1 after each send
}
