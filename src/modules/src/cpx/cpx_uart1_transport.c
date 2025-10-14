/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* UART1 transport layer for CPX */

#define DEBUG_MODULE "CPX-UART1-TRANSP"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "queue.h"
#include "stm32fxxx.h"
#include "system.h"
#include "autoconf.h"

#include "cpx.h"
#include "cpx_uart1_transport.h"

#define UART_TX_QUEUE_LENGTH 4
#define UART_RX_QUEUE_LENGTH 4

static xQueueHandle uartTxQueue;
static xQueueHandle uartRxQueue;

// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[CPX_UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;

// Used when sending/receiving data on the UART
static uart_transport_packet_t uartTxp;
static CPXPacket_t cpxTxp;
static uart_transport_packet_t uartRxp;

static EventGroupHandle_t evGroup;
/* Used to signal when ESP has said clear-to-send */
#define ESP_CTS_EVENT (1 << 0)
/* Used to signal when we should tell ESP it's clear-to-send */
#define ESP_CTR_EVENT (1 << 1)
/* Used to signal that there are packets in the outgoing TX queue */
#define ESP_TXQ_EVENT (1 << 2)
/* Used to signal that TX/RX tasks should shut down */
#define DEINIT_EVENT  (1 << 3)
/* Used to signal that RX task has shut down */
#define RX_DEINIT_EVENT (1 << 4)
/* Used to signal that TX task has shut down */
#define TX_DEINIT_EVENT (1 << 5)

/* Set when transport is de-initialized (i.e needs to release uart) */
static bool shutdownTransport = false;

static bool isInit = false;

static uint8_t calcCrc(const uart_transport_packet_t* packet) {
  const uint8_t* start = (const uint8_t*) packet;
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }

  return crc;
}

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp) {
  ASSERT((packet->route.destination >> 4) == 0);
  ASSERT((packet->route.source >> 4) == 0);
  ASSERT((packet->route.function >> 8) == 0);
  ASSERT(packet->dataLength <= CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

  txp->payloadLength = packet->dataLength + CPX_ROUTING_PACKED_SIZE;
  txp->routablePayload.route.destination = packet->route.destination;
  txp->routablePayload.route.source = packet->route.source;
  txp->routablePayload.route.lastPacket = packet->route.lastPacket;
  txp->routablePayload.route.function = packet->route.function;
  memcpy(txp->routablePayload.data, &packet->data, packet->dataLength);
  txp->payload[txp->payloadLength] = calcCrc(txp);
}

static void CPX_UART1_RX(void *param)
{
  systemWaitStart();
  DEBUG_PRINT("CPX UART1 RX: Task started, waiting for data...\n");

  while (shutdownTransport == false)
  {
    // Wait for start!
    uartRxp.start = 0x00;
    do
    {
      uart1GetDataWithTimeout(&uartRxp.start, M2T(200));
    } while (uartRxp.start != 0xFF && shutdownTransport == false);
    
    if (uartRxp.start == 0xFF) {
      DEBUG_PRINT("CPX UART1 RX: Start byte received (0xFF)\n");
    }

    if (uartRxp.start == 0xFF) {
      uart1GetDataWithDefaultTimeout(&uartRxp.payloadLength);
      DEBUG_PRINT("CPX UART1 RX: Payload length = %d\n", uartRxp.payloadLength);

      if (uartRxp.payloadLength == 0)
      {
        DEBUG_PRINT("CPX UART1 RX: Empty packet, setting CTS event\n");
        xEventGroupSetBits(evGroup, ESP_CTS_EVENT);
      }
      else
      {
        DEBUG_PRINT("CPX UART1 RX: Reading %d bytes of payload\n", uartRxp.payloadLength);
        uart1GetBytesWithDefaultTimeout(uartRxp.payloadLength, (uint8_t*) &uartRxp.payload);

        uint8_t crc;
        uart1GetDataWithDefaultTimeout(&crc);
        uint8_t calculatedCrc = calcCrc(&uartRxp);
        DEBUG_PRINT("CPX UART1 RX: CRC received=0x%02X, calculated=0x%02X\n", crc, calculatedCrc);
        
        if (crc == calculatedCrc) {
          DEBUG_PRINT("CPX UART1 RX: CRC OK, checking version\n");
          if (cpxCheckVersion(uartRxp.routablePayload.route.version)) {
            DEBUG_PRINT("CPX UART1 RX: Version OK, sending to queue\n");
            xQueueSend(uartRxQueue, &uartRxp, portMAX_DELAY);
          } else {
            DEBUG_PRINT("CPX UART1 RX: Version check FAILED\n");
          }
        } else {
          DEBUG_PRINT("CPX UART1 RX: CRC MISMATCH - packet rejected\n");
        }
        xEventGroupSetBits(evGroup, ESP_CTR_EVENT);
      }
    }
  }

  DEBUG_PRINT("CPX UART1 RX: Task shutting down\n");
  xEventGroupSetBits(evGroup, RX_DEINIT_EVENT);
  vTaskDelete(NULL);
}

static void CPX_UART1_TX(void *param)
{
  systemWaitStart();
  DEBUG_PRINT("CPX UART1 TX: Task started\n");

  while (shutdownTransport == false)
  {
    EventBits_t evBits = xEventGroupWaitBits(evGroup,
                                             ESP_TXQ_EVENT | DEINIT_EVENT,
                                             pdTRUE,  // Clear bits before returning
                                             pdFALSE, // Wait for any bit
                                             portMAX_DELAY);

    if ((evBits & DEINIT_EVENT) == DEINIT_EVENT)
    {
      DEBUG_PRINT("CPX UART1 TX: Deinit event received\n");
      break;
    }

    if ((evBits & ESP_TXQ_EVENT) == ESP_TXQ_EVENT)
    {
      DEBUG_PRINT("CPX UART1 TX: TXQ event received\n");
      // Wait for either CTS or CTR
      evBits = xEventGroupWaitBits(evGroup,
                                   ESP_CTR_EVENT | ESP_CTS_EVENT,
                                   pdTRUE,  // Clear bits before returning
                                   pdFALSE, // Wait for any bit
                                   portMAX_DELAY);

      if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
      {
        DEBUG_PRINT("CPX UART1 TX: Sending CTR (0xFF)\n");
        uint8_t ctr = 0xFF;
        uart1SendData(sizeof(ctr), (uint8_t *)&ctr);
      }

      if (uxQueueMessagesWaiting(uartTxQueue) > 0)
      {
        DEBUG_PRINT("CPX UART1 TX: %lu packets in TX queue\n", (unsigned long)uxQueueMessagesWaiting(uartTxQueue));
        // Dequeue and wait for either CTS or CTR
        xQueueReceive(uartTxQueue, &cpxTxp, 0);
        uartTxp.start = 0xFF;
        assemblePacket(&cpxTxp, &uartTxp);
        DEBUG_PRINT("CPX UART1 TX: Assembled packet, length=%d\n", uartTxp.payloadLength + UART_META_LENGTH);
        do
        {
          evBits = xEventGroupWaitBits(evGroup,
                                       ESP_CTR_EVENT | ESP_CTS_EVENT,
                                       pdTRUE,  // Clear bits before returning
                                       pdFALSE, // Wait for any bit
                                       portMAX_DELAY);
          if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
          {
            DEBUG_PRINT("CPX UART1 TX: Sending CTR (0xFF) in loop\n");
            uint8_t ctr = 0xFF;
            uart1SendData(sizeof(ctr), (uint8_t *)&ctr);
          }
        } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);
        DEBUG_PRINT("CPX UART1 TX: Sending packet data\n");
        uart1SendData((uint32_t) uartTxp.payloadLength + UART_META_LENGTH, (uint8_t *)&uartTxp);
      } else {
        DEBUG_PRINT("CPX UART1 TX: No packets in TX queue\n");
      }
    }
  }

  DEBUG_PRINT("CPX UART1 TX: Task shutting down\n");
  xEventGroupSetBits(evGroup, TX_DEINIT_EVENT);
  vTaskDelete(NULL);
}

void cpxUART1TransportSend(const CPXRoutablePacket_t* packet) {
  ASSERT(isInit == true && shutdownTransport == false);
  ASSERT(packet);

  DEBUG_PRINT("CPX UART1: Sending packet - dest=%d, src=%d, func=%d, len=%d\n", 
              packet->route.destination, packet->route.source, packet->route.function, packet->dataLength);
  xQueueSend(uartTxQueue, packet, portMAX_DELAY);
  xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
  DEBUG_PRINT("CPX UART1: Packet queued for transmission\n");
}

void cpxUART1TransportReceive(CPXRoutablePacket_t* packet) {
  ASSERT(isInit == true && shutdownTransport == false);
  ASSERT(packet);

  static uart_transport_packet_t cpxRxp;

  DEBUG_PRINT("CPX UART1: Waiting for packet in RX queue...\n");
  xQueueReceive(uartRxQueue, &cpxRxp, portMAX_DELAY);
  DEBUG_PRINT("CPX UART1: Packet received from RX queue\n");

  packet->dataLength = (uint32_t) cpxRxp.payloadLength - CPX_ROUTING_PACKED_SIZE;
  packet->route.destination = cpxRxp.routablePayload.route.destination;
  packet->route.source = cpxRxp.routablePayload.route.source;
  packet->route.function = cpxRxp.routablePayload.route.function;
  packet->route.lastPacket = cpxRxp.routablePayload.route.lastPacket;
  memcpy(&packet->data, cpxRxp.routablePayload.data, packet->dataLength);

  DEBUG_PRINT("CPX UART1: Packet processed - dest=%d, src=%d, func=%d, len=%d\n", 
              packet->route.destination, packet->route.source, packet->route.function, packet->dataLength);
}

void cpxUART1TransportInit() {
  // There's no support for re-initializing the UART transport once it's
  // been de-initialized. This is not needed by the ESP bootloader use-case,
  // since the procedure will reset the Crazyflie after ESP has been bootloaded
  ASSERT(shutdownTransport==false);

  DEBUG_PRINT("CPX UART1 Transport: Initializing...\n");
  DEBUG_PRINT("CPX UART1 Transport: Baudrate = %d\n", CONFIG_CPX_UART1_BAUDRATE);

  uartTxQueue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(CPXPacket_t));
  uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

  evGroup = xEventGroupCreate();

  DEBUG_PRINT("CPX UART1 Transport: Initializing UART1...\n");
  uart1Init(CONFIG_CPX_UART1_BAUDRATE);
  DEBUG_PRINT("CPX UART1 Transport: UART1 initialized successfully\n");

  // Initialize task for the ESP while it's held in reset
  DEBUG_PRINT("CPX UART1 Transport: Creating RX/TX tasks...\n");
  xTaskCreate(CPX_UART1_RX, AIDECK_ESP_RX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);
  xTaskCreate(CPX_UART1_TX, AIDECK_ESP_TX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("CPX UART1 Transport: Initialization complete!\n");
}

void cpxUART1TransportDeinit() {
  shutdownTransport = true;
  // Send event to unlock TX
  xEventGroupSetBits(evGroup, DEINIT_EVENT);
  // Wait for RX/TX event shutdown
  xEventGroupWaitBits(evGroup,
                      TX_DEINIT_EVENT | RX_DEINIT_EVENT,
                      pdTRUE,  // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);
} 