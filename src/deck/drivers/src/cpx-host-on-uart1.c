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

/* Fake deck driver for using CPX (and CRTP) over UART1 on the expansion connector.
 * Note that this has to be forced on in the deck subsystem, since there's no 1-wire
 * memory available.
 */

#define DEBUG_MODULE "CRTP-OVER-UART1"

#include <stdint.h>

#include "deck.h"
#include "param.h"
#include "debug.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "cpx_uart1_transport.h"
#include "cpx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static bool isInit = false;
#ifdef CONFIG_CPX_UART1_PING_TEST
static TimerHandle_t pingTimer;
#endif

#ifdef CONFIG_CPX_UART1_PING_TEST
static void pingTimerCallback(TimerHandle_t xTimer)
{
  static uint32_t pingCounter = 0;
  CPXRoutablePacket_t pingPacket;
  
  DEBUG_PRINT("CPX Over UART1: Timer callback triggered - ping #%lu\n", (unsigned long)pingCounter);
  
  // Create a simple ping message
  pingPacket.route.destination = CPX_T_WIFI_HOST; // Send to external host
  pingPacket.route.source = CPX_T_STM32;
  pingPacket.route.function = CPX_F_TEST; // Use TEST function
  pingPacket.route.lastPacket = true;
  pingPacket.route.version = CPX_VERSION;
  pingPacket.dataLength = 4;
  
  // Ping data: counter
  pingPacket.data[0] = (uint8_t)((pingCounter >> 24) & 0xFF);
  pingPacket.data[1] = (uint8_t)((pingCounter >> 16) & 0xFF);
  pingPacket.data[2] = (uint8_t)((pingCounter >> 8) & 0xFF);
  pingPacket.data[3] = (uint8_t)(pingCounter & 0xFF);
  
  DEBUG_PRINT("CPX Over UART1: Sending ping #%lu\n", (unsigned long)pingCounter);
  
  // Send ping via CPX UART1 transport
  cpxUART1TransportSend(&pingPacket);
  
  DEBUG_PRINT("CPX Over UART1: Ping #%lu sent successfully\n", (unsigned long)pingCounter);
  
  pingCounter++;
  
  // Stop after 10 pings to avoid flooding
  if (pingCounter >= 10) {
    DEBUG_PRINT("CPX Over UART1: Stopping ping timer after 10 pings\n");
    xTimerStop(pingTimer, 0);
  }
}
#endif

static void cpxOverUart1Init(DeckInfo *info)
{
  if (isInit)
    return;

  DEBUG_PRINT("CPX Over UART1: Initializing deck driver...\n");
  
  DEBUG_PRINT("CPX Over UART1: Initializing UART1 transport...\n");
  cpxUART1TransportInit();
  
  DEBUG_PRINT("CPX Over UART1: Initializing internal router...\n");
  cpxInternalRouterInit();
  
  DEBUG_PRINT("CPX Over UART1: Initializing external router...\n");
  cpxExternalRouterInit();
  
  DEBUG_PRINT("CPX Over UART1: Initializing CPX...\n");
  cpxInit();
  
#ifdef CONFIG_CPX_UART1_PING_TEST
  // Create ping timer (send ping every 2 seconds)
  DEBUG_PRINT("CPX Over UART1: Creating ping timer...\n");
  pingTimer = xTimerCreate("CPXPing", 
                          pdMS_TO_TICKS(2000), // 2 seconds
                          pdTRUE,              // Auto-reload
                          NULL,                // Timer ID
                          pingTimerCallback);
  
  if (pingTimer != NULL) {
    xTimerStart(pingTimer, 0);
    DEBUG_PRINT("CPX Over UART1: Ping timer started (every 2 seconds)\n");
  } else {
    DEBUG_PRINT("CPX Over UART1: ERROR - Failed to create ping timer!\n");
  }
#endif
  
  isInit = true;
  DEBUG_PRINT("CPX Over UART1: Deck driver initialization complete!\n");
}

static bool cpxOverUart1Test()
{
  return true;
}

static const DeckDriver crtpOver1UART = {
    .vid = 0xBC,
    .pid = 0x0E,
    .name = "cpxOverUART1",

    .usedPeriph = DECK_USING_UART1,

    .init = cpxOverUart1Init,
    .test = cpxOverUart1Test,
};

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if CRTP over UART1 has been forced
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART1, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(crtpOver1UART); 