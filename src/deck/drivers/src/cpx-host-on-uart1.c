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

static bool isInit = false;

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

  isInit = true;
  DEBUG_PRINT("CPX Over UART1: Deck driver initialization complete!\n");
}

static bool cpxOverUart1Test()
{
  return true;
}

static const DeckDriver crtpOver1UART = {
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