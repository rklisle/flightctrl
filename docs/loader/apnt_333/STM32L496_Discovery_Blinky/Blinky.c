/*------------------------------------------------------------------------------
 * Copyright (c) 2018 - 2020 Arm Limited (or its affiliates). All
 * rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.Neither the name of Arm nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *------------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------*/

#include "main.h"                        
#include "Board_LED.h"                  // Board Support:LED
#include "Board_Joystick.h"             // Board Support:Joystick

static osThreadId_t tid_thrLED;         // Thread id of thread: LED
static osThreadId_t tid_thrJOY;         // Thread id of thread: BUT

extern const uint32_t delay_data[10];   // array with delay values
static       uint32_t delay = 0U;       // current delay value

/*------------------------------------------------------------------------------
  thrLED: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrLED (void *argument) {

  for (;;) {
    LED_On (0U);                                // Switch LED on
    osDelay (delay);            // Delay
    LED_Off (0U);                               // Switch off
    osDelay (delay);            // Delay
  }
}

/*------------------------------------------------------------------------------
  thrJOY: check Joystick state
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrJOY(void *argument) {
  uint32_t last = 0;
  uint32_t state = 0;
  uint32_t index = 0;

  delay = delay_data[index];

  for (;;) {
    state = (Joystick_GetState ());        // Get joystick state
    if (state != last){										 // Act only on state changes
      if (state != 0){                     // Act only on new presses
        index++;
        if (index == (sizeof(delay_data) / sizeof(delay_data[0]))){
          index = 0U;
        }
        delay = delay_data[index];         // Obtain next delay value from external flash memory
      }
    }
    last = state;
    osDelay (100U);
  }
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
void app_main (void *argument) {

  LED_Initialize ();                    // Initialize LEDs
  Joystick_Initialize ();               // Initialize Joystick

  tid_thrLED = osThreadNew (thrLED, NULL, NULL);  // Create LED thread
  if (tid_thrLED == NULL) { /* add error handling */ }

  tid_thrJOY = osThreadNew (thrJOY, NULL, NULL);  // Create JOY thread
  if (tid_thrJOY == NULL) { /* add error handling */ }

  osThreadExit();
}
