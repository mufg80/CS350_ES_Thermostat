/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

// Adding Timer Include
#include <ti/drivers/Timer.h>

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Declare functions.
void implementAction();
void checkState();

// Global Variables.
bool buttonToggle;
bool initialScan;
enum State{INIT, SOS, OK, GAP};
enum State currentState;
int tickCounter;

// Global Constants.
const int sosPattern = 0x5477715;
const int sosFinished = 0x8000000;
const int sosMask = 0x77700;
const int okPattern = 0x75C777;
const int okFinished = 0x800000;
const int okMask = 0x40000;
const int gapFinished = 0x80;


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);

    // Toggle the boolean on button press.
    buttonToggle = !buttonToggle;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);

    // Toggle the boolean on button press.
    buttonToggle = !buttonToggle;
}


/*
 *  ======== timerCallback ========
 *  Callback function for the timer
 *
 *
 *
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{

    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);

    // Timer callback calls two state machine functions,
    // first fixing state, then performing IO.
    checkState();
    implementAction();
}

/*
 *  ======== initTimer ========
 *  Initialize timer
 *
 *
 *
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);

    // Set to 1/2 second.
    params.period = 500000;

    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

void checkState(){
    // Switch statement to evaluate state of machine.
    switch(currentState){
        // Inititalize on first timer tick.
        case INIT:

            tickCounter = 1;
            buttonToggle = true;

            // After first tick move machine to GAP state.
            if(!initialScan){
                currentState = GAP;
            }

            break;
        case SOS:
            // Check for end of SOS pattern using counter and constant.
            if(tickCounter & sosFinished){
                // Finished with sos, set state to gap and restart counter.
                currentState = GAP;
                tickCounter = 1;
            }
            break;
        case OK:
            // Check for finished pattern using constant.
            if(tickCounter & okFinished){
                // OK pattern finished, change state to gap.
                currentState = GAP;
                tickCounter = 1;
            }
            break;
        case GAP:
            // Checking gap for finished against constant.
            if(tickCounter & gapFinished){
                // Gap finished, check state of boolean for next pattern.
                if(buttonToggle){
                    currentState = SOS;
                }else{
                    currentState = OK;
                }
                tickCounter = 1;
             }
            break;
        default:
            // Should never execute.
            currentState = INIT;
            break;
    }

}

void implementAction(){

    switch(currentState){
           case INIT:
               // On first timer tick, this logic should start LEDs in off condition.
               // Turn LEDs off on INIT and reset counter.
               GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
               GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

               tickCounter = 1;
               // On first tick, now that LEDS have been turned off, set bool to false.
               // This logic should never run after first tick.
               initialScan = false;
               break;
           case SOS:
               // Check pattern to see if any LEDs should turn on this tick.
               if(tickCounter & sosPattern){
                   // LEDS must turn on, check against mask for green or red.
                   if(tickCounter & sosMask){
                       GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                       GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                   }else{
                       GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                       GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                   }
                // Turn both LEDs off if none needed this tick.
               }else{
                   GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                   GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
               }
               break;
           case OK:
               // Check pattern to see if any LEDs should turn on this tick.
               if(tickCounter & okPattern){
                   // LEDS must turn on, check against mask for green or red.
                  if(tickCounter & okMask){
                      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                      GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                  }else{
                      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                      GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                  }
                // Turn both LEDs off if none needed this tick.
                }else{
                  GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                  GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                }
               break;
           case GAP:
               // Gap should always have LEDs off.
               GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
               GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

               break;
           default:
               // Should never execute, but turn off LEDs and set tick counter.
               GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
               GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
               tickCounter = 1;
               break;
       }

        // Increment Bitshift for next iteration of timer.
        tickCounter = tickCounter << 1;
}




/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }



    // Initialize timer.
    initTimer();

    // Initialize State Machine.
    currentState = INIT;

    // Initialize init variable.
    initialScan = true;

    return (NULL);
}
