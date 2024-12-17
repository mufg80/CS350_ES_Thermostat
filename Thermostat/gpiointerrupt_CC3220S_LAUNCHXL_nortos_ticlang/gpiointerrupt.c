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

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

// Struct to hold stats.
struct ServerStats{
       int16_t     temperature;
       int16_t     setpoint;
       uint8_t     heat;
       uint32_t    seconds;
    };

// Function declarations
void updateButtons(struct ServerStats* statStruct);
void updateTemp(struct ServerStats* statStruct);
void updateServer(struct ServerStats* statStruct);
void stateTick();
void stateAction(struct ServerStats* statStruct);

// Enum for state machine.
enum actionTicks{
    INI,
    IDL,
    BTN,
    TMP,
    ALL,}actionTick;



// Contant for time elapsed code.
const unsigned long TIMERPERIOD = 100;

// Variables for elapsed time.
unsigned long elapsedTime = 0;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t             txBuffer[1];
uint8_t             rxBuffer[2];
I2C_Transaction     i2cTransaction;

// UART Global Variables
char                output[64];
int                 bytesToSend;

// Driver Handles - Global variables
I2C_Handle      i2c;
UART2_Handle     uart;
Timer_Handle    timer0;

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
volatile unsigned char Button0Flag = 0;
void gpioButtonFxn0(uint_least8_t index)
{
    Button0Flag = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
volatile unsigned char Button1Flag = 0;
void gpioButtonFxn1(uint_least8_t index)
{
    Button1Flag = 1;
}

void initUART(void)
{
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t              i, found;
    I2C_Params          i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}


void initTimer(void)
{
    Timer_Params    params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }

    DISPLAY( snprintf(output, 64, "Timer Configured\n\r"))
}

int16_t readTemp(void)
{
    int     j;
    int16_t temperature = 0;

    i2cTransaction.readCount  = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
 }

void stateTick(){

    switch(actionTick){
    // Should only reach this at 100ms(first tick).
    case INI:
        actionTick = IDL;
        break;
    // Idle can transition to all other states.
    case IDL:
        // Check first 1000 as first condition. Use Modulus so that
        // multiples are considered. For example, 800 % 200 == 0 is true.
        // At 1000, all actions should be taken.
        if(elapsedTime % 1000 == 0){
            actionTick = ALL;
        }else if(elapsedTime % 500 == 0){
            actionTick = TMP;
        }else if(elapsedTime % 200 == 0){
            actionTick = BTN;
        }else{
            actionTick = IDL;
        }
        break;
    case BTN:
        // If on button, can only transition to temp  or idle.
        if(elapsedTime % 500 == 0){
            actionTick = TMP;
        }else{
            actionTick = IDL;
        }
        break;
    case TMP:
        // If on temp, can only transition to button or idle.
        if(elapsedTime % 200 == 0){
            actionTick = BTN;
        }else{
            actionTick = IDL;
        }
        break;
    case ALL:
        // If on All, must transition to idle, reset elapsed time as well.
        // Setting to 100, since it reached all at 1000, this tick would be 1100ms.
        actionTick = IDL;
        elapsedTime = 100;
        break;
    default:
        // Unreachable code, programmed as best practice.
        actionTick = INI;
    }

}
void stateAction(struct ServerStats* statStruct){
    switch(actionTick){
        case INI:
            // Do nothing.
            break;
        case IDL:
            // Do nothing.
            break;
        case BTN:
            // Call button function to check for interrupts and perform work.
            updateButtons(statStruct);
            break;
        case TMP:
            // Update temperature from sensor through I2C.
            updateTemp(statStruct);
            break;
        case ALL:
            // Every second all operations performed. Server comms simulated through UART.
            updateButtons(statStruct);
            updateTemp(statStruct);
            updateServer(statStruct);
            break;
        default:
            // Unreachable code, programmed as best practice.
            actionTick = INI;
        }
}

// Function to update struct setpoint member using button 0 as addition and button 1 as subtraction.
void updateButtons(struct ServerStats* statStruct){
    // Button 0 is for adding, if its been pressed flag will have a one,
    // add it to setpoint and then zero it out.
    statStruct->setpoint += Button0Flag;
    Button0Flag = 0;

    // Button 1 is for subtraction, if its been pressed flag will have
    // a one, subtract it from setpoint then zero flag.
    statStruct->setpoint -= Button1Flag;
    Button1Flag = 0;

}

// Function to update temp, also turns heat on or off depending on setpoint.
void updateTemp(struct ServerStats* statStruct){
    // Call function to check temp through I2C.
    statStruct->temperature = readTemp();

    // If setpoint is greater, heat must be turned on, else off.
    if(statStruct->setpoint > statStruct->temperature){
        statStruct->heat = 1;
        // Heat on, energize LED.
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }else{
        statStruct->heat = 0;
        // Heat off, deenergize LED.
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }

}

// Function to update server (simulate). Displays info to terminal.
void updateServer(struct ServerStats* statStruct){
    // Increment seconds variable since this task is called every second.
    statStruct->seconds++;
    // Output information through UART
    // <%02d,%02d,%d,%04d>, temperature, setpoint, heat, seconds
    DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", statStruct->temperature, statStruct->setpoint, statStruct->heat, statStruct->seconds))

}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{


  //  size_t bytesWritten = 0;

    /* Call driver init functions */
    GPIO_init();

#ifdef CONFIG_GPIO_TMP_EN
    GPIO_setConfig(CONFIG_GPIO_TMP_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* Allow the sensor to power on */
    sleep(1);
#endif

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }


    initUART(); // The UART must be initialized before calling initI2C()
    DISPLAY( snprintf(output, 64, "UART + GPIO + Timer +I2C + Interrupts by Eric Gregori\n\r"))
    DISPLAY( snprintf(output, 64, "GPIO + Interrupts configured\n\r"))
    initI2C();
    initTimer();



    DISPLAY( snprintf(output, 64, "Starting Task Scheduler\n\r"))

    // Set enum to initialize for first tick.
    actionTick = INI;

    // Declare struct for data and initialize members to 0.
    struct ServerStats stats;
    stats.setpoint = 0;
    stats.heat = 0;
    stats.temperature = 0;
    stats.seconds = 0;

    // Create a pointer for stats struct for passing to functions.
    struct ServerStats* statStruct = &stats;

    // Start infinite loop.
    while (1)
    {

        // Each tick, fix state of state machine.
        stateTick();

        // After fixing state, perform actions. Pass in struct pointer.
        stateAction(statStruct);

        // While loop waiting for timer interrupt, timer ticks at 100ms.
        while (!TimerFlag){}   // Wait for timer period
        TimerFlag = 0;         // Lower flag raised by timer

        // Since another 100ms have elapsed, add to elapsed time variables.
        elapsedTime += TIMERPERIOD;


    }

    return (NULL);
}
