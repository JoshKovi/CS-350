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

//Timer Driver Code
#include <ti/drivers/Timer.h>
//I2C DRIVER CODE
#include <ti/drivers/I2C.h>
//UART Driver
#include <ti/drivers/UART2.h>


/* Driver configuration */
#include "ti_drivers_config.h"



// Driver Handles - Global variables
UART2_Handle uart;
char output[128];
size_t bytesWritten = 0;
#define DISPLAY(x) UART2_write(uart, output, x, &bytesWritten);
// UART Global Variables



//Start developer declared fields

//Button enum used to hold states
enum Button {Button_1, Button_2, No_Button};
volatile enum Button button = No_Button;

//Used as the timer flag
volatile bool TimerFlag = false;
//End developer declared fields


// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
// Driver Handles - Global variables


// Make sure you call initUART() before calling this function.

I2C_Params i2cParams;
void initI2C(I2C_Handle* i2c)
{

    int8_t i, found;


    DISPLAY(snprintf(output, 128, "Initializing I2C Driver - "))
    //UART2_write(uart, output, sizeof(output),  &bytesWritten);
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

#ifdef CONFIG_GPIO_TMP_EN
    GPIO_setConfig(CONFIG_GPIO_TMP_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* Allow the sensor to power on */
    sleep(1);
#endif

    // Open the driver
    //I2C_close(i2c);
    *i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (*i2c == NULL)
    {
        DISPLAY(snprintf(output, 128, "Failed\n\r"))
                while (1);
    }
    DISPLAY(snprintf(output, 128, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 128, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(*i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 128, "Found\n\r"));
                found = true;
            break;
        }
        DISPLAY(snprintf(output, 128, "No\n\r"));
    }
    if(found)
    {
        DISPLAY(snprintf(output, 128, "Detected TMP%s I2C address:%x\n\r",
                         sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 128, "Temperature sensor not found,contact professor\n\r"))
    }
}

int16_t readTemp(I2C_Handle* i2c)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    if (I2C_transfer(*i2c, &i2cTransaction))
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
        DISPLAY(snprintf(output, 128, "Error reading temperature sensor(%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 128,
                      "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}



//UART DRIVER CODE





void initUART(void)
{
    UART2_Params uartParams;
    // Init the driver
    //UART2_init();
    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_NONBLOCKING;
    //uartParams.readDataMode = UART_DATA_BINARY;
    //uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


// Driver Handles - Global variables
Timer_Handle timer0;
//volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = true;
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; //Cycle for timer is highest possible period 100ms
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
}




/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    //Button 1 depressed
    button = Button_1;
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
    //Button 2 depressed
    button = Button_2;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED  */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    //Configure button 0
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_HIGH);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    //Configure Button 1
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_HIGH);
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    //These are used to track time since reset, cycle, threshold temp and curtemp
    int16_t secSinceReset = 0;
    short cycle = 0;
    int16_t threshholdTemp = 30;//Set to 30 C as that is about what it is here right now
    int16_t curTemp;

    //LED Status enum
    enum LEDStatus {OFF, ON};
    enum LEDStatus led = OFF;



    //Initiates dependencies, timer functions as state machine
    I2C_Handle i2c;
    initUART();
    initI2C(&i2c);
    initTimer();

    while(1){
        //Handles the overall state machine  bzero resets the output as it
        //was having issues with buffer without cycle counts to 10 representing
        //100ms each increment
        cycle++;
        bzero(output, 128);

        //Every 200ms checks button status (fully depressed during cycle
        if(cycle % 2 == 0){ //200 ms, check button status
            if(button == Button_1){ //Lower Threshold
                threshholdTemp-= 1;
            }
            else if(button == Button_2) { //Raise Threshold
                threshholdTemp += 1;
            }
            button = No_Button;
        }
        //Checks every .5 seconds the temperature vs the threshold
        if(cycle % 5 == 0){ //500 ms, check Temperature
            curTemp = readTemp(&i2c);
            if(curTemp >= threshholdTemp){ //At or above threshold temp
                led = OFF;
            }
            else{//Below Threshold temp
                led = ON;
            }
        }

        //Checks every 1 second if light should be on or off
        if(cycle % 10 == 0){//1000 ms, update LED
            if(led == OFF){
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            else if(led == ON){
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else{ //Bad state, reset to off by default
                led = OFF;
            }
            //Resets cycle, increments secSinceReset, and outputs status
            cycle=0;
            secSinceReset++;
            DISPLAY(snprintf(output, 128, "<%02d, %02d, %d, %04d>\n\r", curTemp, threshholdTemp, led, secSinceReset))
        }
        TimerFlag = false;
        while(!TimerFlag){ //Unless flaged stay in loop

        }


    }


    return (NULL);
}
