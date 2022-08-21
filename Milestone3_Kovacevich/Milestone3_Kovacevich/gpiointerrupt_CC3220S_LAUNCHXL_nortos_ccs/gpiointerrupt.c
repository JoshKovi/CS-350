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

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//Timer Driver
#include <ti/drivers/Timer.h>

//I would prefer this did not go out of scope by accident or
//be unreachable
Timer_Handle timer0;
unsigned int ms_elap = 0;

//States to track
//Message and message are used to track the message being sent, inline assignment was
//Not working so I ended up with this format
enum Message {SOS, OK};
volatile enum Message message = SOS;

//Writing is used to more clearly differentiate what each state is, and the enum value
//Is used to create the messages
enum Writing{dot, dash, short_b, medium_b, long_b};

//After a bunch of different state machines, this seemed like the best option as it allows
//a hardcoded message string without having to track a bunch of states with complex
//if/else or switch logic, significantly simplified my state machine each int corresponds
//to an enum value in writing, can be written as dot, short_b, dot, short_b but this
//was a little easier to make so I chose this option
char SOS_Order[18] = {0,2,0,2,0,3,1,2,1,2,1,3,0,2,0,2,0,4};
char OK_Order[12] = {1,2,1,2,1,3,1,2,0,2,1,4};

//Position tracks the position in the arrays above, cycle tracks how many passes
//have happened, this allows different lengths of illumination while still calling
//state every 500 ms without complex time tracking
unsigned int position = 0;
unsigned int cycle = 0; //Each cycle is 500ms

//This tracks if a button has been pressed to change the message
//To me it made more sense to have one button for each of the messages
//so I used that logic in each of the button callbacks
bool changeRequest = false;

void StateMachine(){
    //This holds the overall state machine, currently the options are SOS or OK for message
    //I was going to eliminate this logic as well but I was having difficulty using
    //one combined array without a bunch of extra checks, in the future will probably overcome issue
    //and simplify this for more flexibility

    if(message == SOS){
        //Here the current int is pulled from the array and used in the switch
        unsigned int current = SOS_Order[position];
        switch(current){
        case (dot):
            //This has a one cycle duration and turns the light on then goes to the next position
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            position++;
            break;
        case(dash):
            //This has a 3 cycle duration, turning on the light then incrementing cycle until 3
            //Once the cycle hits 3 (0.0 secs cycle = 1, 0.5 secs cycle = 2, 1.0 secs cycle = 3)
            //The cycle is reset and the position is advanced (turned off in next cycle at 1.5 secs)
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cycle++;
            if(cycle == 3){
                position++;
                cycle = 0;
            }
            break;
        case(short_b):
            //This is a single cycle duration and turns off both the red and green LEDs, then advances position
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            position++;
            break;
        case(medium_b):
            //This has a 3 cycle duration where both LEDs are off (Between letters)
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cycle++;
            if(cycle == 3){
                position++;
                cycle = 0;
            }
            break;
        case(long_b):
            //This has a 6 cycle duration where both LEDs are off
            //Depending on changeRequest value this may change message
            //Then will reset cycle and position and changeRequest if necessary
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cycle++;
            if(cycle == 6 && changeRequest){
                message = OK;
                position = 0;
                cycle = 0;
                changeRequest = false;
            }
            else if(cycle == 6){
                cycle = 0;
                position = 0;
            }
            break;

        }
    }
    else if(message == OK){
        //Here the current int is pulled from the array and used in the switch
        unsigned int current = OK_Order[position];
        switch(current){
        case (dot):
            //See dot above in SOS
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            position++;
            break;
        case(dash):
            //See dash above in SOS
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cycle++;
            if(cycle == 3){
                position++;
                cycle = 0;
            }
            break;
        case(short_b):
            //See short_b above in SOS
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            position++;
            break;
        case(medium_b):
            //See medium_b above in SOS
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cycle++;
            if(cycle == 3){
                position++;
                cycle = 0;
            }
            break;
        case(long_b):
            //See long_b above in SOS
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cycle++;
            if(cycle == 6 && changeRequest){
                message = SOS;
                position = 0;
                cycle = 0;
                changeRequest = false;
            }
            else if(cycle == 6){
                cycle = 0;
                position = 0;
            }
            break;
        }
    }

}


//Timer Callback, called every .5 seconds
void timerCallback(Timer_Handle myHandle, int_fast16_t status){
    StateMachine(); //Calls the separate state machine, used for ease
}
void initTimer(void){

    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000; //500ms or 500,000 us
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if(timer0 == NULL){
        /*Failed to initialize timer */
        while(1){}
    }
    if(Timer_start(timer0) == Timer_STATUS_ERROR){
        /*Failed to start timer */
        while(1){}
    }

}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) //Left Button
{
    /* Toggle Red LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);

    //Changes the SOS signal to OK at next message end
    if(message == SOS){
        changeRequest = true;
    }
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) //Right Button
{
    /* Toggle Green LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);

    //Changes the OK signal to SOS at next message end
    if(message == OK){
        changeRequest = true;
    }

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
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); Commented out for ease/clarity

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */

    //Initiate timer
    initTimer();


    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }





    return (NULL);
}
