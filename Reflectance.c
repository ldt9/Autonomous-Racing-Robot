// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "Clock.h"

//volatile uint8_t prev_count = 0;

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    // write this as part of Lab 6
    // Port 5 Init
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->REN |= 0x08;
    P5->OUT &= ~0x08;

    // Port 9 Init
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->REN |= 0x04;
    P9->OUT &= ~0x04;

    // Port 7 Init
    P7->SEL0 &= ~0x00;
    P7->SEL1 &= ~0x00;
    P7->DIR &= ~0x00;
    P7->REN |= 0x00;
    P7->OUT |= 0x00;

}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
    // write this as part of Lab 6
    uint8_t result;
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P9->OUT |= 0x04;      // turn on 4 odd IR LEDs
    P7->DIR = 0xFF;       // make P7.7-P7.0 out
    P7->OUT = 0xFF;       // prime for measurement
    Clock_Delay1us(10);   // wait 10 us
    P7->DIR = 0x00;       // make P7.7-P7.0 in
    Clock_Delay1us(time);
    result = P7->IN&0xFF;
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    P9->OUT &= ~0x04;     // turn off 4 odd IR LEDs
    return result;
}


/*
uint8_t Reflectance_FSM(uint8_t data){
    uint8_t prev_zero_count = 0;
    uint8_t zero_count = 0;
    int32_t result = 0x00;
    int32_t i;
    int32_t bit;
    for (i=0; i<8; i++){
        bit = (prev_data >> i) & 1;
        if(bit == 0){
            prev_zero_count++;
        }
    }
    for (i=0; i<8; i++){
        bit = (data >> i) & 1;
        if(i == 0 && bit == 1){
           result |= 0x02;
        }
        else if(i == 7 && bit == 1){
            result |= 0x04;
        }
        else if(bit == 0){
            zero_count++;
        }
    }
    if(((8-zero_count) - (8-prev_zero_count)) == 0xFF || ((8-zero_count) - (8-prev_zero_count)) < 2 || (prev_zero_count > 7 && zero_count <= 7) || (prev_zero_count <= 7 && zero_count > 7)){
        result |= 0x01;
    }
    prev_data = data;
    return result;
}
*/

uint8_t Reflectance_FSM(uint8_t data){
    uint8_t i;
    uint8_t new_data = 0x00;

    for (i=0; i<8; i++){
        if(((data >> i) & 0x01) == 0x01){
            if(i == 0) {    //0 = Rightmost Sensor
                new_data |= 0x01;   //Right Sensor = On
            }
            else if(i == 7){     // 7 = Leftmost Sensor
                new_data |= 0x04;   //Left Sensor = On
            }
            else{
                new_data |= 0x02;
                i = 6;
            }
        }
    }

    return new_data;

}


/*
uint8_t Reflectance_FSM(uint8_t data){
    uint8_t curr_count = 0;
    uint8_t i, diff;
    uint8_t pc = 0, cc = 0;
    uint8_t new_data = 0x00;

    for (i=0; i<8; i++){
        if(((data >> i) & 0x01) == 0x01){
            if(i == 0) {    //0 = Rightmost Sensor
                new_data |= 0x02;   //Right Sensor = On
            }
            if(i == 7){     // 7 = Leftmost Sensor
                new_data |= 0x04;   //Left Sensor = On
            }
            curr_count++;    //Any Sensor is On
        }
    }

    diff = curr_count - prev_count;
    if(prev_count){
        pc = 1;
    }
    if(curr_count){
        cc = 1;
    }

    //   0 ! <-> 1   AND    diff = -1       diff = 0        diff = 1
    if((pc ^ cc == 0) & (diff == 0xFF || diff == 0x00 || diff == 0x01)){
        new_data &= ~0x01;
    }
    else{
        new_data |= 0x01;
    }

    prev_count = curr_count;
    return new_data;

}
*/

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // write this as part of Lab 10
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P9->OUT |= 0x04;      // turn on 4 odd IR LEDs
    P7->DIR |= 0xFF;       // make P7.7-P7.0 out
    P7->OUT |= 0xFF;       // prime for measurement
    Clock_Delay1us(10);
    P7->DIR &= 0x00;       // make P7.7-P7.0 in
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // write this as part of Lab 10
    uint8_t result;
    result = P7->IN&0xFF;
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    P9->OUT &= ~0x04;     // turn off 4 odd IR LEDs
    return result;
}
