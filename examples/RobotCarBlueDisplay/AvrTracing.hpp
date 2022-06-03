/*
 *  AvrTracing.hpp
 *  Is named "hpp" to enable easy excluding from a project without deleting the file from the source directory.
 *
 *  Attach button between ground and pin 2. As long as you press it, the current program counter is printed.
 *  AND / OR use startTracing() and stopTracing() to trace selected parts of your code.
 *  !!! startTracing() sets pin 2 to LOW !!!
 *  With tracing enabled and 115200 baud, the effective CPU frequency is around 2 kHz.
 *
 *  Program memory size of library:
 *  NUMBER_OF_PUSH is defined (static mode): 284 bytes + 52 if DEBUG_TRACE_INIT is defined (336 total)
 *  NUMBER_OF_PUSH is not defined (dynamic mode): 344 bytes (60 bytes more than static) + 196 if DEBUG_TRACE_INIT is defined (540 total)
 *  You can first use the dynamic mode without DEBUG_TRACE_INIT defined, and call printNumberOfPushesForISR()
 *  to get the right number of pushes and then use static mode with that value, to keep the program memory space low
 *  or to proof, that you have counted the pushes of the ISR correct :-).
 *
 *  Usage:
 *  #define NUMBER_OF_PUSH 0x17 // This enables static mode. Saves 60 bytes program memory.
 *  #include "AvrTracing.hpp"
 *  ...
 *  setup() {
 *    Serial.begin(115200);
 *    initTrace();
 *    // optional info output
 *    printNumberOfPushesForISR();
 *    ...
 *    startTracing();  // This connects Pin 2 to ground
 *    ... // your code
 *    stopTracing();  // This releases connection to ground
 *  }
 *
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of AvrTracing https://github.com/ArminJo/AvrTracing.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */
#ifndef _AVR_TRACING_HPP
#define _AVR_TRACING_HPP

#include "digitalWriteFast.h"
#include "AvrTracing.h" // Contains function prototypes

//#define DEBUG_TRACE_INIT // To see internal information at call of initTrace() costs 52 (static) / 196 (dynamic) bytes of program memory

/*
 * The amount of pushes for this ISR is compiler and compiler flag dependent
 * I saw 15, 17, 19 and 20 pushes.
 * They can be found by looking for <__vector_1> in the assembler file.
 * The assembler file can be generated with avr-objdump --section-headers --source --line-numbers <myfilename>.elf  > <myfilename>.lss.
 * The ATTinyCore board package generates this assembler as a *.lst file.
 * If you use the -g flag for the linker, the c source code content is also available in the lss file.
 *
 * If NUMBER_OF_PUSH is not defined, then initTrace() tries to determine the value dynamically by calling the ISR directly from a known address.
 */
#if !defined(NUMBER_OF_PUSH)
// static mode
//#define NUMBER_OF_PUSH 15
//#define NUMBER_OF_PUSH 17
//#define NUMBER_OF_PUSH 19
#endif

#if !defined(NUMBER_OF_PUSH)
// dynamic mode
#define NUMBER_OF_PUSH_MIN 14 // Minimal number of pushes as start for detection current value
#define INVALID_VALUE_OF_PUSH_ADJUST 42 // any number not equal the expected number of pushes
#define ADDRESS_COMPARISON_MAX_DELTA 16 // Maximal delta of known function start address and address found on stack
#endif

union WordUnionForTracing {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } byte;
    uint16_t UWord;
    int16_t Word;
    uint8_t *BytePointer;
};

#define VERSION_AVR_TRACING "1.0.0"
#define VERSION_AVR_TRACING_MAJOR 1
#define VERSION_AVR_TRACING_MINOR 0

extern volatile unsigned long timer0_millis;
extern volatile unsigned long timer0_overflow_count;
uint8_t sPrintCount = 0;
uint8_t sLastMSBytePrinted = 0;

//uint16_t sCallCount = 0;
/*
 * Prints PC from stack
 * The amount of pushes for this ISR is compiler and compiler flag dependent
 * I saw 15, 17, 19, 20 and 23 (DEBUG_TRACE_INIT enabled) pushes.
 * They can be found by looking for <__vector_1> in the assembler file.
 * The assembler file can be generated with avr-objdump --section-headers --source --line-numbers <myfilename>.elf  > <myfilename>.lss.
 * The ATTinyCore board package generates this assembler as a *.lst file.
 * If you use the -g flag for the linker, the c source code content is also available in the lss file.
 *
 * If NUMBER_OF_PUSH is not defined, the ISR starts with the value 15 and looks for an zero byte of the first R1 push on the stack
 * This register is always held zero, except if we interrupt an MUL operation.
 *
 * Sending 11 characters "PC=0x...\r\n" each time lasts around 1 millisecond.
 */
#if ! defined(NUMBER_OF_PUSH)
uint8_t sPushAdjust = INVALID_VALUE_OF_PUSH_ADJUST; // contains (NUMBER_OF_PUSH + 1). + 1 to get the MSByte of the call address directly.
#endif
extern "C" void INT0_vect(void) __attribute__ ((optimize("-Os"),signal,__INTR_ATTRS));
void INT0_vect(void) {
// ISR(INT0_vect) {
    uint8_t *tStackPtr = (uint8_t*) SP;

#if defined(NUMBER_OF_PUSH)
    tStackPtr += NUMBER_OF_PUSH + 1;
#else
    uint8_t tPushAdjust = sPushAdjust;
    if (tPushAdjust == INVALID_VALUE_OF_PUSH_ADJUST) {
        /*
         * Initialize tPushAdjust here
         * If we have 17 pushs and pops for this function, sPushAdjust is 17+1
         * Search the stack for address of enableINT0InterruptOnFallingEdge,
         * which calls us first and adjust sPushAdjust
         */
        tPushAdjust = NUMBER_OF_PUSH_MIN; // start search from this value
#  if defined(DEBUG_TRACE_INIT)
        sendStringForTrace("Stack=0x");
#  endif
        WordUnionForTracing tAddressToLookFor;
        WordUnionForTracing tAddressFromStack;
        // The address on stack is the address of the statement after the "INT0_vect();" statement in the function enableINT0InterruptOnLowLevel()
        // and therefore &enableINT0InterruptOnLowLevel + 2
        // Real addresses on stack are always shifted right by one!
        tAddressToLookFor.UWord = (reinterpret_cast<uint16_t>(&enableINT0InterruptOnLowLevel)) + 2;
        do {
            tAddressFromStack.byte.HighByte = *(tStackPtr + tPushAdjust);
            tPushAdjust++;
#  if defined(DEBUG_TRACE_INIT)
            // First stack position printed is data of push number NUMBER_OF_PUSH_MIN + 1
            sendUnsignedByteHex(*(tStackPtr + tPushAdjust));
            sendUSARTForTrace(' ');
#  endif
            tAddressFromStack.byte.LowByte = *(tStackPtr + tPushAdjust);
            // Break if tAddressFromStack is between tAddressToLookFor and tAddressToLookFor + ADDRESS_COMPARISON_MAX_DELTA
            // If tAddressFromStack is smaller than tAddressToLookFor we get an unsigned underflow resulting in a great number :-)
            // The comparison tAddressFromStack.UWord != tAddressToLookFor.UWord is 12 bytes longer :-(
        } while (tAddressFromStack.UWord - tAddressToLookFor.UWord > ADDRESS_COMPARISON_MAX_DELTA
                && tPushAdjust < INVALID_VALUE_OF_PUSH_ADJUST);
        tPushAdjust--;

#  if defined(DEBUG_TRACE_INIT)
        sendLineFeed();
        sendStringForTrace("Address of enableINT0InterruptOnLowLevel()>>1=");
        sendUnsignedIntegerHex(reinterpret_cast<uint16_t>(&enableINT0InterruptOnLowLevel) + 2);
        if (tPushAdjust == INVALID_VALUE_OF_PUSH_ADJUST) {
            sendStringForTrace(" not found on stack");
        } else {
            sendStringForTrace(" found after 0x");
            sendUnsignedByteHex(tPushAdjust - 1);
            sendStringForTrace(" pushes on the stack");
        }
        sendLineFeed();
#  endif
        sPushAdjust = tPushAdjust;
    }
    tStackPtr += tPushAdjust;
#endif

    // cannot load 16 bit directly, since bytes are swapped then
    WordUnionForTracing tPC;
    tPC.byte.HighByte = *tStackPtr;
    tPC.byte.LowByte = *(tStackPtr + 1);
    tPC.UWord <<= 1;   // Generate LSB. The program counter points only to even addresses and needs no LSB.
    /*
     * Memory layout:
     * 0000 __vectors (interrupt vector table)
     * Startup is from __ctors_start (==__init) to __bad_interrupt
     * PC (after init()) has value from __bad_interrupt + 2 where e.g. micros() start
     * to __stop_program (== _etext - 2)
     */
    if (!(tPC.UWord >= (uint16_t) &__init && tPC.UWord <= (uint16_t) &_etext)) {
        sendUSARTForTrace('X');  // no valid address
    }
    sendPCHex(tPC.UWord); // PC=0x03DC (11 character@115200 baud) => 954,86 us or 1047,2727 Hz rate. But I had to trigger it externally with 1070 Hz to get no misses.

    /*
     * Check for millis() interrupt pending
     * this is true almost every time, since we require 1 ms for one run.
     * But then delay still lasts 50 times longer.
     * 88 bytes flash
     */
//    if (TIFR0 & _BV(TOV0)) {
//        // reset flag and simulate ISR
//        TIFR0 = _BV(TOV0);
//        timer0_millis++; // do not handle fractions here
//        timer0_overflow_count++; // required for micros()
//    }
//    sCallCount++;
}

__attribute__((optimize("-Os"))) void initTrace() {
#if defined(DEBUG_TRACE_INIT)
#  if defined(NUMBER_OF_PUSH)
    sendStringForTrace("# of pushes in ISR=0x");
    sendUnsignedByteHex(NUMBER_OF_PUSH);
    sendLineFeed();
#  endif // otherwise value of sPushAdjust is printed in ISR on first call
#endif

    enableINT0InterruptOnLowLevel();
}

/*
 * Connect button between pin2 and ground.
 * INT0 is at pin2
 * Enable interrupts during button press.
 */
__attribute__((optimize("-Os"))) void enableINT0InterruptOnLowLevel() {

#if !defined(NUMBER_OF_PUSH)
    INT0_vect(); // call ISR to compute sPushAdjust
#endif

    pinModeFast(2, INPUT_PULLUP);

#if defined(EICRA)
    EICRA &= ~(_BV(ISC01) | _BV(ISC00)); // interrupt on low level
    EIFR |= _BV(INTF0);  // clear interrupt bit
    EIMSK |= _BV(INT0);  // enable interrupt
#elif defined(GIFR)
    MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // interrupt on low level
    GIFR |= _BV(INTF0);  // clear interrupt bit
    GIMSK |= _BV(INT0);  // enable interrupt
#endif

}

/*
 * Set pin2 as output and drive it to low
 * The first 2 instructions after startTracing() are not printed.
 * You can insert 2  "_NOP();" statements right after startTracing() as a workaround.
 */
__attribute__((optimize("-Os"))) void startTracing() {
    pinModeFast(2, OUTPUT);
    digitalWriteFast(2, LOW); // The next 2 instructions are not yet printed
}

__attribute__((optimize("-Os"))) void stopTracing() {
    digitalWriteFast(2, HIGH);
    pinModeFast(2, INPUT); // results in INPUT_PULLUP, since we wrote the bit to HIGH before. This is the last instruction printed.
}

/*
 * Drive pin 2 to high and reset it input pullup
 */
__attribute__((optimize("-Os"))) void sendStringForTrace(const char *aStringPtr) {
    while (*aStringPtr != 0) {
        sendUSARTForTrace(*aStringPtr++);
    }
}

__attribute__((optimize("-Os"))) void sendPCHex(uint16_t aPC) {
    if (sPrintCount == 0) {
        sendUSARTForTrace('P');
        sendUSARTForTrace('C');
        sendUSARTForTrace('=');
        sendUSARTForTrace('0');
        sendUSARTForTrace('x');
        sLastMSBytePrinted = 0;
    }
    sendUnsignedInteger(aPC);
    sendUSARTForTrace(' ');
    if (sPrintCount > 20) {
        sPrintCount = -1;
        sendLineFeed();
    }

    sPrintCount++;
}

/*
 * Not used yet
 */
__attribute__((optimize("-Os"))) void sendHex(uint16_t aInteger, char aName) {
    sendUSARTForTrace(aName);
    sendUSARTForTrace('=');
    sendUnsignedIntegerHex(aInteger);
    sendLineFeed();
}

/*
 * for own debugging
 */
__attribute__((optimize("-Os"))) void sendHexNoInterrupts(uint16_t aInteger, char aName) {
    noInterrupts();
    sendUSARTForTrace(aName);
    sendUSARTForTrace('=');
    sendUnsignedIntegerHex(aInteger);
    sendLineFeed();
    interrupts();
}
/**
 * ultra simple blocking USART send routine - works 100%!
 */
__attribute__((optimize("-Os"))) void sendUSARTForTrace(char aChar) {
// wait for buffer to become empty
#  if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__)
        // Use TX1 on MEGA and on Leonardo, which has no TX0
        while (!((UCSR1A) & (1 << UDRE1))) {
            ;
        }
        UDR1 = aChar;
#  else
    while (!((UCSR0A) & (1 << UDRE0))) {
        ;
    }
    UDR0 = aChar;
#  endif // Atmega...
}

__attribute__((optimize("-Os"))) void sendLineFeed() {
    sendUSARTForTrace('\r');
    sendUSARTForTrace('\n');
}

__attribute__((optimize("-Os"))) char nibbleToHex(uint8_t aByte) {
    aByte = aByte & 0x0F;
    if (aByte < 10) {
        return aByte + '0';
    }
    return aByte + 'A' - 10;
}

__attribute__((optimize("-Os"))) void sendUnsignedByteHex(uint8_t aByte) {
    sendUSARTForTrace(nibbleToHex(aByte >> 4));
    sendUSARTForTrace(nibbleToHex(aByte));
}

union WordUnionForPrint {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } UByte;
    uint8_t UBytes[2];
    uint16_t UWord;
    int16_t Word;
    uint8_t *BytePointer;
};

__attribute__((optimize("-Os"))) void sendUnsignedIntegerHex(uint16_t aInteger) {
    sendUSARTForTrace('0');
    sendUSARTForTrace('x');
    sendUnsignedByteHex(aInteger >> 8);
    sendUnsignedByteHex(aInteger);
}

__attribute__((optimize("-Os"))) void sendUnsignedInteger(uint16_t aInteger) {
    WordUnionForPrint tInteger; // saves 6 bytes
    tInteger.UWord = aInteger;
    if (sLastMSBytePrinted != tInteger.UByte.HighByte) {
        sLastMSBytePrinted = tInteger.UByte.HighByte;
        sendUnsignedByteHex(tInteger.UByte.HighByte);
    }
    sendUnsignedByteHex(tInteger.UByte.LowByte);
}

/*
 * Utility function using standard Serial
 */

/*
 * Function for printing short info about number of pushes defined or found
 */
__attribute__((optimize("-Os"))) void printNumberOfPushesForISR() {
#  if defined(NUMBER_OF_PUSH)
    Serial.print(F("Defined # of pushes in ISR="));
    Serial.println(NUMBER_OF_PUSH);
# else
    Serial.print(F("Found "));
    Serial.print(sPushAdjust - 1);
    Serial.println(F(" pushes in ISR"));
#  endif
    Serial.flush();
}

__attribute__((optimize("-Os"))) void printTextSectionAddresses() {
    Serial.print(F("Start of text section=0x"));
    Serial.print((uint16_t) &__init, HEX);
    Serial.print(F(" end=0x"));
    Serial.println((uint16_t) &_etext, HEX);
    Serial.flush();
}

#endif // _AVR_TRACING_HPP
