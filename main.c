/**
 * (PHY = 2MB)
 *
 * C_PROJECT_MainMCU_SPI_BT_WE_10 - Proyecto fin de curso TIOT
 * This version is be able to:
 * Send data Central to Periferic, and Periferic to Central, and visualize these in the PC.
 * Central: PC + Dongle (Proteus III)
 * Periferic: Proteus-III-SPI + MSP430
 *
 * Once the system is turned ON, it send Proteus III to SLEEP mode.
 * When switch S1 is pressed, the Proteus sends a connection request to the Central.
 * Once the connection is established, a REQUEST TO CHANGE PHYSICAL LAYER to 2MBIT is SENT, through the command CMD_PHYUPDATE_REQ
 * A cmd CMD_PHYUPDATE_CNF and CMD_PHYUPDATE_IND is received.
 * Once this has been changed. the Periferic receives data (simulating command to request specific information).
 * The Periferic receives 0xABCD (TBD)
 * The Periferic will send packets of data: a counter from 0 to 1000, and 1000 to 0.
 * This is performed by transmitting different data_requests, each packet with a max payload of 243 bytes.
 *
 * The data is sent: first 1 packet of data, triggered by the reception of the command: 0xABCDEFGH.
 * Waits for the command: CMD_DATA_CNF, and CMD_TXCOMPLETE_RSP. Sends next packet & Repeat until all data is sent.
 *
 * A machine state is defined with the following states:
 * ACTION_IDLE
 * ACTION_CONNECTED
 * ACTION_SLEEP
 *
 * There are Timeouts that trigger the transition between states. This is defined in the ISR timers.
 *
 * First time the button is pressed, the device will go to SCANNING state to look for Proteus modules in range.
 * It will acquire the BTMAC address of the modules in range, with the help of the command CMD_GET_DEVICES_REQ.
 *
 *
 *
 */

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "interface_ble.h"

// MSP-EXPF5529LP
// Main MCU is the Master SPI and the Proteus-III-SPI board is the slave of this communication.
// Main MCU is considered the HOST of Proteus-III-SPI board which is considered Radio Module.


int flag_channelopen = 0;


int main(void) {


    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Setup USCI B0 SPI for communication with Radio Module Proteus-III-SPI WE
    // UCBxCTL0 Register, page 986 of Family User Manual

    UCB0CTL1 |= UCSWRST;        // Put B0 in SW reset

    UCB0CTL0 |= UCSYNC + UCCKPH + UCCKPL + UCMST + UCMSB;  //UCSTEM only available in eUSCI
    UCB0CTL1 |= UCSSEL__SMCLK;  // SMCLK = 1 MHz. // UCB0BR0 = 2; // Prescaler for CLK. Together with UCB0BR1 // Consider that the MAX CLK for SPI Proteus board is 8Mbit/s
    UCB0CTL0 &= ~UCMODE0 + ~UCMODE1;     // USCI mode: 00 = 3 pin SPI with independent STE pin > active LOW
    // SPI CONFIG: SMCLK = 1MHz, 3-wire SPI, as Master, MSB first, CLK polarity: idle HIGH, CLK phase: data captured on first UCLK and changes on following edge.

    // Configure Ports. LEDs
    LED1_set;
    LED1_OFF;
    LED2_set;
    LED2_OFF;

    // SW1 interrupt - P2.1
    P2DIR &= ~BIT1;             // Make P2.1 as an input (SW1)
    P2REN |= BIT1;              // Enable resistor
    P2OUT |= BIT1;              // Make Resistor pull UP
    P2IES |= BIT1;              // Senstitive to H-to-L

    // SW2 interrupt - P1.1
    P1DIR &= ~BIT1;             // Make P1.1 as an input (SW2)
    P1REN |= BIT1;              // Enable resistor
    P1OUT |= BIT1;              // Make Resistor pull UP
    P1IES |= BIT1;              // Senstitive to H-to-L

    // SPI PORTs configuration: UCB0
    P3SEL |= BIT2;              // P3.2 = SCLK (1)
    P3SEL |= BIT0;              // P3.0 = SIMO (1)
    P3SEL |= BIT1;              // P3.1 = SOMI (1)

    // ADD STE LINE
    P2SEL &= ~BIT7;             // P2.7 as GPIO, an independent pin for SPI STE
    PORT_CS;                    // P2.7 CHIP SELECT as an output, active LOW
    CS_PROTEUS_HIGH;            // Initially state: Inactive when HIGH

    P8SEL &= ~BIT2;             // P8.2 = SPI_INT/WAKE_UP (0), one function only GPIO, no need to do this step
    PIN_SPI_INT;                // This #define is fixing the INITIALLY STATE: P8.2 as an INPUT (pin is SPI_INT)

    // PIN POWER CONSUMPTION, TOGGLE BETWEEN TRANSITIONS, PIN 3.7
    PIN_PW_OUTPUT;
    PIN_PW_OFF;                 // Init  PIN for PW consumption LOW


    //PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O, 0b = I/O pin configuration is not locked and defaults to its reset condition. PAG 123 Family User Guide
    UCB0CTL1 &= ~UCSWRST;      // Take B1 out of SW Reset

    // Enable IRQs
    P2IE |= BIT1;               // Enable P2.1 IRQ of the SW1 to trigger the transmission of a cmd
    P2IFG &= ~BIT1;             // Clear IRQ Flag

    P1IE |= BIT1;               // Enable P1.1 IRQ of the SW2 to trigger the transfer of data
    P1IFG &= ~BIT1;             // Clear IRQ Flag
    TA0_OUTPUT;
    TA1_OUTPUT;
    TA0_OFF;        // DEBUG TIMERS
    TA1_OFF;

    // Timers A0 and A1 for T idle and T connected
    initTimerIdle();
    initTimerConnected();
    //STOP_T_IDLE;
    STOP_T_CONNECTED;


    __enable_interrupt();       // Enable Maskable



    while(1){

        switch (currentState){
            case STATE_IDLE:
                handle_idle_state();
                break;
            case STATE_CONNECTED:

                handle_connected_state();

                break;
            case STATE_SLEEP:
                handle_sleep_state();
                break;
            default:
                currentState = STATE_IDLE;
                break;
        }

    }


return 0;
}

void delay_seconds(unsigned int seconds){
    while (seconds --){
        __delay_cycles(1000000);    // 1 second of delay with the clck 1 MHz.
    }
}





// ISR -------------------------------------//
#pragma vector = PORT2_VECTOR               // SWITCH S1 = P2.1
__interrupt void ISR_Port2_S1(void){
    P2IFG &= ~BIT1;
    if ((currentState == STATE_SLEEP) | (currentState == STATE_IDLE)){
        currentEvent = EVENT_S1_PRESSED;
    }


}


#pragma vector = PORT1_VECTOR               // SWITCH S2 = P1.1
__interrupt void ISR_Port1_S2(void){
    // Transfer data
    // At this point the channel should be already open, check flag
    P1IFG &= ~BIT1;

}
