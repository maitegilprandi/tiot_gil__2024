/*
 * interface_ble.h
 *
 *  Created on: Jul 5, 2024
 *      Author: LENOVO
 */

#ifndef INTERFACE_BLE_H_
#define INTERFACE_BLE_H_

#include <stdbool.h>

// SPI PORT 3, SPI MASTER: MSP430F5529LP - SPI SLAVE: BT PROTEUS-III WE - config below
// CHIP SELECT FOR PROTEUS-III SPI BOARD

// LEDs MSP-EXPF5529LP
#define LED1    BIT0   // P1.0
#define LED2    BIT7   // P4.7

#define LED1_set        P1DIR |= LED1;
#define LED1_TOGGLE     P1OUT ^= LED1;
#define LED1_OFF        P1OUT &= ~LED1;
#define LED1_ON         P1OUT |= LED1;

#define LED2_set        P4DIR |= LED2;
#define LED2_TOGGLE     P4OUT ^= LED2;
#define LED2_OFF        P4OUT &= ~LED2;
#define LED2_ON         P4OUT |= LED2;


#define PORT_CS         P2DIR |= BIT7;   // P2.7 as output = CHIP SELECT
#define CS_PROTEUS_HIGH P2OUT |= BIT7;   // By default CS in 1
#define CS_PROTEUS_LOW  P2OUT &= ~BIT7;

// INPUT SPI_INT: When HIGH it announces that PROTEUS-III board has data available to send to Main MCU
// OUTPUT WAKE_UP: when HIGH the main MCU wakes up the PROTEUS-III SPI board.
#define PIN_SPI_INT         P8DIR &= ~BIT2;  // P8.2 as input when pin is SPI_INT
#define PIN_WAKE_UP         P8DIR |= BIT2;   // P8.2 as output when pin is WAKE_UP
#define WAKE_UP_PROTEUS     P8OUT |= BIT2;  // When active 8.2, as output, it wakes up the module Proteus III

// PIN GPIO TO TOGGLE BETWEEN STATE TRANSITIONS TO GET POWER CONSUMPTION: P3.7
#define PIN_PW_OUTPUT       P3DIR |= BIT7;  // P3.7 as an OUTPUT
#define PIN_PW_TOGGLE       P3OUT ^= BIT7;  // P3.7 Toggle to detect states transitions
#define PIN_PW_OFF          P3OUT &= ~BIT7; // P3.7 low at start


//--------------------------------------------------DEBUG------------------------------------//
// DEBUG TIMER A0: TIMER IDLE: P8.1
#define TA0_OUTPUT       P8DIR |= BIT1;  // P8.1 as an OUTPUT
#define TA0_TOGGLE       P8OUT ^= BIT1;  // P8.1 Toggle to detect timer transitions
#define TA0_OFF          P8OUT &= ~BIT1; // P8.1 low at start

// DEBUG TIMER A1: TIMER CONNECTED: P2.4
#define TA1_OUTPUT       P2DIR |= BIT4;  // P2.4 as an OUTPUT
#define TA1_TOGGLE       P2OUT ^= BIT4;  // P2.4 Toggle to detect timer transitions
#define TA1_OFF          P2OUT &= ~BIT4; // P2.4 low at start

//--------------------------------------------------DEBUG------------------------------------//



// PROTEUS-III-SPI USEFUL DEFINES
#define START_SIGNAL                0x02
#define NULL_BYTE                   0x00
// COMMANDS from PROTEUS-III-SPI

// Scan for other modules in range
#define CMD_SCANSTART_REQ      0x09
#define CMD_SCANSTART_CNF      0x49
#define CMD_SCANSTOP_REQ       0x0A
#define CMD_SCANSTOP_CNF       0x4A
#define CMD_GETDEVICES_REQ      0x0B
#define CMD_GETDEVICES_CNF      0x4B
#define CMD_RSSI_IND            0x8B
#define CMD_BEACON_RSP          0xCC

// Setup connections
#define CMD_CONNECT_REQ         0x06
#define CMD_CONNECT_CNF         0x46
#define CMD_CONNECT_IND         0x86
#define CMD_SECUIRTY_IND        0x88
#define CMD_CHANNELOPEN_RSP     0xC6
#define CMD_DISCONNECT_REQ      0x07
#define CMD_DISCONNECT_CNF      0x47
#define CMD_DISCONNECT_IND      0x87
#define CMD_PHYUPDATE_REQ       0x1A
#define CMD_PHYUPDATE_CNF       0x5A
#define CMD_PHYUPDATE_IND       0x9A
#define CMD_PASSKEY_REQ         0x0D
#define CMD_PASSKEY_CNF         0x4D
#define CMD_PASSKEY_IND         0x8D
#define CMD_DISPLAY_PASSKEY_IND 0xA4
#define CMD_NUMERIC_COMP_REQ    0x24
#define CMD_NUMERIC_COMP_CNF    0x64
#define CMD_GETBONDS_REQ        0x0F
#define CMD_GETBONDS_CNF        0x4F
#define CMD_DELETEBONDS_REQ     0x0E
#define CMD_DELETEBONDS_CNF     0x4E
#define CMD_ALLOWUNBONDEDCONNECTIONS_REQ    0x2D
#define CMD_ALLOWUNBONDEDCONNECTIONS_CNF    0x6D

//Transmit and receive data
#define CMD_DATA_REQ            0x04
#define CMD_DATA_CNF            0x44
#define CMD_TXCOMPLETE_RSP      0xC4
#define CMD_DATA_IND            0x84
#define CMD_SETBEACON_REQ       0x0C
#define CMD_SETBEACON_CNF       0x4C
#define CMD_BEACON_IND          0x8C

//Configure the module and modifying the device settings
#define CMD_SET_REQ             0x11
#define CMD_SET_CNF             0x51
#define CMD_GET_REQ             0x10
#define CMD_GET_CNF             0x50

// Manage the device state
#define CMD_GETSTATE_REQ        0x01
#define CMD_GETSTATE_CNF        0x41
#define CMD_RESET_REQ           0x00
#define CMD_RESET_CNF           0x40
#define CMD_SLEEP_REQ           0x02
#define CMD_SLEEP_CNF           0x42
#define CMD_SLEEP_IND           0x82
#define CMD_FACTORYRESET_REQ    0x1C
#define CMD_FACTORYRESET_CNF    0x5C
#define CMD_BOOTLOADER_REQ      0x1F
#define CMD_BOOTLOADER_CNF      0x5F

#define MAX_COUNTER             1000    // Counter will increase until this value.
#define BYTES_PAYLOAD           242     // Number of bytes sent in each package. Except the last package that will have less payload length.
#define BYTES_PAYLOAD_HEX       0xF2

#define BTMAC_CENTRAL           0x9AE432DA1800  // BTMAC address of the Central Proteus-III

//Timers: MAX VALUE 16 seconds. If more ISR must be updated to consider overflow counter
#define SEG_T_IDLE              8       // seconds of Timer Idle: Transition IDLE > SLEEP
#define SEG_T_CONNECTED         6       // seconds of Timer Connected: Transition CONNECTED > IDLE

// Timer IDLE uses Timer A0
#define STOP_T_IDLE         TA0CTL &= ~MC_3;//TA0CTL |= MC_0;     // Trigger STOP MODE
#define TRIG_T_IDLE         TA0CTL |= MC_1;     // Trigger UP MODE
#define RESTART_T_IDLE      TA0R = 0;           // Restart Timer counter, loosing previous configuration: divisor CLK and compare config
#define RESET_T_IDLE        TA0CTL |= TACLR;    // Reset Timer counter, loosing previous configuration: divisor CLK and compare config
// Timer CONNECTED uses Timer A1
#define STOP_T_CONNECTED    TA1CTL &= ~MC_3;//TA1CTL |= MC_0;     // Trigger STOP MODE
#define TRIG_T_CONNECTED    TA1CTL |= MC_1;     // Trigger UP MODE
#define RESTART_T_CONNECTED TA1R = 0;           // Restart timer counter, loosing previous configuration: divisor CLK and compare config
#define RESET_T_CONNECTED   TA1CTL |= TACLR;    // Reset timer counter, loosing previous configuration: divisor CLK and compare config



//PHY CONFIG in CMD_PHYUPDATE_REQ
#define PHY_1MB         0x01
#define PHY_2MB         0x02
#define PHY_LEcoded     0x04



// MACHINE STATE
typedef enum{
    STATE_IDLE,
    STATE_CONNECTED,
    STATE_SLEEP
}State_t;


typedef enum{
    EVENT_NONE,
    EVENT_S1_PRESSED,
    EVENT_T_IDLE,
    EVENT_T_CONNECTED,
    EVENT_RESET
}Event_t;

extern volatile State_t currentState;
extern volatile Event_t currentEvent;


// Tx/Rx of CMDs between module & host
extern uint8_t cmd_read[];
extern int data_to_send;


/**
 * @brief This function does....
 *
 * @param CMD ....
 * @param LENGTH_LSB_tx ....
 * @return No return for this function
 */

// Declaration of functions

void initTimerIdle();
void initTimerConnected();

uint8_t Get_CRC8(uint8_t * bufP, uint16_t len);
bool check_BTMAC(const uint8_t *array);

bool update_phy(uint8_t PHY);

void handle_idle_state(void);
void handle_connected_state(void);
void handle_sleep_state(void);
void state_transition(Event_t event);

void send_packet_data(void);
void read_command_host(void);

/**
 * @brief This function send predefined CMDs to the host. This CMDs can be found in Proteus manual.
 *
 * @param CMD: 8 bits Command representation in hexa.
 * @param LENGTH_LSB_tx: 8 bits LSB of the length of the payload.
 * @param LENGTH_MSB_tx: MSB is always 0x00. The max payload is represented with only 8 bits.
 * @param PAYLOAD[6]: array of bytes containing payload information. The max payload to send is 6 bytes for CMD_CONNECT_REQ
 * @param CHECK_SUM: 1 byte operation for the XOR of all the bytes involved in the command, including start signal
 * @return No return for this function
 */
void send_command_host(uint8_t CMD, uint8_t LENGTH_LSB_tx, uint8_t LENGTH_MSB_tx, uint8_t PAYLOAD[6], uint8_t CHECK_SUM);




#endif /* INTERFACE_BLE_H_ */

