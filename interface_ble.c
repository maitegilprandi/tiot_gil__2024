#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
//#include <stdlib.h>
#include "interface_ble.h"




volatile State_t currentState = STATE_IDLE;     // Global to the project
volatile Event_t currentEvent = EVENT_NONE;     // Global to the project

int FS_BTMAC_available = 0; // Global to this file

uint8_t cmd_read[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // // Global to the project
int data_to_send = 0;   // Global to the project

uint8_t payload_cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t check_sum = 0x00;
uint8_t length_lsb = 0x00;

int error_arr[15] = {0};
int timer_flag_idle = 0;
int timer_flag_connected = 0;

int phy_updated_ok = 0;

uint8_t BTMAC_CENTRAL_ARR[] = {0x9A, 0xE4, 0x32, 0xDA, 0x18, 0x00};


uint8_t Get_CRC8(uint8_t * bufP, uint16_t len){
    uint8_t crc = 0x00;
    uint16_t i;
    for (i = 0; i < len; i++){
        crc ^= bufP[i];
    }
    return crc;
}

// Functions Timers: Timer T idle and Timer T connected: to leave IDLE and CONNECTED states respectively.

void initTimerIdle(){                           // USES TIMER A0
    TA0CTL = TASSEL_1 | ID_3 | MC_1 | TACLR;    // Source CLK: ACLK 32.768 KHz /  ID_3: CLK divider 8 / MC_1: UP MODE / Timer clear
    TA0CCR0 = SEG_T_IDLE * (32678 / 8);           // Ticks per overflow, or SEG_T_IDLE for this case. MAX 16s.
    TA0CCTL0 = CCIE;                            // Enable interrupt for CCR0
}
void initTimerConnected(){                      // USES TIMER A1
    TA1CTL = TASSEL_1 | ID_3 | MC_1 | TACLR;    // Source CLK: ACLK 32.768 KHz /  ID_3: CLK divider 8 / MC_1: UP MODE / Timer clear
    TA1CCR0 = SEG_T_CONNECTED * (32678 / 8);      // Ticks per overflow, or SEG_T_IDLE for this case. MAX 16s.
    TA1CCTL0 = CCIE;                            // Enable interrupt for CCR0
}



#pragma vector = TIMER0_A0_VECTOR                // Timer A0 interprets TIMER IDLE, when ISR is called state should go from IDLE to SLEEP
__interrupt void Timer_A0_Idle_ISR(){
    TA0CCTL0 &= ~CCIFG;                         // Clear Flag
    TA0_TOGGLE;
    timer_flag_idle = 1;
    if ((currentState == STATE_IDLE)| (currentState == STATE_SLEEP)){
        currentEvent = EVENT_T_IDLE;
    }
}
#pragma vector = TIMER1_A0_VECTOR                // Timer A1 interprets TIMER CONNECTED, when ISR is called sate should go from CONNECTED to IDLE
__interrupt void Timer_A1_Connected_ISR(){
    TA1CCTL0 &= ~CCIFG;                         // Clear Flag
    TA1_TOGGLE;
    timer_flag_connected = 1;
    if (currentState == STATE_CONNECTED){
        currentEvent = EVENT_T_CONNECTED;
    }
}





// Functions machine state
void handle_idle_state(void){

    PIN_SPI_INT;                                                                                    // Pin WakeUP/SPI_INT as an input so data can be read from the Host.
    if (currentEvent == EVENT_T_IDLE){
        send_command_host(CMD_SLEEP_REQ, NULL_BYTE, NULL_BYTE, payload_cmd, NULL_BYTE);             // Send CMD_SLEEP_REQ

        timer_flag_idle = 0;
        while ( (!(P8IN & BIT2)) && (!(timer_flag_idle)) );    // or timer                                                       // Wait for CMD_SLEEP_CNF
        if (P8IN & BIT2){
            read_command_host();
            if (cmd_read[1] != CMD_SLEEP_CNF){
                error_arr[0] = 1;                                                                       // error array: cmd not expected >> cmd expected vs cmd received
                RESTART_T_IDLE;
            }else{
                PIN_PW_TOGGLE;                                                                              // TOGGLE GPIO: IDLE TO SLEEP
                state_transition(EVENT_T_IDLE);                                                             // It will go to SLEEP STATE
            }
        }

    }
    if (currentEvent == EVENT_S1_PRESSED){                                                          // When button SW1 pressed, it will wait for a connection request.
        // Timer T IDLE is already triggered
        timer_flag_idle = 0;
        while ( (!(P8IN & BIT2)) && (!(timer_flag_idle)) ); // READ CMD_GETSTATE_CNF   This does not work  && (!(TA0CCTL0 & CCIFG))
        if (P8IN & BIT2){
           read_command_host();
           if (cmd_read[1] != CMD_GETSTATE_CNF){
               error_arr[9] = 1;                                                                   // error array: cmd not expected >> cmd expected vs cmd received
               RESTART_T_IDLE;                                                                     // Trigger T idle
           }else{
               // WAIT FOR A CONNECTION OR LEAVE BY TIMER A0
               // Interrupt vector from Timer T idle already triggered in the Transition sleep - idle
               timer_flag_idle = 0;
               while ( (!(P8IN & BIT2)) && (!(timer_flag_idle)) );                                     // Interrupt vector from Timer T idle already triggered in the Transition sleep - idle
               if (P8IN & BIT2){
                   read_command_host();
                   if ((cmd_read[1] != CMD_CONNECT_IND) | (cmd_read[4] != 0x00)){
                       error_arr[2] = 1;                                                                   // error array: cmd not expected >> cmd expected vs cmd received
                       RESTART_T_IDLE;                                                                     // Trigger T idle
                   }else{
                       RESTART_T_IDLE;
                       timer_flag_idle = 0;
                       while ( (!(P8IN & BIT2)) && (!(timer_flag_idle)) );
                       if (P8IN & BIT2){
                           read_command_host();
                           if ((cmd_read[1] != CMD_CHANNELOPEN_RSP) | (cmd_read[4] != 0x00)){
                               error_arr[3] = 1;                                                               // error array: cmd not expected >> cmd expected vs cmd received
                               RESTART_T_IDLE;                                                                 // Trigger T idle
                           }else{                                                                              // Both commands expected were received, it will go to STATE CONNECTED
                               PIN_PW_TOGGLE;                                                                  // Toggle GPIO - from IDLE to CONNECTED
                               state_transition(EVENT_S1_PRESSED);                                             // It will go to CONNECTED
                               currentEvent = EVENT_S1_PRESSED;                                                // Maintaining the EVENT_S1_PRESSED it will receive DATA-cmd and send DATA
                           }
                       }
                   }
               }else{
                  // currentEvent = EVENT_T_IDLE;
               }


           }
        }else{
           // currentEvent = EVENT_T_IDLE;
        }// SI SALE POR TIMER A0 en VEZ DE RECIBIR GET STATE CNF

        //RESTART_T_IDLE;


    }
}

// Compare the bytes received in payload with the BTMAC address
bool check_BTMAC(const uint8_t *array){                                                             // Combine bytes 2 to 7 into a 48-bit number to compare it to the BTMAC saved.
    uint64_t combined = ((uint64_t)array[2] << 40) |
            ((uint64_t)array[3] << 32) |
            ((uint64_t)array[4] << 24) |
            ((uint64_t)array[5] << 16) |
            ((uint64_t)array[6] << 8) |
            (uint64_t)array[7];
    const uint64_t target = BTMAC_CENTRAL;
    return (combined == target);
}


bool update_phy(uint8_t PHY){       // IF OK: RETURNS 1
    check_sum = START_SIGNAL ^ CMD_PHYUPDATE_REQ ^ 0x01 ^ NULL_BYTE ^ PHY;
    payload_cmd[0] = PHY;
    send_command_host(CMD_PHYUPDATE_REQ, 0x01, NULL_BYTE, payload_cmd, check_sum);             // send CMD_DISCONNECT_REQ

    RESTART_T_CONNECTED; // restart t_connected
    timer_flag_connected = 0;
    while ( (!(P8IN & BIT2)) && (!(timer_flag_connected)) );                            // Interrupt vector from Timer T connected already triggered in the Transition idle - connected
    // Wait for CMD_PHYUPDATE_CNF
     if (P8IN & BIT2){
         read_command_host();
         if ((cmd_read[1] != CMD_PHYUPDATE_CNF) | (cmd_read[4] != 0x00)){
             error_arr[10] = 1;                                                  // error array: cmd not expected >>
             return 0;
         }else{
             RESTART_T_CONNECTED; // restart t_connected
             timer_flag_connected = 0;
             while ( (!(P8IN & BIT2)) && (!(timer_flag_connected)) );                                     // Interrupt vector from Timer T connected already triggered in the Transition idle - connected
              // Wait for CMD_PHYUPDATE_IND
             if (P8IN & BIT2){
                   read_command_host();
                   if ((cmd_read[1] != CMD_PHYUPDATE_IND) | (cmd_read[4] != 0x00) | (cmd_read[5]!= PHY)| (cmd_read[6]!= PHY)){
                       error_arr[11] = 1;                                                  // error array: cmd not expected >>
                       return 0;
                   }else{
                       RESTART_T_CONNECTED;
                       return 1;    // IF EVERYTHING OK: it will return 1

                   }
              }else{
                  timer_flag_connected = 0;
                  currentEvent = EVENT_T_CONNECTED;
                  return 0;
              }
         }
     }else{
         timer_flag_connected = 0;
         currentEvent = EVENT_T_CONNECTED;
         return 0;
     }
}


void handle_connected_state(void){

    if (currentEvent == EVENT_S1_PRESSED){

        /*if (update_phy(PHY_2MB) == 1){  // phy updated correctly // PHY_2MB or PHY_LEcoded
            phy_updated_ok = 1;
        }*/


        timer_flag_connected = 0;
        while ( (!(P8IN & BIT2)) && (!(timer_flag_connected)) );                                     // Interrupt vector from Timer T connected already triggered in the Transition idle - connected

         if (P8IN & BIT2){                                                                      // Receiving a DATA from Central
             PIN_PW_TOGGLE; // TO IDENTIFY RX DATA
             read_command_host();
             RESTART_T_CONNECTED;
             if (cmd_read[1] == CMD_DATA_IND){
                 if ((cmd_read[11] == 0xAB) & (cmd_read[12] == 0xCD)){                          // INTERPRET DATA-CMD = 0xABCD
                     PIN_PW_TOGGLE; // TO IDENTIFY RX DATA
                     STOP_T_CONNECTED;                                                          // Turn off Timer T_connected
                     data_to_send = 1;

                     while (data_to_send == 1){                                                 // SEND COUNTER 0 - 1000 - 0
                         send_packet_data();
                         while(!(P8IN & BIT2));  // or timer                                    // receive CMD_DATA_CNF
                         read_command_host();
                         if ((cmd_read[1] != CMD_DATA_CNF) | (cmd_read[4] != 0x00)){
                             error_arr[4] = 1;                                                  // error array: cmd not expected >> cmd expected vs cmd received
                         }
                         while(!(P8IN & BIT2));  // or timer                                    // receive CMD_TXCOMPLETE_RSP
                         read_command_host();
                         if ((cmd_read[1] != CMD_TXCOMPLETE_RSP) | (cmd_read[4] != 0x00)){
                             error_arr[5] = 1;                                                  // error array: cmd not expected >> cmd expected vs cmd received
                         }
                     }
                     PIN_PW_TOGGLE;     // END OF TX DATA, identify this Transmisson Connected
                     RESTART_T_CONNECTED;
                     TRIG_T_CONNECTED;                                                          // Trigger Timer T_connected
                     currentEvent = EVENT_NONE;
                 }else{
                     RESTART_T_CONNECTED;
                     TRIG_T_CONNECTED;                                                          // Trigger Timer T_connected
                 }
             }else if(cmd_read[1] == CMD_DISCONNECT_IND){                                       // Identify if command CMD_DISCONNECT_REQ is sent by the Central device
                 PIN_PW_TOGGLE;                                                                 // Toggle GPIO - from CONNECTED to IDLE
                 state_transition(EVENT_T_CONNECTED);                                           // It will go to IDLE STATE. Timer IDLE in Transition function
             }else{
                 error_arr[8] = 1;
                 currentEvent = EVENT_T_CONNECTED;                                           // It will go to IDLE STATE. Timer IDLE in Transition function
             }                                                                                  // WHAT IF OTHER COMMANDS ARE RECIEVED? nothing for now
         }else{
             timer_flag_connected = 0;
             currentEvent = EVENT_T_CONNECTED;
         }
    }

    if (currentEvent == EVENT_T_CONNECTED){                                                         // Means it will send disconnect request to Central and go to IDLE
        send_command_host(CMD_DISCONNECT_REQ, NULL_BYTE, NULL_BYTE, payload_cmd, 0x05);             // send CMD_DISCONNECT_REQ

        timer_flag_connected = 0;
        while ( (!(P8IN & BIT2)) && (!(timer_flag_connected)) );                                     // Interrupt vector from Timer T connected already triggered in the Transition idle - connected
        if (P8IN & BIT2){
            read_command_host();
            if ((cmd_read[1] != CMD_DISCONNECT_CNF) | (cmd_read[4] != 0x00)){
                error_arr[6] = 1;                                                                       // error array: cmd not expected >> cmd expected vs cmd received
                currentState = STATE_IDLE;
                currentEvent = EVENT_NONE;
            }else{
                while(!(P8IN & BIT2));  // or timer                                                     // receive CMD_DISCONNECT_IND
                read_command_host();
                if ((cmd_read[1] != CMD_DISCONNECT_IND) | (cmd_read[4] != 0x16)){                       // REASON WHY DISCONNECTED: 0x16: Host terminated connection
                    error_arr[7] = 1;                                                                   // error array: cmd not expected >> cmd expected vs cmd received
                    currentState = STATE_IDLE;
                    currentEvent = EVENT_NONE;
                }else{
                    PIN_PW_TOGGLE;                                                                      // Toggle GPIO - from CONNECTED to IDLE
                    state_transition(EVENT_T_CONNECTED);                                                // It will go to IDLE state
                }
            }
        }

    }
}

void handle_sleep_state(void){

    if (currentEvent == EVENT_S1_PRESSED){
        PIN_WAKE_UP;                                                                                // Toggle P8.2 as an OUTPUT
        __delay_cycles(5);
        WAKE_UP_PROTEUS;                                                                            // Active WAKE-UP PIN
        PIN_PW_TOGGLE;                                                                              // Toggle GPIO - from SLEEP to IDLE

        state_transition(EVENT_S1_PRESSED);                                                         // Go to IDLE state
        currentEvent = EVENT_S1_PRESSED;                                                            // The event S1 remains, so in next step: will try to establish connection with the Central
    }
}



// Handle transition between states:
void state_transition(Event_t event){
    switch (event){

        case EVENT_S1_PRESSED:

            if (currentState == STATE_IDLE){
                currentState = STATE_CONNECTED;
                STOP_T_IDLE;                                                                    // Turn OFF Timer Tidle
                LED1_ON;                                                                        // Turn on LED 1 of MSP430 while channel is open.
                RESTART_T_CONNECTED;
                TRIG_T_CONNECTED;                                                               // Trigger T connected in case there is no data transmitted from Central

            }else if (currentState == STATE_SLEEP){
                currentState = STATE_IDLE;
                LED2_OFF;                                                                       // Turn off LED 2 MSP430
                RESTART_T_IDLE;
                TRIG_T_IDLE;
            }
            break;

        case EVENT_T_IDLE:
            if (currentState == STATE_IDLE){
                currentState = STATE_SLEEP;
                LED2_ON;                                                                        // Turn on a LED 2 of msp430 to identify SLEEP MODE STATE
                STOP_T_IDLE;                                                                    // Turn off timer T_idle
            }
            break;

        case EVENT_T_CONNECTED:
            if (currentState == STATE_CONNECTED){
                currentState = STATE_IDLE;
                LED1_OFF;                                                                      // Turn OFF LED 1 MSP430
                STOP_T_CONNECTED;                                                              // Turn OFF Timer T_connected
                RESTART_T_IDLE;
                TRIG_T_IDLE;                                                                   // Trigger T_idle
            }
            break;

        case EVENT_RESET:
            currentState = STATE_IDLE;
            break;

        default:
            currentState = STATE_IDLE;
            break;
    }
    currentEvent = EVENT_NONE;
}




// Functions Tx/Rx of CMDs between module & host
void send_packet_data(void){
    static int value = 0;           // Static so the value is not updated every time send_packet_data is called.
    static int starting_pt = 1;
    static int ramp_down = 0;
    uint8_t length_payload;
    uint8_t last_byte_CS = 0x00;    // check sum
    uint8_t count_byte = 0x00;      // number of bytes that are being send.
    uint8_t value_hex_lsb;
    uint8_t value_hex_msb;

    CS_PROTEUS_HIGH;
    __delay_cycles(100);
    CS_PROTEUS_LOW;

    // Trigger transfer of data
    UCB0TXBUF = START_SIGNAL;               // START SIGNAL
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = CMD_DATA_REQ;                   // CMD_DATA_REQ
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    length_payload = BYTES_PAYLOAD_HEX;         // 242 bytes & F2 HEX

    if (value > BYTES_PAYLOAD) {  // > 242, identify when the value is not at the starting point
        starting_pt = 0;
    }
    if ( (starting_pt == 0) & (2*value < BYTES_PAYLOAD) ){ // last packet to be sent
        length_payload = (uint8_t)(2*(value+1));    // +1 because it is down to 0
        data_to_send = 0;
    }
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = length_payload;             // LENGTH LSB: 243 max payload
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = 0x00;                       // LENGTH MSB
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    while (count_byte < length_payload){   // Send always 2 bytes per number

        while(!(UCB0IFG & UCTXIFG));
        value_hex_lsb = (uint8_t)value;
        value_hex_msb = (uint8_t)(value >> 8);
        UCB0TXBUF = value_hex_lsb;

        last_byte_CS = last_byte_CS ^ value_hex_lsb;
        while(!(UCB0IFG & UCRXIFG));
        UCB0RXBUF;

        while(!(UCB0IFG & UCTXIFG));    // 2nd byte MSB
        UCB0TXBUF = value_hex_msb;
        last_byte_CS = last_byte_CS ^ value_hex_msb;
        while(!(UCB0IFG & UCRXIFG));
        UCB0RXBUF;

        count_byte++;
        count_byte++;

        if (ramp_down == 0){
            value++;
        }else{
            value--;    // ramp down
        }
        if (value == MAX_COUNTER+1){      // Reach max counter = 1000, +1 because it has just executed value++
            value = MAX_COUNTER - 1;
            ramp_down = 1;
        }
    }
    last_byte_CS = last_byte_CS ^ START_SIGNAL ^ CMD_DATA_REQ ^ length_payload ^ 0x00;   // ^ Start signal ^ command data_req ^ length LSB ^ length msb = 0x00

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = last_byte_CS;               // LAST BYTE: CS
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    CS_PROTEUS_HIGH;
    __delay_cycles(20);

    if (data_to_send == 0){     // Last package was sent: INIT the variables for next transmission of the counter.
        value = 0;              // These VAR are defined static so the value is not updated every time send_packet_data is called.
        starting_pt = 1;
        ramp_down = 0;
    }
}



void read_command_host(void){
    uint16_t LENGTH_16_rx;
    uint8_t LENGTH_LSB;
    uint8_t LENGTH_MSB;
    unsigned int position_rx;
    position_rx = 0;
    int i;
    // RECEIVE COMMAND
    CS_PROTEUS_HIGH;
    __delay_cycles(10);
    CS_PROTEUS_LOW;

    UCB0TXBUF = 0xFF;                     // Send 0xFF
    while(!(UCB0IFG & UCRXIFG));
    cmd_read[position_rx] = UCB0RXBUF;   // RX 1st byte
    position_rx++;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = 0xFF;
    while(!(UCB0IFG & UCRXIFG));
    cmd_read[position_rx] = UCB0RXBUF;   // RX 2nd byte
    position_rx++;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = 0xFF;
    while(!(UCB0IFG & UCRXIFG));
    cmd_read[position_rx] = UCB0RXBUF;   // RX 3rd byte, LSB of LENGTH
    LENGTH_LSB = cmd_read[position_rx];
    position_rx++;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = 0xFF;
    while(!(UCB0IFG & UCRXIFG));
    cmd_read[position_rx] = UCB0RXBUF;   // RX 4th byte, MSB of LENGTH
    LENGTH_MSB = cmd_read[position_rx];
    position_rx++;

    LENGTH_16_rx = LENGTH_MSB << 8u;
    LENGTH_16_rx = LENGTH_16_rx + LENGTH_LSB;

    for (i=0; i < LENGTH_16_rx; i++){
        while(!(UCB0IFG & UCTXIFG));
        UCB0TXBUF = 0xFF;
        while(!(UCB0IFG & UCRXIFG));
        cmd_read[position_rx] = UCB0RXBUF;   // RX 5th, 6th, 7th..... bytes
        position_rx++;
    }

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = 0xFF;                     // Send 0xFF
    while(!(UCB0IFG & UCRXIFG));
    cmd_read[position_rx] = UCB0RXBUF;   // Receive last byte: CS

    CS_PROTEUS_HIGH;
    __delay_cycles(20); // Important delay to assure that if the next step is reading SPI_INT, this signal is already low.

}


void send_command_host(uint8_t CMD, uint8_t LENGTH_LSB_tx, uint8_t LENGTH_MSB_tx, uint8_t PAYLOAD[6], uint8_t CHECK_SUM){

    uint16_t LENGTH_16_tx;
    int j;

    CS_PROTEUS_HIGH;
    __delay_cycles(20);
    CS_PROTEUS_LOW;

    // Trigger transfer of data
    UCB0TXBUF = START_SIGNAL;               // START SIGNAL
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;                              // CHECK THEN IF 0xFF

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = CMD;                        // Send COMMAND HEX
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = LENGTH_LSB_tx;              // Send LENGTH LSB HEX
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = LENGTH_MSB_tx;              // Send LENGTH MSB HEX
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    LENGTH_16_tx = LENGTH_MSB_tx << 8u;
    LENGTH_16_tx = LENGTH_16_tx + LENGTH_LSB_tx;

    for (j=0; j < LENGTH_16_tx; j++){
        while(!(UCB0IFG & UCTXIFG));
        UCB0TXBUF = PAYLOAD[j];             // TX Payload: 5th, 6th, 7th..... bytes
        while(!(UCB0IFG & UCRXIFG));
        UCB0RXBUF;
    }

    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = CHECK_SUM;                  // Send last byte CHECK SUM HEX
    while(!(UCB0IFG & UCRXIFG));
    UCB0RXBUF;

    CS_PROTEUS_HIGH;
   __delay_cycles(20); // Important delay to assure that if the next step is reading SPI_INT, this signal is already low.

}
