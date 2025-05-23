#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "sensor_general.h"
#include "timer.h"

//#include "IOExpanderDriver.h"
//#include "actuator.h"
//#include "error_checks.h"
//#include "i2c.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000


// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 

// Sets ADC channels for current sense (these currently cannot be tested without board)
adcc_channel_t current_sense_5v = channel_ANC3; // NOTE: adcc.h was modified to add register for pin C3 adc 
adcc_channel_t current_sense_12v = channel_ANC2; 


#define PRES_OX_TIME_DIFF_ms 50 // 4 Hz
#define PRES_FUEL_TIME_DIFF_ms 50 // 64 Hz
#define PRES_CC_1_TIME_DIFF_ms 50 // 64 Hz
// #define PRES_CC_2_TIME_DIFF_ms 16 // 64 Hz // may be unneccesary 
#define HALLSENSE_FUEL_TIME_DIFF_ms 250 // 4 Hz
#define HALLSENSE_OX_TIME_DIFF_ms 250 // 4 Hz


adcc_channel_t pres_cc1 = channel_ANB0;
// adcc_channel_t pres_cc2 = channel_ANB1;
adcc_channel_t pres_ox = channel_ANB2;
adcc_channel_t pres_fuel = channel_ANB3;
adcc_channel_t hallsense_ox = channel_ANB4;
adcc_channel_t hallsense_fuel = channel_ANB5;

double fuel_pres_low_pass = 0;
double ox_pres_low_pass = 0;
double cc1_pres_low_pass = 0;
// double cc2_pres_low_pass = 0;
uint8_t fuel_pres_count = 0;
uint8_t ox_pres_count = 0;
uint8_t cc1_pres_count = 0;
// uint8_t cc2_pres_count = 0;


volatile bool seen_can_message = false;
volatile bool seen_can_command = false;

// memory pool for the CAN tx buffer
uint8_t tx_pool[200];

/* TODO
 - Double check voltage at PT ADC
    - Single 100ohm resistor in series; PT behaves like current source(?), so just use ohm's law 
 - DP FVRCON
 - const float vref = 4.096; 
- check if crystal oscillator has correct specfiications (see table 45-9 in datashset) 
 
 - made changes to rocketlib in adcc.h
 - add conditional compilation to add CCPT2 functionality 
 */

// 7.2.1.4 --> 4xPLL 
 
// Sets fixed reference voltage for ADC, see section 35.0 
//FVRCON = 0xC3;
//RSTOSC = 2;

// sets oscillator to use external crystal in 4xPLL mode, section 5.2 
// Configuration word 1L
//uint8_t* OSC_SET = 0x300000; 

//*OSC_SET = 0b 0 010 0 010; 



// interrupt handler for millis and timer.h
static void __interrupt() interrupt_handler() {
    if (PIR5) {
        can_handle_interrupt();
    }

    // Timer0 has overflowed - update millis() function
    // This happens approximately every 500us
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        timer0_handle_interrupt();
        PIR3bits.TMR0IF = 0;
    }
}

// Send a CAN message with nominal status
static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    
    // status_ok indicates no errors
    uint32_t gen_err_bitfield = 0;
    uint16_t board_specific_err_bitfield = 0;
    
    // [TODO] verify priority level of status_ok CAN message
    build_general_board_status_msg(PRIO_LOW, millis(), gen_err_bitfield, board_specific_err_bitfield, &board_stat_msg);

    txb_enqueue(&board_stat_msg);
}

// EDIT THIS - NASH: removed anything to do with actuator, left LEDs and board reset
static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);
    int dest_id = -1;
    int cmd_type = -1;
    // ignore messages that were sent from this board
    if (get_board_type_unique_id(msg) == BOARD_TYPE_UNIQUE_ID) {
        return;
    }

    // make able to handle multiple actuator
    switch (msg_type) {

        case MSG_LEDS_ON:
            LED_ON_G();
            LED_ON_Y();
            LED_ON_R();
            break;

        case MSG_LEDS_OFF:
            LED_OFF_G();
            LED_OFF_Y();
            LED_OFF_R();
            break;

        case MSG_RESET_CMD:
            if (check_board_need_reset(msg)) {
                RESET();
            }
            break;

        // all the other ones - do nothing
        default:
            break;
    }
}



int main(int argc, char **argv) {
    // MCC generated initializer
    // OSCCON1 register was changed to enable 4xPLL 
    // Sets fixed reference voltage for ADC to 4.096V, see section 35.0 
    SYSTEM_Initialize();
    
    LED_init();

    // init our millisecond function
    timer0_init();
    uint32_t last_millis = millis();
    uint32_t last_message_millis = 0;
    
    uint32_t last_pres_cc_1_millis = millis();
    // uint32_t last_pres_cc_2_millis = millis();
    uint32_t last_pres_ox_millis = millis();
    uint32_t last_pres_fuel_millis = millis();
    uint32_t last_hallsense_ox_millis = millis();
    uint32_t last_hallsense_fuel_millis = millis();

    // Enable global interrupts
    INTCON0bits.GIE = 1;
    
    // Set up CAN TX
    TRISC1 = 0;
    RC1PPS = 0x33;
    
    // Set up CAN RX
    TRISC0 = 1;
    ANSELC0 = 0;
    CANRXPPS = 0x10;
    
    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);
    
    // set up CAN tx buffer
    txb_init(tx_pool, sizeof(tx_pool), can_send, can_send_rdy);
    
    while (1) {
        
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }
        if (seen_can_command) {
            seen_can_command = false;
        }
        
        if ((millis() - last_message_millis) > MAX_BUS_DEAD_TIME_ms) {
            RESET();
        }
        
        // heartbeat Red LED to toggle every 500ms if status_ok
        if (millis() - last_millis > STATUS_TIME_DIFF_ms) {

            // check for general board status
            bool status_ok = true;
            // [TODO] add these, only current check for board, not voltage
            status_ok &= check_5v_current_error(current_sense_5v);
            status_ok &= check_12v_current_error(current_sense_12v);

            // if there was an issue, a message would already have been sent out
            if (status_ok) {
                send_status_ok();
            }
            
            // Red LED flashes during safe state.
            LED_heartbeat_R();
            last_millis = millis();
        }

#if PRES_FUEL_TIME_DIFF_ms
        if (millis() - last_pres_fuel_millis > PRES_FUEL_TIME_DIFF_ms) {
            last_pres_fuel_millis = millis();
            uint16_t fuel_pressure = update_pressure_psi_low_pass(pres_fuel, &fuel_pres_low_pass);
            if ((fuel_pres_count & 0xf) == 0) {
                can_msg_t sensor_msg;
                // TODO adapt new canlib build_analog_data_msg with new parameters
                build_analog_data_msg(
                    millis(), SENSOR_PRESSURE_FUEL, fuel_pressure, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            fuel_pres_count++;
        }
#endif

#if PRES_CC_1_TIME_DIFF_ms
        if (millis() - last_pres_cc_1_millis > PRES_CC_1_TIME_DIFF_ms) {
            last_pres_cc_1_millis = millis();
            uint16_t cc_pressure = update_pressure_psi_low_pass(pres_cc1, &cc1_pres_low_pass);
            if ((cc1_pres_count & 0xf) == 0) {
                can_msg_t sensor_msg;
                // TODO adapt new canlib build_analog_data_msg with new parameters
                build_analog_data_msg(millis(), SENSOR_PRESSURE_CC, cc_pressure, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            cc1_pres_count++;
        }
#endif
        
        // [TODO] do we have PRES_CC_2_TIME_DIFF_ms directive
        
#if HALLSENSE_FUEL_TIME_DIFF_ms
        if (millis() - last_hallsense_fuel_millis > HALLSENSE_FUEL_TIME_DIFF_ms) {
            last_hallsense_fuel_millis = millis();
            uint16_t hallsense_fuel_flux = get_hall_sensor_reading(hallsense_fuel);
            can_msg_t sensor_msg;
            // TODO adapt new canlib build_analog_data_msg with new parameters
            build_analog_data_msg(PRIO_MEDIUM, millis(), SENSOR_HALL_FUEL_INJ, hallsense_fuel_flux, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif
        
#if HALLSENSE_OX_TIME_DIFF_ms
        if (millis() - last_hallsense_ox_millis > HALLSENSE_OX_TIME_DIFF_ms) {
            last_hallsense_ox_millis = millis();
            uint16_t hallsense_ox_flux = get_hall_sensor_reading(hallsense_ox);
            can_msg_t sensor_msg;
            // TODO adapt new canlib build_analog_data_msg with new parameters
            build_analog_data_msg(millis(), SENSOR_HALL_OX_INJ, hallsense_ox_flux, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif
        
#if PRES_OX_TIME_DIFF_ms
        if (millis() - last_pres_ox_millis > PRES_OX_TIME_DIFF_ms) {
            last_pres_ox_millis = millis();
            uint16_t ox_pressure = update_pressure_psi_low_pass(pres_ox, &ox_pres_low_pass);
            if ((ox_pres_count & 0xf) == 0) {
                can_msg_t sensor_msg;
                // TODO adapt new canlib build_analog_data_msg with new parameters
                build_analog_data_msg(millis(), SENSOR_PRESSURE_OX, ox_pressure, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            ox_pres_count++;
        }
#endif
        // send any queued CAN messages
        txb_heartbeat();
    }
    
    return (EXIT_SUCCESS);
}




