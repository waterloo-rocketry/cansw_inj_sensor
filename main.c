#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
#include "sensor_general.h"
#include "timer.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000

// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500

#define PRES_CH0_TIME_DIFF_ms 6 // Sample at 160 Hz, CAN Transmit at 40 Hz (divisor 4)
#define PRES_CH1_TIME_DIFF_ms 50 // Sample at 20 Hz, CAN Transmit at 5 Hz (divisor 4)
#define PRES_CH2_TIME_DIFF_ms 50 // Sample at 20 Hz, CAN Transmit at 5 Hz (divisor 4)
#define PRES_CH3_TIME_DIFF_ms 50 // Sample at 20 Hz, CAN Transmit at 5 Hz (divisor 4)
#define PRES_CH4_TIME_DIFF_ms 50 // Sample at 20 Hz, CAN Transmit at 5 Hz (divisor 4)

#define PRES_CH0_DOWNSAMPLE_MASK 0x3 // 1-in-4
#define PRES_CH1_DOWNSAMPLE_MASK 0x3 // 1-in-4
#define PRES_CH2_DOWNSAMPLE_MASK 0x3 // 1-in-4
#define PRES_CH3_DOWNSAMPLE_MASK 0x3 // 1-in-4
#define PRES_CH4_DOWNSAMPLE_MASK 0x3 // 1-in-4

#define HALLSENSE_CH0_TIME_DIFF_ms 250 // Sample and transmit at 4 Hz
#define HALLSENSE_CH1_TIME_DIFF_ms 250 // Sample and transmit at 4 Hz
#define HALLSENSE_CH2_TIME_DIFF_ms 250 // Sample and transmit at 4 Hz

// Sets ADC channels for current sense (these currently cannot be tested without board)
// NOTE: adcc.h was modified to add register for pin C3 adc
adcc_channel_t current_sense_5v = channel_ANC4;
adcc_channel_t current_sense_12v = channel_ANC2;

adcc_channel_t pres_ch0_port = channel_ANA1; // J3 Connector
adcc_channel_t pres_ch1_port = channel_ANA0; // J4 Connector
adcc_channel_t pres_ch2_port = channel_ANB3; // J6 Connector
adcc_channel_t pres_ch3_port = channel_ANB2; // J8 Connector
adcc_channel_t pres_ch4_port = channel_ANB1; // J10 Connector

adcc_channel_t hallsense_ox = channel_ANC7; // J5 Connector
adcc_channel_t hallsense_fuel = channel_ANC6; // J9 Connector

double pres_ch0_low_pass = 0;
double pres_ch1_low_pass = 0;
double pres_ch2_low_pass = 0;
double pres_ch3_low_pass = 0;
double pres_ch4_low_pass = 0;

uint8_t ch0_pres_count = 0;
uint8_t ch1_pres_count = 0;
uint8_t ch2_pres_count = 0;
uint8_t ch3_pres_count = 0;
uint8_t ch4_pres_count = 0;

volatile bool seen_can_message = false;

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);

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

- Check if the analog msg sent from the overcurrent CAN message was done correctly
 - ADD PT_OUT_OF_RANGE check in error_checks
- change the PRES_TIME_DIFF_ms in sensor_general.h to 50ms
- change the register for RED led back
- Replace all can messages with build_board_stat_msg(... ... ...
    - Same goes to build_analog_data_message(... ... ...
- review message prio for analog can messages and those sent in error_checks

- Add SENSOR_HALL_OX, and SENSOR_HALL_INJ, E_PT_OUT_OF_RANGE
 *
 *
 *
 *
 * REGENERATE THE MCC
 * - we temporarily multiplied the XTAL_FREQ macro by x4, set it back to proper value
 * - - revert the 4xPLL setting?
 * change back the CANRX/TX pins for the inj board
 * test FVR
 * use ./rocketlib/scripts/format-cansw.sh to format code to be pretty
 * -
 * -
 */

int main(int argc, char **argv) {
    // MCC generated initializer
    // OSCCON1 register was changed to enable 4xPLL
    // Sets fixed reference voltage for ADC to 4.096V, see section 35.0
    SYSTEM_Initialize();

    LED_init(); // Turn all LEDs on

    // init our millisecond function
    timer0_init();
    uint32_t last_millis = millis();
    uint32_t last_message_millis = 0;

    // Randomize initial millis to lower peak CAN bus message rate
    uint32_t last_pres_ch0_millis = 1;
    uint32_t last_pres_ch1_millis = 2;
    uint32_t last_pres_ch2_millis = 3;
    uint32_t last_pres_ch3_millis = 4;
    uint32_t last_pres_ch4_millis = 5;

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

        if ((millis() - last_message_millis) > MAX_BUS_DEAD_TIME_ms) {
            RESET();
        }

        // heartbeat Red LED to toggle every 500ms if status_ok
        if (millis() - last_millis > STATUS_TIME_DIFF_ms) {
            // check for general board status
            uint32_t general_error = 0;
            uint16_t board_error = 0;

            general_error |= check_5v_current_error(current_sense_5v) << E_5V_OVER_CURRENT_OFFSET;
            general_error |= check_12v_current_error(current_sense_12v)
                             << E_12V_OVER_CURRENT_OFFSET;
            board_error |= check_PT_current_error(pres_ch0_port) << E_PT_OUT_OF_RANGE_OFFSET;
            board_error |= check_PT_current_error(pres_ch1_port) << E_PT_OUT_OF_RANGE_OFFSET;
            board_error |= check_PT_current_error(pres_ch2_port) << E_PT_OUT_OF_RANGE_OFFSET;
            board_error |= check_PT_current_error(pres_ch3_port) << E_PT_OUT_OF_RANGE_OFFSET;
            board_error |= check_PT_current_error(pres_ch4_port) << E_PT_OUT_OF_RANGE_OFFSET;

            can_msg_t status_msg;
            build_general_board_status_msg(
                PRIO_MEDIUM, (uint16_t)millis(), general_error, board_error, &status_msg
            );
            txb_enqueue(&status_msg);

            // Red LED flashes during safe state.
            LED_R = LED_R ^ LED_OFF;
            last_millis = millis();
        }

#if PRES_CH0_TIME_DIFF_ms
        if (millis() - last_pres_ch0_millis > PRES_CH0_TIME_DIFF_ms) {
            last_pres_ch0_millis = millis();
            uint16_t ch0_pressure = update_pressure_psi_low_pass(
                pres_ch0_port, &pres_ch0_low_pass, PRES_CH0_TIME_DIFF_ms
            );
            if ((ch0_pres_count & PRES_CH0_DOWNSAMPLE_MASK) == 0) {
                can_msg_t sensor_msg;

                build_analog_data_msg(
                    PRIO_LOW, millis(), SENSOR_PT_CHANNEL_0, ch0_pressure, &sensor_msg
                );
                txb_enqueue(&sensor_msg);
            }
            ch0_pres_count++;
        }
#endif

#if PRES_CH1_TIME_DIFF_ms
        if (millis() - last_pres_ch1_millis > PRES_CH1_TIME_DIFF_ms) {
            last_pres_ch1_millis = millis();
            uint16_t ch1_pressure = update_pressure_psi_low_pass(
                pres_ch1_port, &pres_ch1_low_pass, PRES_CH1_TIME_DIFF_ms
            );
            if ((ch1_pres_count & PRES_CH1_DOWNSAMPLE_MASK) == 0) {
                can_msg_t sensor_msg;

                build_analog_data_msg(
                    PRIO_LOW, millis(), SENSOR_PT_CHANNEL_1, ch1_pressure, &sensor_msg
                );
                txb_enqueue(&sensor_msg);
            }
            ch1_pres_count++;
        }
#endif

#if PRES_CH2_TIME_DIFF_ms
        if (millis() - last_pres_ch2_millis > PRES_CH2_TIME_DIFF_ms) {
            last_pres_ch2_millis = millis();
            uint16_t ch2_pressure = update_pressure_psi_low_pass(
                pres_ch2_port, &pres_ch2_low_pass, PRES_CH2_TIME_DIFF_ms
            );
            if ((ch2_pres_count & PRES_CH2_DOWNSAMPLE_MASK) == 0) {
                can_msg_t sensor_msg;

                build_analog_data_msg(
                    PRIO_LOW, millis(), SENSOR_PT_CHANNEL_2, ch2_pressure, &sensor_msg
                );
                txb_enqueue(&sensor_msg);
            }
            ch2_pres_count++;
        }
#endif

#if PRES_CH3_TIME_DIFF_ms
        if (millis() - last_pres_ch3_millis > PRES_CH3_TIME_DIFF_ms) {
            last_pres_ch3_millis = millis();
            uint16_t ch3_pressure = update_pressure_psi_low_pass(
                pres_ch3_port, &pres_ch3_low_pass, PRES_CH3_TIME_DIFF_ms
            );
            if ((ch3_pres_count & PRES_CH3_DOWNSAMPLE_MASK) == 0) {
                can_msg_t sensor_msg;

                build_analog_data_msg(
                    PRIO_LOW, millis(), SENSOR_PT_CHANNEL_3, ch3_pressure, &sensor_msg
                );
                txb_enqueue(&sensor_msg);
            }
            ch3_pres_count++;
        }
#endif

#if PRES_CH4_TIME_DIFF_ms
        if (millis() - last_pres_ch4_millis > PRES_CH4_TIME_DIFF_ms) {
            last_pres_ch4_millis = millis();
            uint16_t ch4_pressure = update_pressure_psi_low_pass(
                pres_ch4_port, &pres_ch4_low_pass, PRES_CH4_TIME_DIFF_ms
            );
            if ((ch4_pres_count & PRES_CH4_DOWNSAMPLE_MASK) == 0) {
                can_msg_t sensor_msg;

                build_analog_data_msg(
                    PRIO_LOW, millis(), SENSOR_PT_CHANNEL_4, ch4_pressure, &sensor_msg
                );
                txb_enqueue(&sensor_msg);
            }
            ch4_pres_count++;
        }
#endif

#if HALLSENSE_FUEL_TIME_DIFF_ms
        if (millis() - last_hallsense_fuel_millis > HALLSENSE_FUEL_TIME_DIFF_ms) {
            last_hallsense_fuel_millis = millis();
            uint16_t hallsense_fuel_flux = get_hall_sensor_reading(hallsense_fuel);
            can_msg_t sensor_msg;

            build_analog_data_msg(
                PRIO_LOW, millis(), SENSOR_FUEL_INJ_HALL, hallsense_fuel_flux, &sensor_msg
            );
            txb_enqueue(&sensor_msg);
        }
#endif

#if HALLSENSE_OX_TIME_DIFF_ms
        if (millis() - last_hallsense_ox_millis > HALLSENSE_OX_TIME_DIFF_ms) {
            last_hallsense_ox_millis = millis();
            uint16_t hallsense_ox_flux = get_hall_sensor_reading(hallsense_ox);
            can_msg_t sensor_msg;

            build_analog_data_msg(
                PRIO_LOW, millis(), SENSOR_OX_INJ_HALL, hallsense_ox_flux, &sensor_msg
            );
            txb_enqueue(&sensor_msg);
        }
#endif

        // send any queued CAN messages
        txb_heartbeat();
        // LED_W = LED_W ^ LED_OFF;
    }

    return (EXIT_SUCCESS);
}

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

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);

    // ignore messages that were sent from this board
    if (get_board_type_unique_id(msg) == BOARD_TYPE_UNIQUE_ID) {
        return;
    }

    switch (msg_type) {
        case MSG_LEDS_ON:
            LED_R = !LED_OFF;
            LED_W = !LED_OFF;
            LED_B = !LED_OFF;
            break;

        case MSG_LEDS_OFF:
            LED_R = LED_OFF;
            LED_W = LED_OFF;
            LED_B = LED_OFF;
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
