#include <xc.h>

#include "canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"

#include "timer.h"

#define OVER_VOLTAGE_12V_BITFIELD 0x04
#define UNDER_VOLTAGE_12V_BITFIELD 0x05

/* TODO
 * add PT range error check
 */

const float mA_SENSE_CONVERT_FACTOR =
    10000 * 3.3 / 4096.0f; //  uV conversion / 100 V/V multiplier * vref / 12bit adc
//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

bool check_5v_current_error(adcc_channel_t current_channel) { // Check bus current error

    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 62; // 62 is R8 rating in mR

    if (curr_draw_mA > BUS_OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        // [TODO] verify message priority
        // replace all CAN messages with build_general_board_status_msg(... ... ... )
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, (1 << E_5V_OVER_CURRENT_OFFSET), 0, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}

bool check_12v_current_error(adcc_channel_t current_channel) { // check battery current error
    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 15; // 15 is R7 rating in mR

    if (curr_draw_mA > BAT_OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, (1 << E_12V_OVER_CURRENT_OFFSET), 0, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}


