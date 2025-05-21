#include <stdio.h>
#include <stdlib.h>

#include <stdbool.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "sensor_general.h"

#include "rocketlib/include/timer.h"

//#include "IOExpanderDriver.h"
//#include "actuator.h"
//#include "error_checks.h"
//#include "i2c.h"

#include <xc.h>


// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 // 2 Hz

int main() {
    while (1) {
        // preface with #if (BOARD_UNIQUE_ID == ??
        uint32_t last_millis = millis();
        
        // heartbeat Red LED to toggle every 500ms if status_ok
        if (millis() - last_millis > STATUS_TIME_DIFF_ms) {

            // check for general board status
            bool status_ok = true;
            // below are likely unnecessary as stated by liz
//            status_ok &= check_battery_voltage_error(batt_vol_sense);
//
//            status_ok &= check_5v_current_error(current_sense_5v);
//            status_ok &= check_12v_current_error(current_sense_12v);

            // if there was an issue, a message would already have been sent out
//            if (status_ok) {
//                send_status_ok();
//            }
            
            // Red LED flashes during safe state.
            LED_heartbeat_R();
        } 
    }
}
