#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

// [TODO] verify naming convention
// at this voltage, a warning will be send out over CAN
#define INPUT_OVERVOLTAGE_THRESHOLD_mV 12700

// [TODO] verify naming convention
// at this voltage, a warning will be send out over CAN
#define INPUT_UNDERVOLTAGE_THRESHOLD_mV 11500

#include "mcc_generated_files/system/system.h"

#include "rocketlib/include/timer.h"

// [TODO] verify naming convention
// General board status checkers
bool check_battery_voltage_error(adcc_channel_t battery_channel, uint32_t *general_error_bitfield);

#endif /* ERROR_CHECKS_H */

