#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "timer.h"

#include <stdbool.h>

// From 5V bus line. At this current, a warning will be sent out over CAN
#define BUS_OVERCURRENT_THRESHOLD_mA 100
#define BAT_OVERCURRENT_THRESHOLD_mA 150

// General board status checkers
bool check_5v_current_error(adcc_channel_t current_channel);
bool check_12v_current_error(adcc_channel_t current_channel);

#endif /* ERROR_CHECKS_H */

