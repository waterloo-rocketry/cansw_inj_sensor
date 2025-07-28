#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "timer.h"

#include <stdbool.h>

// Board specific error bits
#define E_OX_PT_INVALID_OFFSET 0x00
#define E_FUEL_PT_INVALID_OFFSET 0x01
#define E_CC_PT_INVALID_OFFSET 0x02
#define E_OX_HALL_INVALID_OFFSET 0x03
#define E_FUEL_HALL_INVALID_OFFSET 0x04

// General board status checkers
bool check_5v_current_error(adcc_channel_t current_channel);
bool check_12v_current_error(adcc_channel_t current_channel);
bool check_PT_current_error(adcc_channel_t pt_channel);

#endif /* ERROR_CHECKS_H */

