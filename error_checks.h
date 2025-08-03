#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "timer.h"

#include <stdbool.h>

// General board status checkers
bool check_5v_current_error(adcc_channel_t current_channel);
bool check_12v_current_error(adcc_channel_t current_channel);
bool check_PT_current_error(adcc_channel_t pt_channel);

#endif /* ERROR_CHECKS_H */

