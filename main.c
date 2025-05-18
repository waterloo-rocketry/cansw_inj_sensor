#include <stdio.h>
#include <stdlib.h>

#include <stdbool.h>

//#include "canlib/canlib.h"
//#include "canlib/message_types.h"

#include "sensor_general.h"

#include <xc.h>

int main() {
    while (true) {
        for (int i = 0; i < 1000; ++i) {
            continue;
        }
        LED_heartbeat_R();
        for (int i = 0; i < 1000; ++i) {
            continue;
        }
    }
   
}
