#include <msp430.h>
#include "Library.h"

void main(void) {
    STOP_WATCHDOG_TIMER();
    DCO_INITIALIZE(DCO_1MHz);
    RUN{

    }
}
