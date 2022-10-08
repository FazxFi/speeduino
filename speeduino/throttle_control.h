#ifndef THROTTLE_H
#define THROTTLE_H

#include "globals.h"
#include "table2d.h"
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#define THROTTLE_CONTROL_OFF            0
#define THROTTLE_CONTROL_HB             1
#define THROTTLE_CONTROL_HB_ITPS_OL     2 //Test for ITPS based idle
#define THROTTLE_CONTROL_HB_ITPS_OL2    3 //For Using PID
#define THROTTLE_CONTROL_HB_ITPS_FFT    4 //For FeedForwardTerm test

#endif