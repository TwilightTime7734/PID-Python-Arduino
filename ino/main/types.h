#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

#define MAX_COUNT 16

// Little endian union
typedef union {
    unsigned long raw;
    struct {
        word low;
        word high;
    };
} long_t;

typedef union __attribute__ ((packed)) {
    word raw[35];
    struct __attribute__ ((packed)) {  // 35 words total
        word quant;                    //[0] 1  - 1 microsec in timer ticks
        word max_count;                //[1] 1  - Maximum number of channels
        word state;                    //[2] 1  - / 0 - Off / 1 - On / 2 - On (inversion) 
        word count;                    //[3] 1  - Number of channels (0 ... MAX_COUNT)
        word pause;                    //[4] 1  - Pause duration in timer ticks
        long_t sync;                   //[5] 2  - Synchronization pulse duration in timer ticks
        word channel[MAX_COUNT];       //[7] 16 - Pulse duration in timer ticks
        word pulse_chl;                //[23] 1  - Hold override channel index
        word pulse_val;                //[24] 1  - Hold override value in timer ticks
        long_t pulse_dur;              //[25] 2  - Hold override duration in microseconds (0 => end hold)
        word pulse_seq;                //[27] 1  - Incrementing command sequence
        word pulse_status;             //[28] 1  - / 0 idle, 1 active, 2 rejected, 3 timeout-restored, 4 hold-ended
		word movement_status;          //[29] 1  - / 0 - No sensor / 1 - Sensor OK / 2 - Sample Ready / 3 - Sample Failed
		word movement_seq;             //[30] 1  - Increments each time a new sensor sample is stored
		long_t movement_millis;        //[31] 2  - Arduino millis() timestamp for the stored sample
		int16_t roll_angle;            //[33] 1  - Roll angle in degrees
		int16_t pitch_angle;           //[34] 1  - Pitch angle in degrees
    };
} regs_t;

// Inline Helper Functions
static inline void copy_frame_fields(regs_t &dst, const regs_t &src, bool include_movement = false) {

    dst.state = src.state;
    dst.count = src.count;
    dst.pause = src.pause;
    dst.sync = src.sync;
    for (byte i = 0; i < MAX_COUNT; i++) {
        dst.channel[i] = src.channel[i];
    }
}

static inline bool frame_fields_differ(const regs_t &a, const regs_t &b) {
    if (a.state != b.state || a.count != b.count || a.pause != b.pause || a.sync.raw != b.sync.raw) {
        return true;
    }
    for (byte i = 0; i < MAX_COUNT; i++) {
        if (a.channel[i] != b.channel[i]) {
            return true;
        }
    }
    return false;
}

static inline void copy_pulse_command_fields(regs_t &dst, const regs_t &src) {
    dst.pulse_chl = src.pulse_chl;
    dst.pulse_val = src.pulse_val;
    dst.pulse_dur = src.pulse_dur;
    dst.pulse_seq = src.pulse_seq;
}

#endif // TYPES_H