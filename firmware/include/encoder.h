#ifndef ANR_ENCODER_H
#define ANR_ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    explicit Encoder(int pin);
    void begin();

    // Returns the incremental tick count since the previous call and clears it.
    int32_t consumeTicks();
    int32_t totalTicks() const;

private:
    int _pin;
    volatile int32_t _delta;
    volatile int32_t _total;

    static void IRAM_ATTR isrThunk(void* arg);
    void IRAM_ATTR onPulse();
};

#endif // ANR_ENCODER_H
