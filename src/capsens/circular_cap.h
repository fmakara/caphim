#pragma once

#include <stdint.h>
#include <vector>

// A circular capacitive sensor handler.
//   Uses one PIO FSM for time measurement
class CircCapSens {
public:
    CircCapSens(const std::vector<uint8_t>& pins, int maxCnt = 1000, int hister = 3);
    void init();

    // Must be called each "decision iteration", as it updates the internal states
    // Returns the estimated/averaged position of finger
    int read();

    int lastRead(); // last value (used in conjunction with wasClicked)
    int lastDelta(); // used to easily jump between menus

    bool isPressed(); // only true on start of touch
    bool isHeld(); // true while detecting touch
    bool isReleased(); // only true on end of touch

    bool wasClicked(); // true if no movement (in this touch) and isReleased

    // If the pattern is too spread, will invalidate current touch.
    // isHeld and isReleased will return false until next touch
    bool isInvalid();

    std::vector<bool> pressed(); // all the segments states
    std::vector<int> rawVal(); // all the segments values (used in debugging)

private:
    int mMaxCnt, mHister;
    struct PadData {
        uint8_t pin;
        int relPosX, relPosY;
        int lastRead;
        int threshold;
        bool state;
    };
    std::vector<PadData> mPads;

    // Pio logic
    uint32_t readPin(uint8_t pin);
    bool mStarted;
    uint8_t mPioOffset, mPioSm;

    // Scroll logic
    int mLastPos, mFirstPos, mLastDelta;
    bool mCurrState, mLastState, mHadDelta, mTouchInvalid;
};