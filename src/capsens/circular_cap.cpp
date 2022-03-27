#include "circular_cap.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "capsens.pio.h"

#define PIO pio0
#define CALC_PREC 4
#define READ_SMOOTHING 32
#define TEMPORAL_FILTER 4

CircCapSens::CircCapSens(const std::vector<uint8_t>& pins, int maxCnt, int hister) 
    : mMaxCnt(maxCnt)
    , mHister(hister)
    , mPads(pins.size(), PadData({0, maxCnt, maxCnt, false}))
    , mStarted(false)
    , mPioOffset(0)
    , mPioSm(0)
    , mLastPos(0)
    , mLastIntPos(0)
    , mLastDelta(0)
    , mCurrState(false)
    , mLastState(false)
    , mHadDelta(false)
{
    for(unsigned i=0; i<pins.size(); i++){
        mPads[i].pin = pins[i];        
    }
}

void CircCapSens::init(){
    if(!mStarted){
        for(uint8_t i=0; i<mPads.size(); i++){
            gpio_init(mPads[i].pin);
            gpio_set_dir(mPads[i].pin, GPIO_OUT);
            gpio_put(mPads[i].pin, false);
            gpio_set_pulls(mPads[i].pin, false, false);
        }
        mPioOffset = pio_add_program(PIO, &capsens_program);
        mPioSm = pio_claim_unused_sm(PIO, true);
        mStarted = true;
    }
}

uint32_t CircCapSens::readPin(uint8_t pin){
    capsens_program_init(PIO, mPioSm, mPioOffset, pin);
    uint32_t sum = 0;
    for(int i=0; i<READ_SMOOTHING; i++){
        sleep_us(1);
        sum += capsens_program_read(PIO, mPioSm, mMaxCnt);
    }
    capsens_program_deinit(PIO, mPioSm, pin);
    return sum/READ_SMOOTHING;
}

int CircCapSens::read() {
    init();
    unsigned size = mPads.size();
    mLastState = mCurrState;
    mCurrState = false;
    int currPos = 0, currCnt = 0;
    for(unsigned i=0; i<size; i++){
        mPads[i].lastRead = (mPads[i].lastRead*(TEMPORAL_FILTER-1) + readPin(mPads[i].pin))/TEMPORAL_FILTER;
        // Debouncing logic
        if (mPads[i].state) {
            if(mPads[i].lastRead<mPads[i].threshold){
                mPads[i].state = false;
                mPads[i].threshold = mPads[i].lastRead+mHister;
            } else if(mPads[i].threshold+mHister<mPads[i].lastRead) {
                mPads[i].threshold = mPads[i].lastRead-mHister;
            }
        } else {
            if(mPads[i].lastRead>mPads[i].threshold){
                mPads[i].state = true;
                mPads[i].threshold = mPads[i].lastRead-mHister;
            } else if(mPads[i].threshold-mHister>mPads[i].lastRead) {
                mPads[i].threshold = mPads[i].lastRead+mHister;
            }
        }
        if(mPads[i].state){
            mCurrState = true;
            currPos += CALC_PREC*i;
            currCnt++;
        }
    }
    if(!mLastState || !mCurrState){
        mLastPos = 0;
        mLastDelta = 0;
    }

    if(!mCurrState || currCnt==0) return 0;
    currPos /= currCnt;
    if(!mLastState){
        mLastIntPos = currPos;
        mHadDelta = false;
    }
    mLastPos = currPos;
    int dv = currPos-mLastIntPos;
    if(dv>0) dv -= CALC_PREC/2;
    else dv += CALC_PREC/2;
    mLastDelta = dv/CALC_PREC;
    mLastIntPos += mLastDelta*CALC_PREC;
    if(mLastDelta!=0) mHadDelta = true;
    return mLastPos;
}

std::vector<bool> CircCapSens::pressed(){
    std::vector<bool> ret(mPads.size());
    for(unsigned i=0; i<mPads.size(); i++){
        ret[i] = mPads[i].state;
    }
    return ret;
}
std::vector<int> CircCapSens::rawVal(){
    std::vector<int> ret(mPads.size());
    for(unsigned i=0; i<mPads.size(); i++){
        ret[i] = mPads[i].lastRead;
    }
    return ret;
}

int CircCapSens::lastDelta(){
    return mLastDelta;
}

int CircCapSens::lastRead() { 
    return mLastPos;
}

bool CircCapSens::isHeld() {
    return mCurrState;
}

bool CircCapSens::isPressed() {
    return mCurrState && !mLastState;
}

bool CircCapSens::isReleased() {
    return !mCurrState && mLastState;
}

bool CircCapSens::wasClicked() {
    return isReleased() && !mHadDelta;
}