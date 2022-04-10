#include "circular_cap.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "capsens.pio.h"
#include "math.h"

#define PIO pio1
#define READ_SMOOTHING 128
#define TEMPORAL_FILTER 4
#define FIXED_POS_SCALE 1000
#define CUTOFF_POS_RATIO 0.1

#define CUTOFF_POS_SQUARED (CUTOFF_POS_RATIO*CUTOFF_POS_RATIO*FIXED_POS_SCALE*FIXED_POS_SCALE)

CircCapSens::CircCapSens(const std::vector<uint8_t>& pins, int maxCnt, int hister)
    : mMaxCnt(maxCnt)
    , mHister(hister*READ_SMOOTHING)
    , mPads(pins.size(), PadData({0, 0, 0, maxCnt*READ_SMOOTHING, maxCnt*READ_SMOOTHING, false}))
    , mStarted(false)
    , mPioOffset(0)
    , mPioSm(0)
    , mLastPos(0)
    , mFirstPos(0)
    , mLastDelta(0)
    , mCurrState(false)
    , mLastState(false)
    , mHadDelta(false)
    , mTouchInvalid(false)
{
    for(unsigned i=0; i<pins.size(); i++){
        mPads[i].pin = pins[i];
        mPads[i].relPosX = FIXED_POS_SCALE*cos((i*2*M_PI)/pins.size());
        mPads[i].relPosY = FIXED_POS_SCALE*sin((i*2*M_PI)/pins.size());
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
        for(int i=0; i<10; i++) read();
    }
}

uint32_t CircCapSens::readPin(uint8_t pin){
    capsens_program_init(PIO, mPioSm, mPioOffset, pin);
    uint32_t ret = capsens_program_read(PIO, mPioSm, mMaxCnt);
    capsens_program_deinit(PIO, mPioSm, pin);
    return ret;
}

int CircCapSens::read() {
    init();
    unsigned size = mPads.size();
    mLastState = mCurrState;
    mCurrState = false;
    int currPosX = 0, currPosY = 0, currCnt = 0;
    {
        std::vector<uint32_t> sums(mPads.size(), 0);
        for(int i=0; i<READ_SMOOTHING; i++){
            for(int j=0; j<mPads.size(); j++) sums[j] += readPin(mPads[j].pin);
        }
        for(int i=0; i<mPads.size(); i++) mPads[i].lastRead = ((mPads[i].lastRead*(TEMPORAL_FILTER-1))+sums[i])/TEMPORAL_FILTER;
    }
    for(unsigned i=0; i<size; i++){
        // Patent pending (TM) (#sqn) Debouncing logic
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
            currPosX += mPads[i].relPosX;
            currPosY += mPads[i].relPosY;
            currCnt++;
        }
    }
    if(!mLastState || !mCurrState){
        mLastPos = 0;
        mLastDelta = 0;
        mFirstPos = 0;
        mTouchInvalid = false;
    }

    if(!mCurrState || currCnt==0) return 0;
    currPosX /= currCnt;
    currPosY /= currCnt;

    int distSquared = currPosY*currPosY+currPosX*currPosX;
    if(distSquared<CUTOFF_POS_SQUARED){
        mTouchInvalid = true;
        return mLastPos;
    }

    int angle = mPads.size()*(1+atan2(currPosY, currPosX)/M_PI);
    if(!mLastState && mCurrState) {
        mLastPos = angle;
        mFirstPos = angle;
        mHadDelta = false;
    }
    int psize = mPads.size();
    int modCurrAngle = mLastPos%(psize*2);
    if(modCurrAngle<0) modCurrAngle+=psize*2;
    int deltaAngle = angle-modCurrAngle;
    if(deltaAngle<-psize) deltaAngle += 2*psize;
    if(deltaAngle>psize) deltaAngle -= 2*psize;

    if(deltaAngle>0) deltaAngle -= 2;
    else deltaAngle += 2;
    mLastDelta = deltaAngle/3;
    mLastPos += mLastDelta*3;
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
        ret[i] = mPads[i].lastRead/READ_SMOOTHING;
    }
    return ret;
}

int CircCapSens::lastDelta(){
    return mLastDelta;
}

int CircCapSens::lastRead() { 
    return (mLastPos-mFirstPos)/2;
}

bool CircCapSens::isHeld() {
    return mCurrState && !mTouchInvalid;
}

bool CircCapSens::isPressed() {
    return (mCurrState && !mLastState) && !mTouchInvalid;
}

bool CircCapSens::isReleased() {
    return (!mCurrState && mLastState) && !mTouchInvalid;
}

bool CircCapSens::wasClicked() {
    return isReleased() && !mHadDelta;
}

bool CircCapSens::isInvalid() {
    return mTouchInvalid;
}