from machine import Pin
from rp2 import PIO, StateMachine, asm_pio
from utime import sleep_ms, sleep_us
from math import cos, sin, atan2, pi
import _thread

@asm_pio(set_init=PIO.IN_LOW, out_shiftdir=PIO.SHIFT_LEFT, in_shiftdir=PIO.SHIFT_LEFT, autopull=True, autopush=True, pull_thresh=32, push_thresh=32)
def capsens_prog():
    wrap_target()          # .wrap_target
    set(pindirs, 1)        #     set pindirs 1
    set(pins, 0)           #     set pins 0
    out(x, 32)             #     out x 32
    set(pindirs, 0)        #     set pindirs 0
    label("waitloop")      # waitloop:
    jmp(pin, "done")       #     jmp pin done
    jmp(x_dec, "waitloop") #     jmp x-- waitloop
    label("done")          # done:
    in_(x, 32)             #     in x 32
    wrap()                 # .wrap

def capsens_thread(self):
    while True:
        data = self.fakeRead()
        self._mutex.acquire()
        self._data = data
        self._mutex.release()

class PIOCapSens:
    """A capacitive sensor using PIO"""
    def __init__(self, sm_id, maxCount=5000):
        """
        Parameters:
            sm_id: The StateMachine ID that will be used
            maxCount: The maximum value of the counter
        """
        assert (sm_id>=0 and sm_id<=1), 'Invalid SM id'
        assert maxCount>1, 'MaxCount must be positive'
        self._sm = StateMachine(sm_id)
        self._maxCount = maxCount

    def read(self, pin):
        """Runs the algorithm on Pin 'pin' and then cleans itself"""
        pin = pin if isinstance(pin, Pin) else Pin(pin)
        pin.init(Pin.IN, pull=None)

        self._sm.init(capsens_prog, in_base=pin, out_base=pin, set_base=pin, jmp_pin=pin)
        self._sm.active(1)
        self._sm.put(self._maxCount)
        ret = self._maxCount-self._sm.get()
        self._sm.active(0)
        pin.init(Pin.OUT, value=0)
        return ret
    
    def read_mult(self, pin, mult):
        """Runs the algorithm for 'mult' times on Pin 'pin' and then cleans itself"""
        pin = pin if isinstance(pin, Pin) else Pin(pin)
        pin.init(Pin.IN, pull=None)
        self._sm.init(capsens_prog, set_base=pin, jmp_pin=pin)
        self._sm.active(1)
        sum = 0
        for i in range(mult):
            self._sm.put(self._maxCount)
            sum = sum+(self._maxCount-self._sm.get())
            sleep_us(1)
        self._sm.active(0)
        pin.init(Pin.OUT, value=0)
        return sum

class CircCapSens:
    """A circular capacitive controller, able to implement buttons and/or circular dial"""
    SCALE_FIXED_POS = 1000
    CUTOFF_POS_RATIO = 0.1
    CUTOFF_POS_SQUARED = int(CUTOFF_POS_RATIO*CUTOFF_POS_RATIO*SCALE_FIXED_POS*SCALE_FIXED_POS)
    
    def __init__(self, pins, maxCount=5000, histeresis=5, readSmoothing=128, temporalFilter=4, sm_id=1, precision=2, threading=False):
        """
        Parameters:
            pins: Erray with the capacitive pins in order. Elements can be int or Pin()
            maxCount: maximum wait period for each sample
            histeresis: value delta that determines an action. Make lower if no response, make higher if too sensitive
            readSmoothing: number of times each pin is read each interation ('instant filter')
            temporalFilter: the factor for simple IIR filter. Set to 1 to disable
            sm_id: the ID for StateMachine
            precision: Precision passed to the user. Output range for dial is len(pins)*precision for each turn
            threading: read() may take some ms, so in order to have fluid operation this allocates second core to do it for you
        """
        truepins = [i if isinstance(i, Pin) else Pin(i) for i in pins]
        self.histeresis = histeresis
        self.readSmoothing = readSmoothing
        self.temporalFilter = temporalFilter
        self._pins = []
        self._sensor = PIOCapSens(sm_id, maxCount)
        self._lastPos = 0
        self._firstPos = 0
        self._lastDelta = 0
        self._currState = False
        self._lastState = False
        self._hadDelta = False
        self._touchInvalid = False
        self._precision = precision
        self._mutex = None
        for i in range(len(truepins)):
            angle = i*2*pi/len(truepins)
            maxcnt = maxCount*readSmoothing
            truepins[i].init(Pin.OUT, value=0)
            self._pins.append({
                'pin': truepins[i],
                'pos': (self.SCALE_FIXED_POS*cos(angle),self.SCALE_FIXED_POS*sin(angle)),
                'last': maxcnt,
                'th': maxcnt,
                'state': False
                })
        if threading:
            self._mutex = _thread.allocate_lock()
            _thread.start_new_thread(capsens_thread, (self))

    def fakeRead(self):
        """
        Reads the current status, but without updating the internal states
        Used for instant update with multithreading (i.e. fakeRead on background,
        send the return to foreground, update state instantly)
        As such, this function usually should not be used. See 'read()'
        """
        pinCount = len(self._pins)
        if not(self._mutex is None): self._mutex.acquire()
        hist = self.histeresis*self.readSmoothing
        smooth = self.readSmoothing
        temp = self.temporalFilter
        lastState = self._currState
        lastPos = self._lastPos
        firstPos = self._firstPos
        lastDelta = self._lastDelta
        hadDelta = self._hadDelta
        touchInvalid = self._touchInvalid
        lastRead = [ it['last'] for it in self._pins ]
        states = [ it['state'] for it in self._pins ]
        threshold = [ it['th'] for it in self._pins ]
        if not(self._mutex is None): self._mutex.release()

        #readSum = pinCount*[0]
        readSum = [ self._sensor.read_mult(it['pin'], smooth) for it in self._pins ]

        #for i in range(smooth):
        #    for j in range(pinCount):
        #        readSum[j] = readSum[j] + self._sensor.read(self._pins[j]['pin'])
        for i in range(pinCount):
            lastRead[i] = (lastRead[i]*(temp-1)+readSum[i])/temp

        currPosX = 0
        currPosY = 0
        currCnt = 0
        for i in range(pinCount):
            # Patent penting (TM) (#sqn) Debouncing Logic
            if states[i]:
                if lastRead[i]<threshold[i]:
                    states[i] = False
                    threshold[i] = lastRead[i]+hist
                elif threshold[i]+hist<lastRead[i]:
                    threshold[i] = lastRead[i]-hist
            else:
                if lastRead[i]>threshold[i]:
                    states[i] = True
                    threshold[i] = lastRead[i]-hist
                elif threshold[i]>lastRead[i]+hist:
                    threshold[i] = lastRead[i]+hist

            if states[i]:
                currPosX += self._pins[i]['pos'][0]
                currPosY += self._pins[i]['pos'][1]
                currCnt = currCnt+1
        
        currState = currCnt>0

        if not(lastState and currState):
            lastPos = 0
            firstPos = 0
            lastDelta = 0
            touchInvalid = False
        
        if currState:
            currPosX = currPosX/currCnt
            currPosY = currPosY/currCnt
            distSquared = currPosX*currPosX + currPosY*currPosY

            if distSquared<self.CUTOFF_POS_SQUARED:
                # average position too close to center
                touchInvalid = True
            else:
                maxAngle = pinCount*self._precision
                angle = int((maxAngle/2)*(1+atan2(currPosX, currPosY)/pi))
                
                # Reset angles on start of pressing
                if not lastState:
                    lastPos = angle
                    firstPos = angle
                    hadDelta = False
                
                modCurrAngle = angle%maxAngle
                deltaAngle = angle-modCurrAngle
                if deltaAngle<-maxAngle/2: deltaAngle = deltaAngle+maxAngle
                if deltaAngle>maxAngle/2: deltaAngle = deltaAngle-maxAngle
                
                # dead-zone algorithm (avoids jumping between 2 positions)
                deltaAngle = deltaAngle-(self._precision/2) if deltaAngle>0 else deltaAngle-self._precision/2
                lastDelta = deltaAngle//self._precision
                lastPos = lastPos+lastDelta*self._precision
                hadDelta = lastDelta!=0
        
        return {
            'lastState': lastState,
            'currState': currState,
            'lastPos': lastPos,
            'firstPos': firstPos,
            'lastDelta': lastDelta,
            'hadDelta': hadDelta,
            'touchInvalid': touchInvalid,
            'lastRead': lastRead,
            'threshold': threshold,
            'states': states
            }
    
    def updateState(self, state):
        """
        Updates the current state. Used in conjunction with 'fakeRead()'
        This function usually should not be used. See 'read()'
        """
        if not(self._mutex is None): self._mutex.acquire()
        self._currState = state['currState']
        self._lastState = state['lastState']
        self._lastPos = state['lastPos']
        self._firstPos = state['firstPos']
        self._lastDelta = state['lastDelta']
        self._hadDelta = state['hadDelta']
        self._touchInvalid = state['touchInvalid']
        for i in range(len(self._pins)):
            self._pins[i]['last'] = state['lastRead'][i]
            self._pins[i]['th'] = state['threshold'][i]
            self._pins[i]['state'] = state['states'][i]
        if not(self._mutex is None): self._mutex.release()

    def read(self):
        """
        Runs the read algorithm and/or updates the internal variables
        Only call this function once each 'decision loop', since it will
        update the internal values and a button press may be lost
        """
        if not(self._mutex is None):
            while True:
                self._mutex.acquire()
                data = self._data
                self._data = None
                self._mutex.release()
                if not (data is None):
                    self.updateState(data)
                    return
                sleep_ms(1)
        # else:
        self.updateState(self.fakeRead())

    def lastDelta(self):
        """
        The value delta of the dial. Use in your application like:
            menuPos = (menuPos + sens.lastDelta()) % numItems
        or:
            menuPos = min(numItems-1, max(0, menuPos + sens.lastDelta() ))
        """
        return self._lastDelta

    def lastRead(self):
        """The current dial 'value'. Resets on release"""
        return self._lastPos-self._firstPos

    def isHeld(self):
        """True while something is beeing pressed and not invalid"""
        return self._currState and not self._touchInvalid

    def isPressed(self):
        """True only on start of press"""
        return (self._currState and not self._lastState)

    def isReleased(self):
        """True only on end of press nd not invalidated in the middle of the motion"""
        return (self._lastState and not self._currState) and not self._touchInvalid

    def wasClicked(self):
        """True only when there is a press+release without movement"""
        return self.isReleased() and not self._hadDelta

    def isInvalid(self):
        """Set True if during the movement the average position of all touched positions is too close to center"""
        return self._touchInvalid
    
    def states(self):
        """array[bool] with the individual pads pressed"""
        return [ it['state'] for it in self._pins ]
    
    def values(self):
        """array[int] with the individual capacitive/time values (for debug mostly)"""
        return  [ it['last']//self.readSmoothing for it in self._pins ]

def displayRing(states, fbuf, startOff, size=(128,64)):
    """
    As a form to inform the current pads being considered pressed, it is sugested that
    (after rendering the screen) this funcion or something similar be called.
    This function renders a 1 pixel ring around the screen that (try to) display
    which pads are being considered as pressed.
    """
    maxLen = 2*size[0]+2*size[1]-4
    acc = 1+maxLen//len(states);
    for i in range(len(states)):
        offset = (startOff+(i*maxLen)//len(states))%maxLen
        end = offset+acc
        c = 1 if states[i] else 0
        # top
        if offset<(size[0]-1):
            if end<(size[0]-1):
                fbuf.hline(offset, 0, end-offset, c)
                continue
            fbuf.hline(offset, 0, size[0]-(1+offset), c)
            offset = size[0]-1
        offset = offset-(size[0]-1)
        end = end-(size[0]-1)
        # right
        if offset<(size[1]-1):
            if end<(size[1]-1):
                fbuf.vline(size[0]-1, offset, end-offset, c)
                continue
            fbuf.vline(size[0]-1, offset, size[1]-(1+offset), c)
            offset = size[1]-1
        offset = offset-(size[1]-1)
        end = end-(size[1]-1)
        # bottom
        if offset<(size[0]-1):
            if end<(size[0]-1):
                fbuf.hline(size[0]-(end+1), size[1]-1, end-offset, c)
                continue
            fbuf.hline(0, size[1]-1, size[0]-(1+offset), c)
            offset = size[0]-1
        offset = offset-(size[0]-1)
        end = end-(size[0]-1)
        # left
        if offset<(size[1]-1):
            if end<(size[1]-1):
                fbuf.vline(0, size[1]-end, end-offset, c)
                continue
            fbuf.vline(0, 0, size[1]-(1+offset), c)
            offset = size[1]-1
        offset = offset-(size[1]-1)
        end = end-(size[1]-1)
        # top again
        if offset<(size[0]-1):
            if end<(size[0]-1):
                fbuf.hline(offset, 0, end-offset, c)
                continue
            fbuf.hline(offset, 0, size[0]-(1+offset), c)
        # right again only if len(states)<=3 or width too small

