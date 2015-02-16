import Adafruit_BBIO.GPIO as gpio
import Adafruit_BBIO.ADC as adc
import Adafruit_BBIO.PWM as pwm
import time
sleep = time.sleep
import kj
gpio.cleanup()
pwm.cleanup()

'''
plan to get this code on track for OA:
1. get IR pins working
3. get button presses working
2. get manual motor commands working
3. link IR to adjustment in motor comm
'''

def getIR():
    L=0.0
    M=0.0
    R=0.0
    k=3
    for i in range(0,2**k):
        L=L+kj.irReadcm (irLPin)
        M=M+kj.irReadcm(irMPin)
        R=R+kj.irReadcm(irRPin)
        # sleep(1e-6)
    L=int(L)
    L=L>>k
    M=int(M)
    M=M>>k
    R=int(R)
    R=R>>k
    return (L,M,R)
#def printIR

def decision(Ldist,Mdist,Rdist):

    Ldist = kj.tooClose(Ldist)
    Mdist = kj.tooClose(Mdist)
    Rdist = kj.tooClose(Rdist)
    if((not Ldist) and (not Mdist) and (not Rdist)):
        print "fwd"
    elif((not Ldist) and Rdist):
        print "left"
    elif((not Rdist) and ((not Ldist and Mdist) or (Ldist and not Mdist))):
        print "right"

    elif((Ldist) and ((not Rdist and Mdist) or (Rdist and not Mdist))):
        print "backup, turn"
    else:
        print "backup, 180"

def fwd(dutyCycle):
    print "fwd"
    gpio.output(L1pin,0)
    gpio.output(L2pin,1)
    gpio.output(R1pin,1) #this makes mR turn clockwise, fwd motion.
    gpio.output(R2pin,0)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)
#def fwd
def bwd(dutyCycle):
    print "bwd"
    gpio.output(L1pin,1)
    gpio.output(L2pin,0)
    gpio.output(R1pin,0)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)
#def bwd
def left(dutyCycle):
    # will initially do 0-radius turns
    print "left"
    gpio.output(L1pin,1)
    gpio.output(L2pin,0)
    gpio.output(R1pin,1)
    gpio.output(R2pin,0)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)
#def left
def right(dutyCycle):
    # will initially do 0-radius turns
    print "right"
    gpio.output(L1pin,0)
    gpio.output(L2pin,1)
    gpio.output(R1pin,0)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)
#def right
def stop():
    print "stop"
    gpio.output(L1pin,1)
    gpio.output(L2pin,1)
    gpio.output(R1pin,1)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,0,50)
    pwm.start(RpwmPin,0,50)
#def stop

def userInput():
    notdone=1
    choice=3
    while(notdone):
        if gpio.input(pinUP):
            notdone=0
            choice=0
        if gpio.input(pinDN):
            notdone=0
            choice=1
        if gpio.input(pinYES):
            notdone=0
            choice=2
    return choice
# MAIN START ##############################################
b1Pin="P9_12"
pin1="P9_23"
irLPin="P9_39"  #ADC0 #note: refer to ADC still as P#_##
irMPin="P9_40"  #ADC1
irRPin="P9_37"  #ADC2
enRApin="P8_3"  #encoder, right, chA
enRBpin="P8_4"  #encoder, right, chB
L1pin="P8_7"    #h-bridge, left
L2pin="P8_8"    #h-bridge, left
R1pin="P8_14"   #h-bridge, right
R2pin="P8_15"   #h-bridge, right
LpwmPin="P9_21" #h-bridge, left
RpwmPin="P9_16" #h-bridge, right
pinUP = "P8_17"
pinDN = "P8_18"
pinYES = "P8_19"


prgmDone=0


# setting up pin directions
    # note, code replacements for ease:
    # 0 = gpio.IN, gpio.LOW
    # 1 = gpio.OUT, gpio.HIGH
gpio.setup(pin1,0)
gpio.add_event_detect(pin1,gpio.RISING) # end program when pressed

gpio.setup(enRApin, 0) # setup encoder inputs
gpio.setup(enRBpin, gpio.IN)
gpio.add_event_detect(enRApin, gpio.RISING)
gpio.add_event_detect(enRBpin, gpio.RISING)

gpio.setup(R1pin, 1) # setup h-bridge direction
gpio.setup(R2pin, 1)
gpio.setup(L1pin, 1)
gpio.setup(L2pin, 1)
gpio.output(L1pin, 0) # set default value to low
gpio.output(L2pin, 0)
gpio.output(R1pin, 0)
gpio.output(R2pin, 0)

gpio.setup(pinUP,0)
# gpio.add_event_detect(pinUP,gpio.RISING) # end program when pressed
gpio.setup(pinDN,0)
# gpio.add_event_detect(pinDN,gpio.RISING) # end program when pressed
gpio.setup(pinYES,0)
# gpio.add_event_detect(pinYES,gpio.RISING) # end program when pressed

adc.setup()
kj.ledINIT()
print "Start..."
sleep(1)
print "Now"
kj.blink(0)

# menu options can either go on top or bottom
# sequence options must go on top
menu0=['1. add','2. sub','3. cal']
sequ1=['add-N1','add-N2','add-RES']
sequ2=['sub-N1','sub-N2','sub-RES']
menu3=['3a. cam','3b. IR']
sequ3a=['cam s1','cam s2']
sequ3b=['IR s1','IR s2']



# LATCH LIST ###############################
# if gpio.event_detected(b1Pin):
#     prgmDone=1
# if gpio.event_detected(pin1):
#     print "Flash LED"
#
# if gpio.event_detected(enRApin):
#     print "chA"
# if gpio.event_detected(enRBpin):
#     print "chB"

# LATCH LIST END ###########################


# MAIN LOOP ###############################################
i=0
prevTime=time.time()
while(not prgmDone):
    if gpio.event_detected(pin1):
        prgmDone=1
        print "Ending Program"

    # if gpio.input(pinUP):
    #     print "up"
    # if gpio.input(pinDN):
    #     print "down"
    # if gpio.input(pinYES):
    #     print "yes"

# sample change
    # (L,M,R) = getIR()
    # decision(L,M,R)

    # for the moment, will now make quick options menu:
    print "please select option: "
    print '''
    1. add
    2. subtract
    3. calibrate
    '''
    user=userInput()
    print user

    LoopTime=time.time()-prevTime
    prevTime=time.time()
    print LoopTime*1000
    sleep(1)
# MAIN END ################################################

gpio.cleanup()
pwm.cleanup()
