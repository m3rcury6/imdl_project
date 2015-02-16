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
    Ldist=0.0
    Mdist=0.0
    Rdist=0.0
    k=3
    for i in range(0,2**k):
        # sleep(1e-4)
        adcL=kj.irReadcm(irLPin)
        # sleep(1e-4)
        adcM=kj.irReadcm(irMPin)
        # sleep(1e-4)
        adcR=kj.irReadcm(irRPin)
        # sleep(1e-4)
        Ldist=Ldist+adcL
        Mdist=Mdist+adcM
        Rdist=Rdist+adcR
        # sleep(1e-3) # this whole fn takes ~40ms to do anyway
    Ldist=int(Ldist)
    Ldist=Ldist>>k
    Mdist=int(Mdist)
    Mdist=Mdist>>k
    Rdist=int(Rdist)
    Rdist=Rdist>>k
    return (Ldist,Mdist,Rdist)

def decision(Ldist,Mdist,Rdist):

    Ldist = kj.tooClose(Ldist)
    Mdist = kj.tooClose(Mdist)
    Rdist = kj.tooClose(Rdist)
    if((not Ldist) and (not Mdist) and (not Rdist)):
        print "fwd"
        fwd(speed)
    elif((not Ldist) and Rdist):
        print "left"
        left(speed)
    elif((not Rdist) and ((not Ldist and Mdist) or (Ldist and not Mdist))):
        print "right"
        right(speed)

    elif((Ldist) and ((not Rdist and Mdist) or (Rdist and not Mdist))):
        print "backup, turn"
        bwd(speed)
        sleep(1.5)
        right(speed)
        sleep(1.5)
    else:
        print "backup, 180"
        bwd(speed)
        sleep(1.5)
        right(speed)
        sleep(1.5)

def fwd(dutyCycle):
    # print "fwd"
    gpio.output(L1pin,0)
    gpio.output(L2pin,1)
    gpio.output(R1pin,1) #this makes mR turn clockwise, fwd motion.
    gpio.output(R2pin,0)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)

def bwd(dutyCycle):
    # print "bwd"
    gpio.output(L1pin,1)
    gpio.output(L2pin,0)
    gpio.output(R1pin,0)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)

def left(dutyCycle):
    # will initially do 0-radius turns
    # print "left"
    gpio.output(L1pin,1)
    gpio.output(L2pin,0)
    gpio.output(R1pin,1)
    gpio.output(R2pin,0)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)

def right(dutyCycle):
    # will initially do 0-radius turns
    # print "right"
    gpio.output(L1pin,0)
    gpio.output(L2pin,1)
    gpio.output(R1pin,0)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,dutyCycle,50)
    pwm.start(RpwmPin,dutyCycle,50)

def stop():
    # print "stop"
    gpio.output(L1pin,1)
    gpio.output(L2pin,1)
    gpio.output(R1pin,1)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,0,50)
    pwm.start(RpwmPin,0,50)


# MAIN START ##############################################
mainPin="P9_23"

LpwmPin="P9_21" #h-bridge, left
L1pin="P8_7"    #h-bridge, left
L2pin="P8_8"    #h-bridge, left
RpwmPin="P9_16" #h-bridge, right
R1pin="P8_14"   #h-bridge, right
R2pin="P8_15"   #h-bridge, right

pinUP = "P8_17"
pinDN = "P8_18"
pinYES = "P8_19"

irLPin=0 #"P9_39"  #ADC0 #note: refer to ADC still as P#_##
irMPin=1 #"P9_40"  #ADC1
irRPin=2 #"P9_37"  #ADC2

enRApin="P8_3"  #encoder, right, chA
enRBpin="P8_4"  #encoder, right, chB

speed=70 # duty cycle
# setting up pin directions
    # note, code replacements for ease:
    # 0 = gpio.IN, gpio.LOW
    # 1 = gpio.OUT, gpio.HIGH
gpio.setup(mainPin,0)
gpio.add_event_detect(mainPin,gpio.RISING) # end program when pressed

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
gpio.add_event_detect(pinUP,gpio.RISING) # end program when pressed
gpio.setup(pinDN,0)
gpio.add_event_detect(pinDN,gpio.RISING) # end program when pressed
gpio.setup(pinYES,0)
gpio.add_event_detect(pinYES,gpio.RISING) # end program when pressed

adc.setup()
kj.ledINIT()
print "Start..."
sleep(1)
print "Now"
kj.blink(0)
deathcounter=0 # death at 169 counts, 2 times
# menu options can either go on top or bottom
# sequence options must go on top
prevTime=time.time()
prgmDone=0
# MAIN LOOP ###############################################
while(not prgmDone):
    if gpio.event_detected(mainPin):
        prgmDone=1
        print "Ending Program"

    (L,M,R) = getIR()
    print L,M,R
    # decision(L,M,R)


    # L=kj.analogRead(0)
    # M=kj.analogRead(1)
    # R=kj.analogRead(2)
    # print L,M,R

    # LoopTime=time.time()-prevTime
    # prevTime=time.time()
    # deathcounter+=1
    # print deathcounter, LoopTime*1000
    sleep(1)
    if gpio.event_detected(mainPin):
        prgmDone=1
        print "Ending Program"
# MAIN END ################################################

gpio.cleanup()
pwm.cleanup()
