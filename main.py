import Adafruit_BBIO.GPIO as gpio
import Adafruit_BBIO.ADC as adc
import Adafruit_BBIO.PWM as pwm
import time
import kj
# import Adafruit_BBIO.UART as serial

'''
plan to get this code on track for OA:
1. get IR pins working
3. get button presses working
2. get manual motor commands working
3. link IR to adjustment in motor comm
'''

print "change made at 19:14"


def MLComm(velocity,kjgdir):
    #take in direction, move motor at that speed
    if(kjgdir==1):
        print "Lfwd", velocity
    elif(kjgdir==2):
        print "Lbwd"
    else:
        print "Lstop"
#def MLComm

def MRComm(velocity,kjgdir):
    #take in direction, move motor at that speed
    if(kjgdir==1):
        print "Rfwd",velocity
    elif(kjgdir==2):
        print "Rbwd"
    else:
        print "Rstop"
#def MRComm

def checkforPause():
    print "testing"

def tooClose(cmDistance):
    if(cmDistance<minDist): return 1
    else: return 0
#def tooClose

def USELESS_calibrate():
    print "old ir cal values: "
    print kj.irReadVarA,kj.irReadVarB


    a=range(10,90,10)
    b=range(0,len(a))

    for j in range(0,len(a)):
        kjgstr="press enter to save ADC @",a[j],"cm"
        raw_input(kjgstr)
        for i in range(0,10):
            sleep(.2)
            b[j]=b[j]+adc.read(irMPin)
        b[j]=b[j]/10.0
        sleep(.1)


    print "saved values (a,b):"
    print a
    print b
    sleep(2)
    print "calibrating ir..."
    # print kj.irCalibrate(a,b)
    (kj.irReadVarA,kj.irReadVarB)=kj.irCalibrate(a,b)
    print "new IR values: "
    print kj.irReadVarA,kj.irReadVarB
    # L=adc.read(irLPin)

    # R=adc.read(irRPin)
    # ir1=kj.irReadcm(irLPin)
    # ir2=kj.irReadcm(irMPin)
    # ir3=kj.irReadcm(ir3Pin)
    # print L, M, R
    # print M
#def USELESS_calibrate

def printIR():
    L=0.0
    M=0.0
    R=0.0
    k=3
    for i in range(0,2**k):
        L=L+kj.irReadcm(irLPin)
        M=M+kj.irReadcm(irMPin)
        R=R+kj.irReadcm(irRPin)
        sleep(1e-6)
    L=int(L)
    L=L>>k
    M=int(M)
    M=M>>k
    R=int(R)
    R=R>>k
    print L,M,R
#def printIR

def fwd():
    print "fwd"
    gpio.output(L1pin,0)
    gpio.output(L2pin,1)
    gpio.output(R1pin,1)
    gpio.output(R2pin,0)
    pwm.start(LpwmPin,60,50)
    pwm.start(RpwmPin,60,50)

def bwd():
    print "bwd"
    gpio.output(L1pin,1)
    gpio.output(L2pin,0)
    gpio.output(R1pin,0)
    gpio.output(R2pin,1)
    pwm.start(LpwmPin,60,50)
    pwm.start(RpwmPin,60,50)
def stop():
    print "stop"
    pwm.stop(LpwmPin)
    pwm.stop(RpwmPin)




# MAIN START ##############################################
gpio.cleanup()
pwm.cleanup()
sleep = time.sleep
# remember: P9_1,2 = GND // P9_3,4 = 3v3 // P9_5,6 = 5v
# remember: P9_34 = ADCGND
b1Pin="P9_12"
b2Pin="P9_15"
irLPin="P9_39" #ADC0 #note: refer to ADC still as P#_##
irMPin="P9_40" #ADC1
irRPin="P9_37" #ADC2
enRApin="P8_3" #encoder, right, chA
enRBpin="P8_4" #encoder, right, chB

L1pin="P8_5"
L2pin="P8_6"
R1pin="P8_14"
R2pin="P8_15"
LpwmPin="P9_14"
RpwmPin="P9_16"

endProgram=0


minDist=17 #cm


gpio.setup(b1Pin,gpio.IN)
gpio.setup(b2Pin,gpio.IN)
gpio.add_event_detect(b1Pin,gpio.RISING) # pause when pressed
gpio.add_event_detect(b2Pin,gpio.HIGH) # flash LED when pressed

gpio.setup(enRApin,gpio.IN)
gpio.setup(enRBpin,gpio.IN)
gpio.add_event_detect(enRApin,gpio.RISING)
gpio.add_event_detect(enRBpin,gpio.RISING)


adc.setup()

sleep(1)
print "starting"


# INT LIST START ###########################
if gpio.event_detected(b1Pin):
    endProgram=1
if gpio.event_detected(b2Pin):
    print "Flash LED"

if gpio.event_detected(enRApin):
    print "chA"
if gpio.event_detected(enRBpin):
    print "chB"

# INT LIST END #############################


# MAIN START ##############################################
while(not endProgram):
    if gpio.event_detected(enRApin):
        print "chA"
    if gpio.event_detected(enRBpin):
        print "chB"

# MAIN LOOP ###############################################


    #read in an ADC value.
    #if button is pressed, ask for user input

#    if(ir1>100): ir1 = 100 # help stop garbage data
    print "lala"
    sleep(1)
# MAIN END ################################################

# next portion:
L=tooClose(L)
M=tooClose(M)
R=tooClose(R)

#now do stuff based on what you know:
if((not L) and (not M) and (not R)):
    print "fwd"
elif((not L) and (not R)):
    print "left"
elif((not L and M) or (L and not M) or (L and M)):
    print "right"
elif(L and (not M) and R):
    print "backup, turn"
else:
    print "backup, 180"


while(1):
    print "lala"
    sleep(1)


#next, next portion:
la=gpio.input(somePin) #left encoder, chA
lb=gpio.input(somePin) #left encoder, chA




gpio.cleanup()
pwm.cleanup()
