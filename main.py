import Adafruit_BBIO.GPIO as gpio
import Adafruit_BBIO.ADC as adc
import Adafruit_BBIO.PWM as pwm
import cv2
import numpy as np
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
# FUNCTIONS ###############################################

def getCameraAngle():
	_, frame = cap.read()
	# Resize the captured frame
	ratio=0.5 #note, 1 = 1:1 ratio
	frame = cv2.resize(frame,None,fx=ratio, fy=ratio,
					   interpolation = cv2.INTER_AREA)
	frameOrig=frame

	#blur with Gauss
	blurVal=5 #should be a positive odd number
	frame=cv2.GaussianBlur(frame,(blurVal,blurVal),0)

	# save only colors in desired range
	try:
		hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #create HSV version #need fix here...
		bb,gg,rr=cv2.split(frame)
		lower=np.array([hL,sL,vL])
		upper=np.array([hU,sU,vU])
		frame=cv2.inRange(hsv,lower,upper) #this is the true result
		#get from HSV to Grayscale
		frameGray=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR) #back to BGR version
		frameGray=cv2.cvtColor(frameGray,cv2.COLOR_BGR2GRAY) #now to Grayscale


		#erode then dilate, aka open the image
		morphVal=11 #should be a positive odd number
		kernel = np.ones((morphVal,morphVal),np.uint8)
		frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)

		# combine and threshold
		frame=cv2.bitwise_and(frameGray,frameGray,mask=frame)

		# now make everything not-black into white.
		ret,frame2=cv2.threshold(frame,0,255,cv2.THRESH_BINARY)

		# find contours in the image
		contours, hierarchy = cv2.findContours(frame2,
			cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		# find moments, get centroid (as long as there are contours)
		if len(contours) > 0:

			# first, want to find the largest contour, by area:
			try:
				loc=contours.index(max(contours,key=cv2.contourArea))
			# print loc
			except ValueError:
				print "ValueError: loc set to 0"
				loc=0

			# ensure never go out of bounds
			if(loc>len(contours)):
				print "OutOfBoundsError: loc set to zero"
				loc=0

			cnt=contours[loc]

			# get frame properties, get largest object properties
			M=cv2.moments(cnt)
			cx=int(M['m10']/M['m00']) # remember A*xavg=int(fn*dA)
			cy=int(M['m01']/M['m00'])
			imgH=frameOrig.shape[0]
			imgW=frameOrig.shape[1]

			# create bounding circle
			(xcirc,ycirc),rcirc=cv2.minEnclosingCircle(cnt)
			center = (int(xcirc),int(ycirc))
			cv2.circle(frameOrig,center,int(rcirc),(255,0,0),1)


			#create 2D error line
			cv2.line(frameOrig,(imgW/2,imgH/2),(cx,cy),(255,0,255),1)




		# END OF IMAGE PROCESSING #############################
	except:
		print "ColorspaceError"


	try:
		cv2.imshow('contours',frameOrig)
	except:
		print "VideoFrameError"

	return cx-imgW




# PIN DEFINES #############################################

# OUTPUTS
pin_green = "P8_7"
pin_yellow = "P8_8"
pin_red = "P8_14"
pin_blue = "P8_15"
cols=[pin_green,pin_yellow,pin_red,pin_blue]

# INPUTS
pin_button = "P8_17"







# SETUP ###################################################
# note: can set globals here

gpio.setup(pin_green,1)
gpio.setup(pin_yellow,1)
gpio.setup(pin_red,1)
gpio.setup(pin_blue,1)
gpio.setup(pin_button,0)
gpio.add_event_detect(pin_button,gpio.BOTH)

adc.setup()
kj.ledINIT()

t=time.time()*1000

# OPEN CV SETUP:
ball1=[13,193,150,22,255,201] # orange balloon
ball2=[17,90,114,25,255,201] #pink balloon
ball3=[41,139,39,75,255,125]#green baloon

ini=ball1

hL=ini[0]
sL=ini[1]
vL=ini[2]
hU=ini[3]
sU=ini[4]
vU=ini[5]
cv2.namedWindow('contours')
cap = cv2.VideoCapture(0) #select video source



# MAIN LOOP ###############################################
while(1):
	getCameraAngle()



	#end program when hit 'Escape' key
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

# MAIN END ################################################
cv2.destroyAllWindows()
gpio.cleanup()
pwm.cleanup()
