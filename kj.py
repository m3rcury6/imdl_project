import Adafruit_BBIO.ADC as adc
# avg #####################################################
def avg(xarray):
    # assumes that an array is being passed
    n=len(xarray)
    sumavg=0.0
    for i in range(0,len(xarray)):
        sumavg=sumavg+xarray[i]
    return sumavg/n

# pyt #####################################################
def pyt(a,b):
    return (a*a+b*b)**0.5

# stdev ###################################################
def stdev(xarray):
    n=len(xarray)
    xavg=avg(xarray)
    sumstd=0.0
    for i in range(0,n):
        sumstd=sumstd+(xarray[i]-xavg)
    sumstd=sumstd/n
    return sumstd**0.5

# map #####################################################
def map(original,inLow,inHigh,outLow,outHigh):
    m=(outHigh-outLow)/(inLow-outLow)
    b=outHigh-inHigh*m
    return m*original+b

# simpleSolve #############################################
def simpleSolve(kjgFn,yd,xl,xu):
    # use bisection method to do some simple root finding
    def ee1(x,y):
      #choose ee1 if yd==0
      return abs(x)
    #def ee1

    def ee2(x,y):
      #choose ee2 if yd!=0
      return abs((x+0.0)/y)
    #def ee2

    def f2(x):
      return kjgFn(x)-yd +1.0-1.0
    #def f2
    e=1.0;
    i=0;
    while (e>1e-10 and i<10000):
      xr=(xl+xu)/2.0

      if(f2(xl)*f2(xr)<0):
        xu=xr
      else:
        xl=xr
      if(yd==0):
        e=ee1(f2(xr),yd)
      else:
        e=ee2(f2(xr),yd)
      #end ifstatement

      i=i+1 #dont need it yet
    # end whileloop
    return xr
  #def simpleSolve

# slopeLinReg #############################################
def slopeLinReg(xarray,yarray):
    n=len(yarray)
    xavg=avg(xarray)
    yavg=avg(yarray)
    betaC1=range(0,n)
    betaC2=range(0,n)
    for i in range(0,n):
        betaC1[i]=(xarray[i]-xavg)*(yarray[i]-yavg)
        betaC2[i]=(xarray[i]-xavg)**2
    return sum(betaC1)/sum(betaC2) #return lin regression slope, beta

# interceptLinReg #########################################
def interceptLinReg(xarray,yarray):
    beta=slope(xarray,yarray)
    xavg=avg(xarray)
    yavg=avg(yarray)
    return yavg-beta*xavg # return lin regression intercept, alpha

# irCalibrate #############################################
def irCalibrate(xarr,yarr):

  def getIRBeta(xarr,yarr):
    for i in range(0,len(xarr)):
      xarr[i]=1.0/(xarr[i]) #REMEMBER TO USE PERIODS FOR NON-INTEGERS
    #get average for beta calc
    xavg=avg(xarr)
    yavg=avg(yarr)

    #array initalization
    betaC1=range(0,len(yarr))
    betaC2=range(0,len(yarr))

    #get beta
    for i in range(0,len(yarr)):
      betaC1[i]=(xarr[i]-xavg)*(yarr[i]-yavg)
      betaC2[i]=pow((xarr[i]-xavg),2)
    #end forloop

    beta = sum(betaC1)/sum(betaC2)
    return beta
  #def getIRBeta

  def simpleSolve(kjgFn,yd,xl,xu):
    # part of simpleSolve
    def ee1(x,y):
      #choose ee1 if yd==0
      return abs(x)
    #def ee1

    def ee2(x,y):
      #choose ee2 if yd!=0
      return abs((x+0.0)/y)
    #def ee2

    def f2(x):
      return kjgFn(x)-yd +1.0-1.0
    #def f2
    e=1.0;
    i=0;
    xr=0.0
    while (e>1e-10 and i<10000):
      xr=(xl+xu)/2.0

      if(f2(xl)*f2(xr)<0):
        xu=xr
      else:
        xl=xr
      if(yd==0):
        e=ee1(f2(xr),yd)
      else:
        e=ee2(f2(xr),yd)
      #end ifstatement

      i=i+1 #dont need it yet
    # end whileloop
    return xr
  #def simpleSolve


  beta=getIRBeta(xarr,yarr)
  #print beta

  #gotta start looping here

  b1=1.0
  a=0.0

  #xp=range(0,len(x))
  #for i in range(0,len(x)):
  #  xp[i]=beta*b1/y[i]+a
  ##create xp
  #xd1=xp[1]-x[1]
  #xd2=xp[len(x)]-x[len(x)]

  def f(xx):
    return (beta*xx/yarr[0]-1/xarr[0])-(beta*xx/yarr[len(yarr)-1]-1/xarr[len(yarr)-1])

  b1=simpleSolve(f,0,0,10)
  b=b1*beta
  #print b
  a=1/xarr[0]-b/yarr[0]
  #print a
  for i in range(0,len(xarr)):
    xarr[i]=1.0/(xarr[i]) #REMEMBER TO USE PERIODS FOR NON-INTEGERS


  return (a,b)

# irReadcm ################################################
irReadVarA=-2.1485 #default value
irReadVarB=7.4789 #default value

# new default values:
# a = -2.5485
# b = 7.4789

def irReadcm(pinString):
    var1=adc.read(pinString)
    return irReadVarB/var1+irReadVarA #calibration constants

# TruthTurn ###############################################

# write here the algoritm used










# end of file #############################################