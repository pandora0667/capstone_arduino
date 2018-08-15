#import sys
#import asyncio
import IMU
import datetime
#import os
import time
import queue
from math import pi, atan2, degrees , sqrt , sin, cos, asin


from Vector import Vector3
from Filters import *



## constants
X_GAIN = 0.732 * 0.001 # [g/LSB]
G_GAIN = 0.070  	# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      	# Complementary filter constant
MAG_LPF_FACTOR = 0.9 	# Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.9 	# Low pass filter constant for accelerometer

# 
IMU_UPSIDE_DOWN = 0


## compass calibration value
magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0

timelast = datetime.datetime.now()

class _ImuReader(object) :
    pass
    
    
        
## 센서에서 받아온 raw 데이터.
class ImuData(object) :
    def __init__(self, *args) :
        self.deltaTime = args[0] ## 기존 측정과 현재 측정의 시간간격. 밀리초(0.001초) 단위.
        self.shock = args[1] ## 충격센서 값
        self.accel = Vector3( args[2], args[3], args[4] ) ## 가속도값
        self.gyro = Vector3( args[5], args[6], args[7] ) ## gyro, 각가속도 값
        self.status = 0
    def __str__(self) :
        return "time : " + str(self.deltaTime) + " , " + \
               "status : " + str(self.status) + ", " + \
               "shock : " + str(self.shock) + " , " + \
               "acceleration" + str(self.accel)  + " , " + \
               "gyro" + str(self.gyro)
      
    def __repr__(self) :
        return str(self)
      
## IMU 에서 데이터 받아서 변환해서 넘겨줌
## 필터를 파이 등에 올릴경우 이쪽에 넣으면 됨.
class ImuReader() :
    def __init__(self, recvType = 'berry') :
        self.recvType = recvType.lower()
        self.filterAccX = FilterEMA(use_lpf = True, lpf_factor = ACC_LPF_FACTOR)
        self.filterAccY = FilterEMA(use_lpf = True, lpf_factor = ACC_LPF_FACTOR)
        self.filterAccZ = FilterEMA(use_lpf = True, lpf_factor = ACC_LPF_FACTOR)
        self.filterAngleX = KalmanFilterGyroAccel()
        self.filterAngleY = KalmanFilterGyroAccel()
        self.filterMagX = FilterEMA(use_lpf = True, lpf_factor = MAG_LPF_FACTOR)
        self.filterMagY = FilterEMA(use_lpf = True, lpf_factor = MAG_LPF_FACTOR)
        self.filterMagZ = FilterEMA(use_lpf = True, lpf_factor = MAG_LPF_FACTOR)
        
        IMU.detectIMU()
        IMU.initIMU()
        
        self.timelast = datetime.datetime.now()
        
        self.buffer = queue.Queue()
       
        ## set recv type
        if self.recvType == 'csvtest' :
            self.recvData = self._recvDataCsvTest
        elif self.recvType == 'berry' :
            self.recvData = self._recvDataBerry
        else :
            print("unknown Input Channel")
            raise ValueError
        
    def getAccelFiltered(self) :
        return self.filterAccX.get(), self.filterAccY.get(), self.filterAccZ.get()
        
    def getMagnetFilterd(self) :
        return self.filterMagX.get(), self.filterMagY.get(), self.filterMagZ.get()
        
        
    def recvData(self) :
        pass
        
    def _recvDataCsvTest(self, filename = "data.csv") :
        file = open(filename, 'r')
        for line in file :
            print(line)
            if not line :
                continue
            dataTemp = line.replace(" ",'').strip().split(",")
            print(dataTemp)
            if len(dataTemp) < 4 :
                continue
            dataRaw = []
            for ii in dataTemp :
                dataRaw.append( float(ii.strip() ) )
            
            print('result Raw')
            print(dataRaw , "" )
            self.filterAccX.put(dataRaw[1])
            self.filterAccY.put(dataRaw[2])
            self.filterAccZ.put(dataRaw[3])
            
            data = [dataRaw[0], self.filterAccX.get(), self.filterAccY.get(), self.filterAccZ.get() ]
            print(data, "")
            self.buffer.put( ImuData(data[0], 0, data[1], data[2], data[3], 0,0,0) )
    
    def _recvDataRaw(self) :
        return  IMU.readACCx(), IMU.readACCy(), IMU.readACCz(), \
                IMU.readGYRx(), IMU.readGYRy(), IMU.readGYRz(), \
                IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()
    
    def _recvDataBerry(self) :
        ## read values from IMU
        AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY , MagZ = self._recvDataRaw()
        
        ## convert raw data
        
        
        ## apply compass calibration
        MagX -= (magXmin + magXmax) / 2
        MagY -= (magYmin + magYmax) / 2
        MagZ -= (magZmin + magZmax) / 2

    
        ## get time between loops. deltaTime(dt)
        timecurrent = datetime.datetime.now() - self.timelast
        self.timelast = datetime.datetime.now() 
        dt = timecurrent.microseconds/ (1000000*1.0)
        
        ## put raw accel/mag data to filter (LPF/EMA)
        ## put LPF bypassed one to get angles from XL /Mag
        self.filterAccX.put(AccX)
        AccX = self.filterAccX.get(bypass_lpf = True)
        self.filterAccY.put(AccY)
        AccY = self.filterAccY.get(bypass_lpf = True)
        self.filterAccZ.put(AccZ)
        AccZ = self.filterAccZ.get(bypass_lpf = True)
        
        self.filterMagX.put(MagX)
        MagX = self.filterMagX.get(bypass_lpf = True)
        self.filterMagY.put(MagY)
        MagY = self.filterMagY.get(bypass_lpf = True)
        self.filterMagZ.put(MagZ)
        MagZ = self.filterMagZ.get(bypass_lpf = True)
        
        
        
        ## get angle by acceleration
        
        if not IMU_UPSIDE_DOWN :
            # Logo facing down
            AccAngleX = degrees(atan2(AccY, AccZ) )
            AccAngleY = degrees(atan2(AccZ, AccX) + pi )
            # Logo facing up
            AccAngleX = degrees(atan2(-AccY, -AccZ) )
            AccAngleY = degrees(atan2(-AccZ, -AccX) + pi )
                
        ## change Y axis '0' points up. for ease of reading
        if AccAngleY > 90 :
            AccAngleY -= 270
        else :
            AccAngleY += 90
        
        # correct heading if IMU is upside down
        if IMU_UPSIDE_DOWN :
            MagY = -MagY
        
        # get Angle From Kalman filter and 
        AnglePrevX = self.filterAngleX.get()
        AnglePrevY = self.filterAngleY.get()
        
        
        self.filterAngleX.filter(AccAngleX, GyrX * G_GAIN, dt)
        self.filterAngleY.filter(AccAngleY, GyrY * G_GAIN, dt)
        
        GyrXkalman = (self.filterAngleX.get() - AnglePrevX ) / dt
        GyrYkalman = (self.filterAngleY.get() - AnglePrevY ) / dt
        
        
        ## Tilt Compensated Heading ##
        # normalise acceleration
        if not IMU_UPSIDE_DOWN :
            AccXnorm = AccX / sqrt( (AccX**2)  + (AccY ** 2) + (AccZ ** 2) )
            AccYnorm = AccY / sqrt( (AccX**2)  + (AccY ** 2) + (AccZ ** 2) )
        else :
            AccXnorm = -AccX / sqrt( (AccX**2)  + (AccY ** 2) + (AccZ ** 2) )
            AccYnorm = AccY / sqrt( (AccX**2)  + (AccY ** 2) + (AccZ ** 2) )
            
        # get pitch/roll
        
        pitch = asin(AccXnorm)
        roll = -asin(AccYnorm / cos(pitch) )
        
        # get new tilt compensated values
        MagXcomp = ( MagX * cos(pitch) ) + ( MagZ * sin(pitch) )
        
        # compensate compass/accelerometer defference between LSM9DS0 and LSM9DS1
        if IMU.LSM9DS0 :
            MagYcomp = MagX * (sin(roll) * sin(pitch)) + MagY * cos(roll) - (MagZ * sin(roll) * cos(pitch) )
        else :
            MagYcomp = MagX * (sin(roll) * sin(pitch)) + MagY * cos(roll) + (MagZ * sin(roll) * cos(pitch) )
        
        headingTC = degrees( atan2(MagYcomp, MagXcomp) )
        
        if headingTC < 0 :
            headingTC += 360
            
        ##put data to buffer so detector can take
        self.buffer.put(
                        ImuData(dt, 0 , \
                            self.filterAccX.get() * X_GAIN, self.filterAccY.get() * X_GAIN, self.filterAccZ.get() * X_GAIN, \
                            GyrXkalman, GyrYkalman, GyrZ * G_GAIN )
                        )
                               
        ## printout
        #print(dt)
        #print(self.getAccelFiltered(), self.getMagnetFilterd() )
        ##print(self.filterAngleX.get(), self.filterAngleY.get() )
        #print("CH %5.2f" % (headingTC) )         
        return
            
    
    
    
if __name__ == "__main__" :
    reader = ImuReader()
    while True :
        reader.recvData()
    
    
    
    
    
    
    
    

