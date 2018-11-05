import threading
import queue
import time

from Vector import *
from ImuReader import ImuReader, ImuData
import dataSender
import RPi.GPIO as GPIO

from OutputControl import buzzer, blink, normalled

shockThreshole = 1

accel_normaldrive_max = 0.3
accel_quick_braking_max = 0.9
accel_medium_colision = 2.0
accel_sensor_limit = 16

accel_vertial_accept = 0.2

DEBUG_TYPE = 'i2c'


class DriveData(object) :
    def __init__(self, shock, accel_vector, gyro_vector, dt = 0, status = 0, latitude = 0, longitude = 0, speed = 0 ) :
        self.shock = shock
        self.accel = accel_vector
        self.gyro = gyro_vector
        self.deltaTime = dt
        self.status = status
        self.speed = speed
        self.latitude = latitude
        self.longitude = longitude
    def __repr__(self) :
        return str(self.accel) + ", " + str(self.speed)
        
## 사고탐지
class Detector(object) :
    def __init__(self, recvType = 'berry') :
    
        self.dataBuffer = queue.Queue() # list of InputData class
        self.accidentBuffer = []
        
        
        self.normalData = []
        self.accidentData = []
        
        self.gpsbuffer = queue.Queue(1)
        self.speed = -1
        self.latitude = -1
        self.longitude = -1
        
        # temperal values just for saving detection values when data stream stoppped.
        self.actionTime = 0
        self.magnitude  = 0
        self.time_detected = 0
        
        self.sender = dataSender.DataSender("http://just2.asuscomm.com", "10001", serial = "1000", username = "Raininn")
        
        self.on = True
        self.led = True
        
        
    def recvGPS(self, data ) :
        if self.gpsbuffer.empty() == False :
            self.gpsbuffer.get()
        self.gpsbuffer.put(data) 
        return
        
    def sendGPS(self, data) :
        pass
        
    def put(self, data) :
        if self.gpsbuffer.empty() == False :
            gps = self.gpsbuffer.get()
            
            if gps :
                self.speed = gps["speed"]
                self.latitude = gps["latitude"]
                self.longitude = gps["longitude"]
                print("gps updated {} {} {}".format(self.latitude, self.longitude, self.speed) )
    
        temp = DriveData(data.shock, data.accel, data.gyro, data.deltaTime, \
                        status = data.status, latitude = self.latitude, longitude = self.longitude, \
                        speed = self.speed )
        self.dataBuffer.put(temp)
        
    def sendData(self, format = "JSON") :
        for ii in self.normalData :
            self.sender.sendDataNormal( ii)
            
        for jj in self.accidentData :
            for kk in jj :
                self.sender.sendDataAccident( kk)
            
        self.normalData = []
        self.accidentData = []
        
        
        
        
    def detect(self) :
        ## 데이터 전체를 탐색
        ## 충격값이 기준치 이상일경우 가속도값 확인
        ## 가속도 값이 기준시간 이내에 일정수치 이상일경우 사고로 간주.
        
        if self.dataBuffer.empty() != True :
            data = self.dataBuffer.get()
            print(data)
            if self.time_detected :
                print(data.accel.size() )
                if data.accel.size() < accel_normaldrive_max :
                    pass
                elif data.accel.size() < accel_quick_braking_max :
                    if data.accel.size_horizental() < accel_normaldrive_max and data.accel.z < accel_vertial_accept :
                        pass
                    else :
                        if self.magnitude < 1 :
                            self.magnitude = 1
                elif data.accel.size() < accel_medium_colision :
                    if self.magnitude < 2 :
                        self.magnitude = 2
                elif data.accel.size() < accel_sensor_limit :
                    if self.magnitude < 7 :
                        self.magnitude = 7
                else :
                    print("wrong data")
                    pass
                    
                self.actionTime += data.deltaTime
                self.accidentBuffer.append(data)
                
                if self.actionTime > 0.3 :
                    for index, item in enumerate(self.accidentBuffer) :
                        self.accidentBuffer[index].status = self.magnitude
                    self.accidentData.append(self.accidentBuffer)
                    
                    print("action dectect over")
                    print(self.actionTime, self.magnitude)
                    
                    self.accidentBuffer = []
                    self.actionTime = 0
                    self.time_detected = 0
                    self.magnitude = 0
                    
                    ##detection output here
                    buzzer()
                    blink()
                    
                    
            else :
                #if data.shock > shockThreshole :
                if data.accel.size() > 0.3 :
                    self.time_detected = data.deltaTime
                    print("shock detected")
                else :
                    self.normalData.append(data) 
                    
        normalled(self.led)
        self.led = not self.led
                    
    def run(self) :
        while self.on == True :
            try :
                if self.dataBuffer.empty() :
                    print("no data in detector")
                    time.sleep(0.2)
                else :
                    print("detecting accident")
                    self.detect()
            
            except :
                raise
        
    def _run_once(self) :
        self.detect()
        self.sendData()
                    
if __name__ == "__main__" :
    if DEBUG_TYPE == 'csv' :
        detect = Detector(recvType = 'csvtest')
        print(detect.imu.recvType)
        detect.recvData()
        print("\ndata recving done")
        #print(detect.dataBuffer )
        detect.detect()
        print("\ndetction done")
        print(detect.accidentData)
        print(detect.accidentBuffer)
    else :
        detect = Detector(recvType = 'berry')
        while True :
            try :
                detect.run()
            except:
                raise
                break
        print(detect.actionTime)
        print(detect.accidentData)
        print(detect.accidentBuffer)
        
            