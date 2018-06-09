import csv

from math import sqrt, pi, sin, cos, tan, atan
import time


accel_normaldrive_max = 0.3

accel_quick_braking_max = 0.9

accel_sensor_limit = 15.5

class VehicleData(object) :
    def __init__(self, weight) :
        self.weight = weight

## x is longitugal(front/back), y is latheral(left/right), z is vertical(up/down)
class Vector3(object) :
    def __init__(self, x, y, z) :
        self.x = x
        self.y = y
        self.z = z
    def size(self) :
        return sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def size_horizental(self) :
        return sqrt(self.x**2 + self.y**2)
    
    def __str__(self) :
        return "x : " + str(self.x) + ", y : " + str(self.y) + ", z : " + str(self.z)
    
        
        
class ImuData(object) :
    def __init__(self, *args) :
        self.deltaTime = args[0] ## 기존 측정과 현재 측정의 시간간격. 초 단위.
        self.shock = args[1] ## 충격센서 값
        self.accel = Vector3( args[2], args[3], args[4] ) ## 가속도값
        self.gyro = Vector3( args[5], args[6], args[7] ) ## gyro, 각가속도 값
    def __str__(self) :
        return "time : " + str(self.deltaTime) + " , " + \
               "shock : " + str(self.shock) + " , " + \
               "acceleration" + str(self.accel)  + " , " + \
               "gyro" + str(self.gyro)
      
      
## IMU 에서 데이터 받아서 변환해서 넘겨줌
## 필터를 파이 등에 올릴경우 이쪽에 넣으면 됨.
class ImuReader() :
    pass

## 
    
## 사고탐지
class Detector(object) :
    def __init__(self) :
        self.dataList = [] # list of InputData class
        self.accidentData = []
        self.imu = ImuReader()
        
    def recvData(self) :
        pass
        
    def recvDataCsv(self, filename = "data.csv") :
        file = open(filename, 'r')
        for line in file :
            print(line)
            if not line :
                continue
            dataTemp = line.replace(" ",'').strip().split(",")
            print(dataTemp)
            if len(dataTemp) < 4 :
                continue
            data = []
            for ii in dataTemp :
                data.append( float(ii.strip() ) )
            
            print('result')
            print(data )
            
            self.dataList.append( ImuData(data[0], 0, data[1], data[2], data[3], 0,0,0) )
            
        
    def sendData(self, format = "JSON") :
        pass
        
    def printData(self) :
        pass
        
    def addAccData(self, data) :
        self.dataList.append(data)
        
    def detect(self) :
        ## 데이터 전체를 탐색
        ## 충격값이 기준치 이상일경우 가속도값 확인
        ## 가속도 값이 기준시간 이내에 일정수치 이상일경우 사고로 간주.

        actionTime = 0
        time_detected = 0
        
        magnitude = 0
        detected = False
        
        
        for data in self.dataList :
            if detected :
                if data.accel.size() < accel_normaldrive_max :
                    pass
                elif data.accel.size() < accel.quick_braking_max :
                    if magnitude < 1 :
                        magnitude = 1
                elif data.accel.size() < accel_medium_colision :
                    if magnitude < 2 :
                        magnitude = 2
                elif data.accel.size() < accel_sensor_limit :
                    if magnitude < 7 :
                        magnitude = 7
                else :
                    print("wrong data")
                    pass
                    
                #actionTime += data.deltaTime
                actionTime += data.deltaTime - time_detected
                self.accidentData.append(data)
                
                if actionTime > 0.3 * 1000 :
                    self.sendData()
                    
                    print("action dectect over")
                    print(actionTime)
                    print(magnitude)
                    
                    self.accidentData = []
                    actionTime = 0
                    time_detected = 0
                    detected = False
                    magnitude = 0
                    
            else :
                if data.shock > shockThreshole :
                    detected = True
                    time_detected = data.deltaTime
                    print("shock detected")
                else :
                    pass


if __name__ == "__main__" :
    detect = Detector()
    detect.recvDataCsv()
    print("parsed")
    #print(detect.dataList)
    detect.detect()
    
    print(detect.accidentData)
    