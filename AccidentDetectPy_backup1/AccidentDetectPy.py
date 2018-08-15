

from Vector import *
from ImuReader import ImuReader
shockThreshole = 1

accel_normaldrive_max = 0.3
accel_quick_braking_max = 0.9
accel_medium_colision = 2.0
accel_sensor_limit = 16

accel_vertial_accept = 0.2

DEBUG_TYPE = 'i2c'

class VehicleData(object) :
    pass
    
## 사고탐지
class Detector(object) :
    def __init__(self, recvType = 'i2c') :
        self.dataList = [] # list of InputData class
        self.dataBuffer = [] 
        self.accidentData = []
        self.accidentBuffer = []
        self.imu = ImuReader(recvType)
        
        # temperal values just for saving detection values when data stream stoppped.
        self.actionTime = 0
        self.magnitude  = 0
        self.time_detected = 0
        
    def recvData(self) :
        self.imu.recvData()
        while self.imu.buffer.empty() == False :
            self.dataList.append( self.imu.buffer.get() )
        
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

        self.actionTime = 0
        
        self.magnitude = 0
        
        
        for data in self.dataList :
            print(data)
            if self.time_detected :
                print(data.accel.size() )
                if data.accel.size() < accel_normaldrive_max :
                    pass
                elif data.accel.size() < accel_quick_braking_max :
                    if data.accel.size_horizental() < accel_normaldrive_max and data.accel.z < accel_vertial_accept :
                        pass
                    else :
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
                    
                self.actionTime += data.deltaTime
                self.accidentBuffer.append(data)
                
                if self.actionTime > 0.3 * 1000 :
                    for index, item in enumerate(self.accidentBuffer) :
                        self.accidentBuffer[index].status = self.magnitude
                    self.accidentData.append(self.accidentBuffer)
                    self.sendData()
                    
                    print("action dectect over")
                    print(self.actionTime, self.magnitude)
                    
                    self.accidentBuffer = []
                    self.actionTime = 0
                    self.time_detected = 0
                    self.magnitude = 0
                    
            else :
                #if data.shock > shockThreshole :
                if data.accel.size() > 0.3 :
                    self.time_detected = data.deltaTime
                    print("shock detected")
                else :
                    pass
        
        self.dataList = []
                    
    def run(self) :
        if self.dataList :
            self.detect()
        else :
            self.recvData()
            
                    
                    
                    
if __name__ == "__main__" :
    if DEBUG_TYPE == 'csv' :
        detect = Detector(recvType = 'csvtest')
        print(detect.imu.recvType)
        detect.recvData()
        print("\ndata recving done")
        #print(detect.dataList )
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
        
        print(detect.accidentData)
        print(detect.accidentBuffer)
        
            