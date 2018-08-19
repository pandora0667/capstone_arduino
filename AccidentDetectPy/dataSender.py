import requests
import time
import datetime
import json

from Vector import *

http = "http://"
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
               "acceleration : " + str(self.accel)  + " , " + \
               "gyro : " + str(self.gyro)
      
    def __repr__(self) :
        return str(self)
        
class DataSender(object) :
    def __init__(self, host, port, * , serial, username ) :
        self.host = host
        self.port = port
        self.rootdir = "/2018/capstone/v1/api/"
        
        self.serial = serial
        self.user = username
        
    def makeUrl(self, *args) :
        return http + self.host + ":" + self.port + self.rootdir + "/".join(args) + "/"
        
    def getCar(self) :
        url = self.makeUrl("cars", self.user)
        print(url)
        headers = {'Content-Type': 'application/json'}

        #response = requests.request("GET", url, headers=headers)

        #print(response.text)
        
    def sendDataNormal(self, data) :
        url = self.makeUrl("regulars")
        headers = {'Content-Type': 'application/json'}
        ##data sent to server
        "serial=0001&accelerationX=10&accelerationY=10&accelerationZ=10&inclination=10"

        payload = "serial={}&accelerationX={}&accelerationY={}&accelerationZ={}&inclination={}".format( \
            self.serial, data.accel.x, data.accel.y, data.accel.z, 0) 
            
        print(payload)
        
        #response = requests.request("POST", url, data=payload, headers=headers)

        #print(response.text)
        
    def sendDataAccident(self, data) :
        url = "self.urlaccidents/"

        payload = "serial=0001&status=2&accelerationX=50&accelerationY=50&accelerationZ=50&inclination=50&latitude=35.1567&longitude=127.4156"
        headers = {'Content-Type': 'application/json'}

        response = requests.request("POST", url, data=payload, headers=headers)

        print(response.text)
        
if __name__ == "__main__" :
    ds = DataSender("host", "65535", serial = "6974", username = "Melda")
    data = ImuData(0.002, 8, -1, 0, 9.8, 5,6,7)
    ds.getCar()
    ds.sendDataNormal(data)