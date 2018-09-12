import requests
import time
import datetime
import json

from Vector import *
        
        
        
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
        
class DataSender(object) :
    def __init__(self, host, port, serial, username ) :
        self.host = host
        self.port = port
        self.rootdir = "/2018/capstone/v1/api/"
        
        self.serial = serial
        self.user = username
        
    def makeUrl(self, *args) :
        return self.host + ":" + self.port + self.rootdir + "/".join(args) + "/"
        
        
    def sendDataNormal(self, data = None) :
        url = "http://judo01.asuscomm.com:10001/2018/capstone/v1/api/regulars/"
        headers = {'Content-Type': 'application/json'}
        
        ##data sent to server
        payload = str(json.dumps( \
                {"serial" : str(self.serial),\
                 "accelerationX" : str(data.accel.x), "accelerationY" :str(data.accel.y), "accelerationZ" : str(data.accel.z), \
                 "gyroX" : str(data.gyro.x), "gyroY" : str(data.gyro.y), "gyroZ" : str(data.gyro.z), \
                 "inclination" : str(0), "speed" : str(0)}\
                 ) )
        
        
        
        print(url)
        print(payload)
        
        response = requests.request("POST", url, data=payload, headers=headers)

        print(response.text)
        
    def sendDataAccident(self, data = None) :
        url = "http://judo01.asuscomm.com:10001/2018/capstone/v1/api/accidents/"
        headers = {'Content-Type': 'application/json'}
        payload = str(json.dumps( \
                {"serial" : str(self.serial), "status" : str(data.status), \
                 "accelerationX" : str(data.accel.x), "accelerationY" :str(data.accel.y), "accelerationZ" : str(data.accel.z), \
                 "gyroX" : str(data.gyro.x), "gyroY" : str(data.gyro.y), "gyroZ" : str(data.gyro.z), \
                 "inclination" : str(0),\
                 "longitude" : str(data.longitude), "latitude":str(data.latitude), \
                 "temperature" : str(20) }\
                 ) )
        
        response = requests.request("POST", url, data=payload, headers=headers)

        print(response.text)
        
        
if __name__ == "__main__" :
    ds = DataSender("http://judo01.asuscomm.com", "10001", serial = "1000", username = "Melda")
    data = DriveData(0.2, Vector3(1,2,3), Vector3(4,5,6), dt=0.005, status = 0)
    #ds.getCar()
    #ds.sendDataNormal(data)
    ds.sendDataAccident(data)