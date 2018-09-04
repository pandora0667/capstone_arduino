import threading
#import requests
import sys
import os
import time
import datetime


import Detector
import ImuReader
import GpsReader



## initialise 

imuThread = ImuReader.ImuReader()
detectorThread = Detector.Detector()
gpsThread = GpsReader.GpsReader()

imuThread.start()
detectorThread.start()
gpsThread.start()

starttime = time.time()

while True :
    if imuThread.buffer.empty() == False :
        detectorThread.put(imuThread.buffer.get() )  
    
    
    time.sleep(0.1)
    detectorThread.recvGPS(gpsThread.get() )  
    time.sleep(0.1)

#imuThread.on = False
#detectorThread.on = False

print("\n\n")
print(detectorThread.accidentBuffer)
print("\n\n")
print(detectorThread.accidentData)
print("\n\n")
print(detectorThread.normalData)