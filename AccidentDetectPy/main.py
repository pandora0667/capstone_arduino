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


starttime = time.time()
gpstime = time.time()

while True :
    if time.time() > gpstime + 5 :
        gpsThread._run_once()
        detectorThread.recvGPS(gpsThread.get() )
        gpstime = time.time()
        
    imuThread._run_once()
    detectorThread.put(imuThread.get() )
    detectorThread._run_once()
    

#imuThread.on = False
#detectorThread.on = False

print("\n\n")
print(detectorThread.accidentBuffer)
print("\n\n")
print(detectorThread.accidentData)
print("\n\n")
print(detectorThread.normalData)