import threading
import requests
import sys
import os
import time
import datetime


import Detector
import gpsreader



## initialise 
detector = Detector.Detector()

gpsreader = gpsreader.init()

