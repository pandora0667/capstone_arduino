import gpsd

import threading
from time import sleep
import queue

class GpsReader(object) :
    def __init__(self ) :
        gpsd.connect()
        self.data = None
        self.buffer = queue.Queue(1)
        self.on = True
        sleep(1)
        
    def recvSignal(self) :
        signal = gpsd.get_current()
        data = {}
        
        ## can't get GPS signal
        if signal.mode < 2 :
            return None
            
        if signal.mode >= 2 :
            data["latitude"] = str(signal.lat)
            data["longitude"] = str(signal.lon)
            data["speed"] = str(signal.hspeed)
            data["time"] = str(signal.time)
            
        self.data = data
        
        return data
        
    def put(self, data) :
        if self.buffer.empty() == False :
            self.buffer.get()
            
        self.buffer.put(data)
        
    def get(self) :
        if self.buffer.empty() :
            return None
        else :
            return self.buffer.get()
        
    def run(self) :
        while self.on :
            self.put(self.recvSignal() )
            sleep(0.97)
            
    def _run_once(self) :
        self.put(self.recvSignal() )
        
        
if __name__ == "__main__" :
    gpsThread = GpsReader()
    gpsThread.start()
    for ii in range(1,100) :
        print(gpsThread.get() )
        sleep(1)