import RPi.GPIO as GPIO
import multiprocessing
import time

PIN_LED_RED = 20
PIN_LED_STATUS = 21
PIN_BUZZER = 26

device_on = True



## LED settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_LED_RED, GPIO.OUT)
GPIO.setup(PIN_LED_STATUS, GPIO.OUT)
GPIO.setup(PIN_BUZZER, GPIO.OUT)


def _heartbeat(delay = 0.5, count = 0) :
    global device_on
    while True :
        if device_on == False :
            return
        GPIO.output(PIN_LED_RED, True)
        time.sleep(delay)
        GPIO.output(PIN_LED_RED, False)
        time.sleep(delay)

    
def normalled(status) :
    GPIO.output(PIN_LED_STATUS, status )
    
def _buzzer(duration = 0.1, freq = 500) :
    GPIO.output(PIN_BUZZER, True)
    time.sleep(duration)
    GPIO.output(PIN_BUZZER, False)
    
def buzzer(duration = 0.1) :
    buzz = multiprocessing.Process(target = _buzzer, args = (duration,) )
    buzz.start()
    
def _blink(duration = 0.1) :
    GPIO.output(PIN_LED_RED, True)
    time.sleep(duration)
    GPIO.output(PIN_LED_RED, False)
    
def blink(duration = 0.1) :
    blink = multiprocessing.Process(target = _blink, args = (duration,) )
    blink.start()
    
    
def heartbeat() :
    heart = multiprocessing.Process(target = _heartbeat )
    heart.start()

    
    
if __name__ == "__main__" :
    #heartbeat()
    status = True
    for ii in range(1,5) :
        try :
            normalled(status)
            buzzer(0.1*ii)
            status = not status
            time.sleep(1)
        except :
            break
            
    device_on = False
    GPIO.cleanup()