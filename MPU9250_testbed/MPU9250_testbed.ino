#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

// base voltage ADC value of shock sensor
const float Volt2G = 0.00537;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU9250 accelgyro;
I2Cdev   I2C_M;

void get_one_sample_date_mxyz();
void getAccel_Data(void);
void getGyro_Data(void);
void getCompass_Data(void);
void getCompassDate_calibrated ();

void initAngle(); // initialise angle from gravity. not implemented yet
void calcAngleFromGyro(int dt);
void getShockSensorValue() ;

uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

// variable for calibration.
float Axyz_calib[3];
float Gxyz_calib[3];
int piezo_calib = 600;
int calib_done = 0;

// final data
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
float Wxyz[3]; // Angle of device

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

// calced datas
int time_last = 0;
int deltatime = 0;

void setup() {
    Wire.begin();
    Serial.begin(9600);    // serial 통신 9600bps                    
    accelgyro.initialize();  
    
    delay(1000);
}

void loop() {
    // get data from sensors
    Serial.flush();
    getAccel_Data();
    getGyro_Data();
    getShockSensor_data();
    
    //calc speed and angles
    deltatime = millis() - time_last;
    time_last = millis();
    calcAngleFromGyro(deltatime);
  
   if(calib_done) {
    
     //JSON Parsing... 
      StaticJsonBuffer<700> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      
      JsonArray& acceleration = root.createNestedArray("acceleration");
      acceleration.add(Axyz[0]);
      acceleration.add(Axyz[1]);
      acceleration.add(Axyz[2]);
  
      JsonArray& gyro = root.createNestedArray("gyro");
      gyro.add(Gxyz[0]);
      gyro.add(Gxyz[1]);
      gyro.add(Gxyz[2]);
  
      JsonArray& angle = root.createNestedArray("angle");
      angle.add(Wxyz[0]);
      angle.add(Wxyz[1]);
      angle.add(Wxyz[2]); 
  
      root["shock"] = getShockSensor_data();  
      
      root.printTo(Serial);
      Serial.print("\n");
    }

    else {
      Axyz_calib[0] = Axyz[0];
      Axyz_calib[1] = Axyz[1];
      Axyz_calib[2] = Axyz[2];
      
      // calibrate gyro
      Gxyz_calib[0] = Gxyz[0];
      Gxyz_calib[1] = Gxyz[1];
      Gxyz_calib[2] = Gxyz[2];

      // calibrate piezo
      piezo_calib = analogRead(A0);

      StaticJsonBuffer<100> successBuffer;
      JsonObject& root = successBuffer.createObject();
      
      root["calibration"] = "success"; 
      root.printTo(Serial);
      Serial.print("\n");

      calib_done = 1;
    }
    
    delay(1000);
}

void getAccel_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = ((double) ax / 16384 ) - Axyz_calib[0];
    Axyz[1] = ((double) ay / 16384 ) - Axyz_calib[1];
    Axyz[2] = ((double) az / 16384 ) - Axyz_calib[2];
}

void getGyro_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = ((double) gx * 250 / 32768 ) - Gxyz_calib[0];
    Gxyz[1] = ((double) gy * 250 / 32768 ) - Gxyz_calib[1];
    Gxyz[2] = ((double) gz * 250 / 32768 ) - Gxyz_calib[2];
}

void calcAngleFromGyro(int dt) {
  int i = 0; // temp value

  for(i=0; i<3; i++) {
      Wxyz[i] = Wxyz[i] + (0.001*dt*Gxyz[i] );
  }
}

float getShockSensor_data() {
    float piezo = analogRead(A0);
    return Volt2G*(piezo - piezo_calib);
}
