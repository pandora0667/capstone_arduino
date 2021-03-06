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
int piezo; // Raw ADC data from shock sensor

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
float shock; // shock sensor data

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

float temperature;
float pressure;
float atm;
float altitude;

// calced datas
int time_last = 0;
int deltatime = 0;

void setup() {
    Wire.begin();
    Serial.begin(9600);    // serial 통신 9600bps                    

    accelgyro.initialize();
    // verify connection
    // Serial.println("Testing device connections...");
    
    delay(1000);
    Serial.println("     ");
    //  Mxyz_init_calibrated ();
    
}


void loop() {
    // get data from sensors
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();
    getShockSensor_data();
    
    //calc speed and angles
    deltatime = millis() - time_last;
    time_last = millis();
    calcAngleFromGyro(deltatime);
    //JSON Parsing... 
    StaticJsonBuffer<700> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
  
   if(calib_done) {
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
  
      root["attack"] = shock;    // calced Shock by sensor. Analog Piezo Shock Sensor. 핀번호 A0
  
      root.printTo(Serial);
      Serial.print("\n");
    }

    else {
      // Serial.println("{""calibration"":""calibrating""}");
      JsonArray& AccelCalJSON = root.createNestedArray("Accel_calib");
      // Calibration. device must not move or get force while calibrating
      // Kalman Filter will be added soon
      // calibrate Accelerometer
      Axyz_calib[0] = Axyz[0];
      Axyz_calib[1] = Axyz[1];
      Axyz_calib[2] = Axyz[2];
      AccelCalJSON.add(Axyz_calib[0]);
      AccelCalJSON.add(Axyz_calib[1]);
      AccelCalJSON.add(Axyz_calib[2]);
      // calibrate gyro
      JsonArray& gyroCalJSON = root.createNestedArray("gyro_calib");
      Gxyz_calib[0] = Gxyz[0];
      Gxyz_calib[1] = Gxyz[1];
      Gxyz_calib[2] = Gxyz[2];
      gyroCalJSON.add(Gxyz_calib[9]);
      gyroCalJSON.add(Gxyz_calib[1]);
      gyroCalJSON.add(Gxyz_calib[2]);
      
      // calibrate Initial Angle from Accelerometer
      /*
       * not implemented
       */
       
      // calibrate piezo
      JsonArray& piezoCalibJSON = root.createNestedArray("piezo_calib");
      piezo_calib = analogRead(A0);
      piezoCalibJSON.add(Axyz_calib[2]);

      // print result
      JsonArray& calibration = root.createNestedArray("calibration");
      calibration.add("successful");
      
      root.printTo(Serial);
      Serial.print("\n");
      calib_done = 1;
      
    }
    
    delay(1000);
}

void getHeading(void) {
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated () {

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}

void get_calibration_Data () {
    for (int i = 0; i < sample_num_mdate; i++) {
        get_one_sample_date_mxyz();

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
    }
    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];
    
    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}

void get_one_sample_date_mxyz() {
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
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

void getCompass_Data(void) {
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated () {
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}

void calcAngleFromGyro(int dt) {
  int ii = 0; // temp value

  for(ii=0;ii<3;ii++) {
      Wxyz[ii] = Wxyz[ii] + (0.001*dt*Gxyz[ii] );
      
  }
}

void getShockSensor_data() {
    piezo = analogRead(A0);
    shock = Volt2G*(piezo - piezo_calib);
}
