#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "MPU9250.h"

#include <Wire.h>
#include <math.h>

// all datas from imu is RAW int data
MPU9250 imu;
I2Cdev I2C_M;

/* Kalman filter */
struct GyroKalman {
  /* These variables represent our state matrix x */
  float x_angle, x_bias;

  /* Our error covariance matrix */
  float P_00, P_01, P_10, P_11;
  
  /*
  * Q is a 2x2 matrix of the covariance. Because we
  * assume the gyro and accelerometer noise to be independent
  * of each other, the covariances on the / diagonal are 0.
  * Covariance Q, the process noise, from the assumption
  * x = F x + B u + w
  * with w having a normal distribution with covariance Q.
  * (covariance = E[ (X - E[X])*(X - E[X])' ]
  * We assume is linear with dt
  */
  float Q_angle, Q_gyro;

  /*
  * Covariance R, our observation noise (from the accelerometer)
  * Also assumed to be linear with dt
  */
  float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle);
void predict(struct GyroKalman *kalman, float dotAngle, float dt) ;
float update(struct GyroKalman *kalman, float angle_m);

void getAccel_Data(void);
void getGyro_Data(void);

/*
* R represents the measurement covariance noise. In this case,
* it is a 1x1 matrix that says that we expect 0.3 rad jitter
* from the accelerometer.
*/
static const float R_angle = 0.3;     //.3 default

/*
* Q is a 2x2 matrix that represents the process covariance noise.
* In this case, it indicates how much we trust the acceleromter
* relative to the gyros
*/
static const float Q_angle = 0.01;  //0.01 (Kalman)
static const float Q_gyro = 0.04; //0.04 (Kalman)


/* time */
unsigned long prevTime = 0;
unsigned long nowTime = 0;

/* calibration data */
int initSize = 5;
int initIndex = 0;
int initGyroX[5] = {0,0,0,0,0};
int initGyroY[5] = {0,0,0,0,0};
int initGyroZ[5] = {0,0,0,0,0};

int GyroCalX = 0;
int GyroCalY = 0;
int GyroCalZ = 0;

/* shock sensor ADC to voltage */
const float Volt2G = 0.00537;


/* temp variables for recieving data from MPU9250 */
uint8_t buffer_m[6];
int16_t imu_a_x, imu_a_y, imu_a_z;
int16_t imu_g_x, imu_g_y, imu_g_z;

/* shock Sensor variabls */
int piezo = 0;
int piezoCal = 0;
float shock = 0.0;


/* accelerometer filter variables */
float accelEMAalpha = 0.5;
int accelTabSize = 5;
float accelAlphas[5] = {0,0,0,0,0};
int accelTabX[5] =  {0,0,0,0,0};
int accelTabY[5] =  {0,0,0,0,0};
int accelTabZ[5] =  {0,0,0,0,0};
int accelTabCount = 0;

int accelCalX = 0;
int accelCalY = 0;
int accelCalZ = 0;

float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;

float accelFinalX = 0;
float accelFinalY = 0;
float accelFinalZ = 0;


float accelAngleX = 0;
float accelAngleY = 0;
float accelAngleZ = 0;





void setup()
{
  Serial.begin(230400);
  Wire.begin();
  //init gyro and kalman filter for gyro
  initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);

  //init accelerometer and EMA values
  char ii; // temp var
  for(ii =0; ii < accelTabSize; ii++ ) {
    accelAlphas[ii] = pow((accelEMAalpha), ii+1);
  }
  
  imu.initialize();
  delay(1000);
  getShockData();
  piezoCal = piezo;
  Serial.print("\n");

}

void loop()
{
  getImuData();
  getShockData();

  nowTime = millis();

  double deltaTime = (float)(nowTime - prevTime) * 0.001 ;
  
  
  if(prevTime > 0) {
    
    /* accelerometer code here */
    if(accelTabCount < accelTabSize) {
        accelTabX[accelTabCount] = imu_a_x;
        accelTabY[accelTabCount] = imu_a_y;
        accelTabZ[accelTabCount] = imu_a_z;
        accelTabCount++;
    }

    if(accelTabCount == accelTabSize) {
        char ii;
        for(ii =0; ii < accelTabSize; ii++ ) {
            accelCalX += accelTabX[ii];
            accelCalY += accelTabY[ii];
            accelCalZ += accelTabZ[ii];
        }
        accelCalX /= accelTabSize;
        accelCalY /= accelTabSize;
        accelCalZ /= accelTabSize;
        for(ii =0; ii < accelTabSize; ii++ ) {
            accelTabX[ii] -= accelCalX;
            accelTabY[ii] -= accelCalY;
            accelTabZ[ii] -= accelCalZ;
        }
        accelTabCount = accelTabSize+1;
    }
   
    else {
      char ii;
      float tempEMA_X = 0;
      float tempEMA_Y = 0;
      float tempEMA_Z = 0;

      
      //push datas in accelTabX and put new data from sensor at the end of accelTabX
      for(ii = 0; ii <accelTabSize-1; ii++) {
          accelTabX[ii] = accelTabX[ii+1];
          accelTabY[ii] = accelTabY[ii+1];
          accelTabZ[ii] = accelTabZ[ii+1];
      }
      accelTabX[accelTabSize - 1] = imu_a_x - accelCalX;
      accelTabY[accelTabSize - 1] = imu_a_y - accelCalY;
      accelTabZ[accelTabSize - 1] = imu_a_z - accelCalZ;

      //calc EMA of acceleration.
      // EMA_now = alpha*(val1 + ((1-alpha)**1)val2 + ((1-alpha)**2)val3 + ....
      for(ii = accelTabSize; ii > 0; ii--) {
          tempEMA_X += ((float)accelAlphas[accelTabSize - ii] * (float)accelTabX[accelTabSize-ii] );
          tempEMA_Y += ((float)accelAlphas[accelTabSize - ii] * (float)accelTabY[accelTabSize-ii] );
          tempEMA_Z += ((float)accelAlphas[accelTabSize - ii] * (float)accelTabZ[accelTabSize-ii] );
      }
      // low pass filter for removing gravity
      gravityX = 0.1 * tempEMA_X + 0.9 * gravityX;
      gravityY = 0.1 * tempEMA_Y + 0.9 * gravityY;
      gravityZ = 0.1 * tempEMA_Z + 0.9 * gravityZ;
      
      accelFinalX = 0.061 * 0.001 * (tempEMA_X - gravityX);
      accelFinalY = 0.061 * 0.001 * (tempEMA_Y - gravityY);
      accelFinalZ = 0.061 * 0.001 * (tempEMA_Z - gravityZ);

      accelAngleX = atan((float)accelFinalY/(float)accelFinalZ) * RAD_TO_DEG;
      accelAngleY = atan((float)accelFinalZ/(float)accelFinalX) * RAD_TO_DEG;
      accelAngleZ = atan((float)accelFinalX/(float)accelFinalZ) * RAD_TO_DEG;
      
    }

    /* gyro code here */
    float gx1=0, gy1=0, gz1 = 0;
    float gx2=0, gy2=0, gz2 = 0;


    gx2 = 8.75 * 0.001 * imu_g_x;
    gy2 = 8.75 * 0.001 * imu_g_y;
    gz2 = 8.75 * 0.001 * imu_g_z;

    predict(&angX, gx2, deltaTime);
    predict(&angY, gy2, deltaTime);
    predict(&angZ, gz2, deltaTime);

    gx1 = update(&angX, accelAngleX) ;
    gy1 = update(&angY, accelAngleY) ;
    gz1 = update(&angZ, accelAngleZ) ;

    /////////////////////////////////////////////////////////////////////////////
    //  gyro/Angle init.
    /////////////////////////////////////////////////////////////////////////////
    if(initIndex < initSize) {
      initGyroX[initIndex] = gx1;
      initGyroY[initIndex] = gy1;
      initGyroZ[initIndex] = gz1;
      if(initIndex == initSize - 1) {
        int sumX = 0; int sumY = 0; int sumZ = 0;
        for(int k=1; k <= initSize; k++) {
          sumX += initGyroX[k];
          sumY += initGyroX[k];
          sumZ += initGyroX[k];
        }

        GyroCalX -= sumX/(initSize -1);
        GyroCalY -= sumY/(initSize -1);
        GyroCalZ = (sumZ/(initSize -1) - GyroCalZ);
      }
      initIndex++;
    }
    
    /////////////////////////////////////////////////////////////////////////////
    //  adjust calibration of gyro.
    /////////////////////////////////////////////////////////////////////////////
    else {
        gx1 += GyroCalX;
        gy1 += GyroCalY;
        gz1 += GyroCalZ;

        /* actions here */
    }

       

    
    
    /* output under here */
    
    // Serial.print("time (ms) : ");
    Serial.print(nowTime);
    Serial.print(", ");
    Serial.print(shock);
    Serial.print(", ");
    Serial.print(accelFinalX);
    Serial.print(", ");
    Serial.print(accelFinalY);
    Serial.print(", ");
    Serial.print(accelFinalZ);
    Serial.print(", ");
    Serial.print(gx1, DEC);
    Serial.print(F(", "));
    Serial.print(gy1, DEC);
    Serial.print(F(", "));
    Serial.print(gz1, DEC);
    Serial.print("\n");
   // Serial.println(F(""));
    

    /* debug output here */
    /*
    char ii;
    Serial.println("accelAngle");
    Serial.print(accelAngleX);
    Serial.print(F(", "));
    Serial.print(accelAngleY);
    Serial.print(F(", "));
    Serial.print(accelAngleZ);
    Serial.print("\n");

    Serial.println("gyroAngle");
    Serial.print(gx1);
    Serial.print(F(", "));
    Serial.print(gy1);
    Serial.print(F(", "));
    Serial.print(gz1);
    Serial.print("\n");
    */
  
  }

  prevTime = nowTime;
  delay(10);
  //delay(100); // for debug
  
} // End of loop()

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
  kalman->Q_angle = Q_angle;
  kalman->Q_gyro = Q_gyro;
  kalman->R_angle = R_angle;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}

/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
  kalman->x_angle += dt * (dotAngle - kalman->x_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_gyro;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float update(struct GyroKalman *kalman, float angle_m) {
  const float y = angle_m - kalman->x_angle;
  const float S = kalman->P_00 + kalman->R_angle;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  kalman->x_angle += K_0 * y;
  kalman->x_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_angle;
}


void getImuData(void) {
    imu.getMotion6(&imu_a_x, &imu_a_y, &imu_a_z, &imu_g_x, &imu_g_y, &imu_g_z);
}

void getShockData(void) {
    piezo = analogRead(A0);
    shock = Volt2G*(piezo - piezoCal);   
}


