/* Includes */
#include "mbed.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"

#include <cmath>
#include "MyCircularBuffer.h"
#include <iostream>
using namespace std;

/* Defines */
#define BUFFER_SIZE 200

/* Functions Declaration */
void singleSensor(bool collectData);
void sensorFusionDataAndDefence();
void singleSensorDefence(double max, double min, double standardDev);
void read_gyro_and_magnetometer();
void read_gyro(bool collectData);
double calculateLR(float pGyroDataXYZ[3]);
void controlServo(float pGyroDataXYZ[3]);
void sensorFusionAlgorithm(double gyroMax, double gyroMean, double gyroMin, double gyroStandardDev, double gyroAvgDev, double magnetometerMax, double magnetometerMean, double magnetometerMin, double magnetometerStandardDev, double magnetometerAvgDev);
double calculateLR(float pGyroDataXYZ[3]);
double computeMSE(double* zeta, double* eta);
double* calculateTimeDiffMagnetometer(int16_t pDataXYZ[3]);
double* calculateOmegaCrossB(float pGyroDataXYZ[3], int16_t pDataXYZ[3]);

/* Global Variables Declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

// Led
DigitalOut led(LED1);

// Gyro buffers
MyCircularBuffer<double, BUFFER_SIZE> gyroBuf;

// Magno buffers
MyCircularBuffer<int16_t*, BUFFER_SIZE> magnoBuf;

// Configure PwmOut
PwmOut _pwm(D0);

/**
 * Main function
 */
int main() {
    // TODO - check the button that change between the modes
    bool collectData = false;
    singleSensor(collectData);
    //sensorFusionDataAndDefence();
}

/**
 * Initiate the gyroscope and configure eventqueue that call the function that collects the gyroscope data
 */
void singleSensor(bool collectData){
    //Initiate the gyroscope
    BSP_GYRO_Init();

    //Configure eventqueue 
    EventQueue queue;
    queue.call_every(5, read_gyro, collectData);
    queue.dispatch(-1);
}

/**
 * Read the data from the gyroscope and print the features
 */
void read_gyro(bool collectData){
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Collect features when the buffer is full, print them and reset the buffer
    if(gyroBuf.full())
    {
        double gyroMax = gyroBuf.max();
        double gyroMean = gyroBuf.mean();
        double gyroMin = gyroBuf.min();
        double gyroStandardDev = gyroBuf.standardDev();
        double gyroAvgDev = gyroBuf.avgDev();
        if(collectData){
            printf("%f %f %f %f %f\n", gyroMax, gyroMean, gyroMin, gyroStandardDev, gyroAvgDev);
        }
        else{
            singleSensorDefence(gyroMax, gyroMin, gyroStandardDev);
            //singleSensorDefence(247405, 4590.2, 63222.5);
        }
        gyroBuf.reset();
    }

    // Get data from the gyroscope, normalize and push it to the buffer 
    gyroBuf.push(calculateLR(pGyroDataXYZ));
}

/**
 * Will calculate the LR for the single sensor defence. 
 * @param pGyroDataXYZ array of readings from the gyroscop
 */ 
double calculateLR(float pGyroDataXYZ[3]){
    double xPow = pow(pGyroDataXYZ[0], 2);
    double yPow = pow(pGyroDataXYZ[1], 2);
    double zPow = pow(pGyroDataXYZ[2], 2);
    double lr = sqrt(xPow + yPow + zPow);
    return lr;
}

/**
 * Give the data that had been collected from the gyro
 * and use it to control the servo engine
 * @param pGyroDataXYZ array of the data from the gyro. On 3 axes
 */
void controlServo(float pGyroDataXYZ[3]){
     float xGyro = pGyroDataXYZ[0];
     _pwm.pulsewidth_us(1500+xGyro/1000.0);
}

/**
 * Initiate the sensors and configure eventqueue that call the function that collects the sensors data and checks if an attack occurs
 */
void sensorFusionDataAndDefence(){
    //Initiate the sensors
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();

    //Configure eventqueue 
    EventQueue queue;
    queue.call_every(10, read_gyro_and_magnetometer);
    queue.dispatch(-1);
}

/**
 * Read the data from the magnetometer and the gyroscope, call the sensor fusion algorithm and decide if an attack occurs
 */
void read_gyro_and_magnetometer(){
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Get data from the magnetometer
    BSP_MAGNETO_GetXYZ(pDataXYZ);

    // Compute the features when the mse buffer is full
    /*if(mseBuf.full())
    {
        double mseMax = mseBuf.max();
        double mseMean = mseBuf.mean();
        double mseMin = mseBuf.min();
        double mseStandardDev = mseBuf.standardDev();
        double mseAvgDev = mseBuf.avgDev();
    }*/
    if(!magnoBuf.empty()){
        double* omegaCrossB = calculateOmegaCrossB(pGyroDataXYZ, pDataXYZ);
        double* timeDiffMagnetometer = calculateTimeDiffMagnetometer(pDataXYZ);
        double mse = computeMSE(omegaCrossB, timeDiffMagnetometer);
        //printf("%f \n", mse);
        if (mse >= 71350000000000000){
            printf("Under Attack\n");
            led = 1;
        }
        else{
            led = 0;
        }
        delete [] omegaCrossB;
        delete [] timeDiffMagnetometer;
    }
    magnoBuf.push(pDataXYZ);
}

//TODO add comment
double* calculateTimeDiffMagnetometer(int16_t pDataXYZ[3]){
    double* result = new double[3];
    int16_t* lastMagno;
    magnoBuf.peek(lastMagno);
    result[0] = (pDataXYZ[0] - lastMagno[0]) / 5;
    result[1] = (pDataXYZ[1] - lastMagno[1]) / 5;
    result[2] = (pDataXYZ[2] - lastMagno[2]) / 5;
    return result;
}

//TODO add comment
double* calculateOmegaCrossB(float pGyroDataXYZ[3], int16_t pDataXYZ[3]){
    float gyroX = pGyroDataXYZ[0];
    float gyroY = pGyroDataXYZ[1];
    float gyroZ = pGyroDataXYZ[2];
    int16_t magX = pDataXYZ[0];
    int16_t magY = pDataXYZ[1];
    int16_t magZ = pDataXYZ[2];

    double cx = -gyroX * magZ + gyroZ * magY;
    double cy = -gyroZ * magX + gyroX * magZ;
    double cz = -gyroX * magY + gyroY * magX;
    double* result = new double[3];
    result[0] = cx;
    result[1] = cy;
    result[2]= cz;
    return result;
}

/**
 * Compute the MSE between the readings of the gyroscope and the magnetometer
 */
double computeMSE(double* zeta, double* eta){
    return pow(zeta[0] - eta[0], 2) + pow(zeta[1] - eta[1], 2) + pow(zeta[2] - eta[2], 2);
}


/**
 * Implementation of sensor fusion algorithm
 * @param gyroMax the maximum feature calculated from the gyro 
 * @param gyroMean the mean feature calculated from the gyro
 * @param gyroMin the min feature calculated form the gyro
 * @param gyroStandardDev the std feature calculated from the gyro
 * @param 
 */
void sensorFusionAlgorithm(double gyroMax, double gyroMean, double gyroMin, double gyroStandardDev, double gyroAvgDev, double magnetometerMax, double magnetometerMean, double magnetometerMin, double magnetometerStandardDev, double magnetometerAvgDev){
    //TODO complete
}

/**
 * This is the implementation of the Machin Learning single sensor defence based on the readings of the gyro only.
 * It is a simple decision tree model. It based on readings measured for 8 seconds.
 * @param max maximum value from the gyro
 * @param min minimum value from the gyro
 * @param standardDev the standard deviation of the gyro readings
 * @return 1 if there is an attack and 0 if not
 */
void singleSensorDefence(double max, double min, double standardDev){
    if (max < 247406)
    {
        if (min < 4590.13)
        {
            printf("No attack\n");
            led = 0;
        }
        else{
            if (standardDev < 63222.4)
            {
                printf("Under attack\n");
                led = 1;
            }
            else
            {
            printf("No attack\n");
                led = 0;
            }
        }
    }
    else
    {
        printf("No attack\n");
        led = 0;
    }
}