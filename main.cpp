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
void controlServo(float pGyroDataXYZ[3]);
void sensorFusionAlgorithm(double gyroMax, double gyroMean, double gyroMin, double gyroStandardDev, double gyroAvgDev, double magnetometerMax, double magnetometerMean, double magnetometerMin, double magnetometerStandardDev, double magnetometerAvgDev);
double computeMSE(float gyroX, float gyroY, float gyroZ, int16_t magnoX, int16_t magnoY, int16_t magnoZ);

/* Global Variables Declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

// Led
DigitalOut led(LED1);

// Gyro buffers
MyCircularBuffer<double, BUFFER_SIZE> gyroBuf;

// MSE buffers
MyCircularBuffer<double, BUFFER_SIZE> mseBuf;

// Configure PwmOut
PwmOut _pwm(D0);

/**
 * Main function
 */
int main() {
    // TODO - check the button that change between the modes
    bool collectData = false;
    //singleSensor(collectData);
    sensorFusionDataAndDefence();
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

    // Collect featurs when the buffer is full, print them and reset the buffer
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
        double gyroMin = gyroBuf.min();
            singleSensorDefence(gyroMax, gyroMin, gyroStandardDev);
            //singleSensorDefence(247405, 4590.2, 63222.5);
        }
        gyroBuf.reset();
    }

    // Get data from the gyroscope, normalize and push it to the buffer 
    gyroBuf.push(sqrt(pow(pGyroDataXYZ[0],2) + pow(pGyroDataXYZ[1],2) + pow(pGyroDataXYZ[2],2)));
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
    queue.call_every(5, read_gyro_and_magnetometer);
    queue.dispatch(-1);
}

/**
 * Read the data from the magnetometer and the gyroscope, call the sensor fusion algorithm and decide if an attack occurs
 */
void read_gyro_and_magnetometer(){
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Get data from the megnetometer
    BSP_MAGNETO_GetXYZ(pDataXYZ);

    // Compute the features when the mse buffer is full
    if(mseBuf.full())
    {
        double mseMax = mseBuf.max();
        double mseMean = mseBuf.mean();
        double mseMin = mseBuf.min();
        double mseStandardDev = mseBuf.standardDev();
        double mseAvgDev = mseBuf.avgDev();
    }
    double mse = computeMSE(pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2], pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    printf("%f \n", mse);
    mseBuf.push(mse);
}

/**
 * Compute the MSE between the readings of the gyroscope and the magnetometer
 */
double computeMSE(float gyroX, float gyroY, float gyroZ, int16_t magnoX, int16_t magnoY, int16_t magnoZ){
    return pow(gyroX - magnoX, 2) + pow(gyroY - magnoY, 2) + pow(gyroZ - magnoZ, 2);
}


/**
 * Implementation of sensor fusion algorithm
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
            led = 0;
        }
        else{
            if (standardDev < 63222.4)
            {
                printf("Under attack \n");
                led = 1;
            }
            else
            {
                led = 0;
            }
        }
    }
    else
    {
        led = 0;
    }
}