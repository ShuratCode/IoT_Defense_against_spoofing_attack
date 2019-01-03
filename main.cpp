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
void read_gyro_and_magnetometer();
void read_gyro();
void controlServo(float pGyroDataXYZ[3]);
void singleSensorData();
void sensorFusionDataAndDefence();
void singleSensorDefence();
void sensorFusionAlgorithm(int gyroMax, int gyroMean, int gyroMin, int gyroStandardDev, int gyroAvgDev, int magnetometerMax, int magnetometerMean, int magnetometerMin, int magnetometerStandardDev, int magnetometerAvgDev){

/* Global Variables Declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

// Led
DigitalOut led(LED1);
    
// Magnetometer buffer
MyCircularBuffer<int16_t, BUFFER_SIZE> magnoBuf;

// Gyro buffers
MyCircularBuffer<double, BUFFER_SIZE> gyroBuf;

// Configure PwmOut
PwmOut _pwm(D0);

/**
 * Main function
 */
int main() {
    // TODO - check the button that change between the modes
    singleSensorData();
    sensorFusionDataAndDefence();
    singleSensorDefence();
}

/**
 * Initiate the gyroscope and configure eventqueue that call the function that collects the gyroscope data
 */
void singleSensorData(){
    //Initiate the gyroscope
    BSP_GYRO_Init();

    //Configure eventqueue 
    EventQueue queue;
    queue.call_every(5, read_gyro);
    queue.dispatch(-1);
}

/**
 * Read the data from the gyroscope and print the features
 */
void read_gyro(){
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Collect features when the buffer is full, print them and reset the buffer
    if(gyroBuf.full())
    {
        printf("%f %f %f %f %f\n", gyroBuf.max(), gyroBuf.mean(), gyroBuf.min(), gyroBuf.standardDev(), gyroBuf.avgDev());
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

    // Get data from the magnetometer
    BSP_MAGNETO_GetXYZ(pDataXYZ);

    // TODO ask Kevin if we need to calculate features or just compare readings?
    // Collect feature when the buffers are full, call the sensor fusion algorithm and reset the buffers
    if(gyroBuf.full() && magnoBuf.full())
    {
        float gyroMax = gyroBuf.max();
        float gyroMean = gyroBuf.mean();
        float gyroMin = gyroBuf.min();
        float gyroStandardDev = gyroBuf.standardDev();
        float gyroAvgDev = gyroBuf.avgDev();
        float magnetometerMax = magnoBuf.max();
        float magnetometerMean = magnoBuf.mean();
        float magnetometerMin = magnoBuf.min();
        float magnetometerStandardDev = magnoBuf.standardDev();
        float magnetometerAvgDev = magnoBuf.avgDev();

        sensorFusionAlgorithm(gyroMax, gyroMean, gyroMin, gyroStandardDev, gyroAvgDev, magnetometerMax, magnetometerMean, magnetometerMin, magnetometerStandardDev, magnetometerAvgDev);
        
        gyroBuf.reset();
        magnoBuf.reset();
    }

    // Get data from the sensors, normalize and push it to the buffers 
    gyroBuf.push(sqrt(pow(pGyroDataXYZ[0],2) + pow(pGyroDataXYZ[1],2) + pow(pGyroDataXYZ[2],2)));
    magnoBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));

}

/**
 * Implementation of sensor fusion algorithm
 */
void sensorFusionAlgorithm(int gyroMax, int gyroMean, int gyroMin, int gyroStandardDev, int gyroAvgDev, int magnetometerMax, int magnetometerMean, int magnetometerMin, int magnetometerStandardDev, int magnetometerAvgDev){
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
int decisionTree(double max, double min, double standardDev)
{
    if (max < 247406)
    {
        if (min < 4590.13)
        {
            return 0;
        }
        else if (standardDev < 63222.4)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}