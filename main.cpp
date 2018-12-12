/* Includes */
#include "mbed.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"

#include <cmath>
#include "MyCircularBuffer.h"
#include <iostream>
using namespace std;


DigitalOut led(LED1);

/* Defines */
#define BUFFER_SIZE 521


/*Declaration */

void checkData();
void controlServo(float pGyroDataXYZ[3]);
void read_magnetometer();
void read_gyro_accelerometer();

/* Global variables declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};
    
//magnetometer buffer
MyCircularBuffer<int16_t, BUFFER_SIZE> magnoBuf;

//accelerometer buffer
MyCircularBuffer<int16_t, BUFFER_SIZE> acceloBuf;

//gyro buffers
MyCircularBuffer<float, BUFFER_SIZE> gyroBuf;

//Configure PwmOut
PwmOut _pwm(D0);

/**
 * Main function. Initialize the sensors and set up the event queue
 */
int main() {

    //Initiate the sensors
    BSP_MAGNETO_Init();
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();

    //Configure eventqueue 
    EventQueue queue;
    queue.call_every(2000, checkData);
    queue.call_every(25, read_magnetometer);
    queue.call_every(80, read_gyro_accelerometer);
    queue.dispatch(-1);
}

/**
 * Check the buffers and decide if we have an attack
 */ 
void checkData() {
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
 * Read the data from the magnetometer
 */
void read_magnetometer(){
    BSP_MAGNETO_GetXYZ(pDataXYZ);
    magnoBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));
}

/**
 * Read the data from the accelerometer and from the gyro
 */
void read_gyro_accelerometer(){
<<<<<<< HEAD
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    //acceloBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));
    //if(gyroBuf.full())
    //{
    //    printf("Max Mean Min StandardDev AvgDev \n %f %f %f %f %f", gyroBuf.max(), gyroBuf.mean(), gyroBuf.min(), gyroBuf.standardDev(), gyroBuf.avgDev());
    //    exit(1);
    //}
    //else{
    //    gyroBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));
    //    printf("num of values: %u\n", gyroBuf.size());
    //}

    controlServo(pGyroDataXYZ);
=======
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);
    //acceloBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));
    if(gyroBuf.full())
    {
        printf("%f %f %f %f %f\n", gyroBuf.max(), gyroBuf.mean(), gyroBuf.min(), gyroBuf.standardDev(), gyroBuf.avgDev());

    }
    gyroBuf.push(sqrt(pow(pDataXYZ[0],2) + pow(pDataXYZ[1],2) + pow(pDataXYZ[2],2)));
>>>>>>> 2bc77424cc5c93209d85c413658bd93f1ac85767
}


