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
#define BUFFER_SIZE 200

/*Declaration */
void checkData();
void controlServo(float pGyroDataXYZ[3]);
void read_magnetometer();
void read_gyro_accelerometer();
int decisionTree(double max, double min, double standardDev);

/* Global variables declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

//magnetometer buffer
MyCircularBuffer<int16_t, BUFFER_SIZE> magnoBuf;

//accelerometer buffer
MyCircularBuffer<int16_t, BUFFER_SIZE> acceloBuf;

//gyro buffers
MyCircularBuffer<double, BUFFER_SIZE> gyroBuf;

//Configure PwmOut
PwmOut _pwm(D0);

/**
 * Main function. Initialize the sensors and set up the event queue
 */
int main()
{

    //Initiate the sensors
    //BSP_MAGNETO_Init();
    BSP_GYRO_Init();
    //BSP_ACCELERO_Init();

    // Configure eventqueue
    EventQueue queue;
    //queue.call_every(25, read_magnetometer);
    queue.call_every(5, read_gyro_accelerometer);
    queue.dispatch(-1);
}

/**
 * Give the data that had been collected from the gyro
 * and use it to control the servo engine
 * @param pGyroDataXYZ array of the data from the gyro. On 3 axes
 */
void controlServo(float pGyroDataXYZ[3])
{
    float xGyro = pGyroDataXYZ[0];
    _pwm.pulsewidth_us(1500 + xGyro / 1000.0);
}

/**
 * Read the data from the magnetometer
 */
void read_magnetometer()
{
    BSP_MAGNETO_GetXYZ(pDataXYZ);
    magnoBuf.push(sqrt(pow(pDataXYZ[0], 2) + pow(pDataXYZ[1], 2) + pow(pDataXYZ[2], 2)));
}

/**
 * Read the data from the accelerometer and from the gyro
 */
void read_gyro_accelerometer()
{
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    if (gyroBuf.full())
    {
        printf("%f %f %f %f %f\n", gyroBuf.max(), gyroBuf.mean(), gyroBuf.min(), gyroBuf.standardDev(), gyroBuf.avgDev());
        gyroBuf.reset();
    }
    gyroBuf.push(sqrt(pow(pGyroDataXYZ[0], 2) + pow(pGyroDataXYZ[1], 2) + pow(pGyroDataXYZ[2], 2)));
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
