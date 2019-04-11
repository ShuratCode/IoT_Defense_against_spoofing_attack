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
#define BUFFER_SIZE 300

/* Functions Declaration */
void singleSensorDefence(double max, double min, double standardDev);
void sensorFusionDefence(double mse);
void read_gyro_and_magnetometer();
void read_gyro();
void controlServo(float pGyroDataXYZ[3]);
double calculateLR(float pGyroDataXYZ[3]);
double computeMSE(double *zeta, double *eta);
double *calculateTimeDiffMagnetometer(int16_t pDataXYZ[3]);
double *calculateOmegaCrossB(float pGyroDataXYZ[3], int16_t pDataXYZ[3]);
void check_button();
void updateLeds();
void init();
void read_mag();
void magnoDefence(double mean, double max);

/* Booleans of states */
bool singleSensorState;
bool defenceState;

/* Global Variables Declaration */
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

// Leds and button
DigitalOut led(LED3);
DigitalOut sensorFusionLed(LED1);
DigitalOut singleSensorLed(LED2);
InterruptIn btn(USER_BUTTON);

// Gyro buffers
MyCircularBuffer<double, BUFFER_SIZE> gyroBuf;

// Magno buffers
MyCircularBuffer<double, BUFFER_SIZE> magnoBuf;

// Configure PwmOut
PwmOut _pwm(D0);
PwmOut _pwm2(D1);

/**
 * Main function
 */
int main()
{
    init();
    EventQueue queue;
    btn.rise(&check_button);
    printf("main\n");
    queue.call_every(5, updateLeds);
    queue.dispatch(-1);
}

void init()
{
    defenceState = false;
    singleSensorState = false;
    BSP_GYRO_Init();    //Initiate the gyroscope
    BSP_MAGNETO_Init(); //Initiate the magnetometer
}

void check_button()
{
    if (!defenceState)
    { // no defence
        singleSensorState = true;
        defenceState = true;
        printf("Changed from no defense to single sensor \n");
    }
    else
    { //defence
        if (singleSensorState)
        {
            singleSensorState = false;
            printf("Changed from single sensor to sensor fusion \n");
        }
        else
        {
            defenceState = false;
            printf("changed from sensor fusion to no defence \n");
        }
    }
}

void updateLeds()
{
    if (defenceState)
    {
        if (singleSensorState)
        {
            singleSensorLed = 1;
            sensorFusionLed = 0;
            read_gyro();
        }
        else
        {
            singleSensorLed = 0;
            sensorFusionLed = 1;
            read_gyro_and_magnetometer();
        }
    }
    else
    {
        singleSensorLed = 0;
        sensorFusionLed = 0;
    }
}

/**
 * Read the data from the gyroscope and print the features
 */
void read_gyro()
{
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Collect features when the buffer is full, print them and reset the buffer
    if (gyroBuf.full())
    {
        double gyroMax = gyroBuf.max();
        double gyroMean = gyroBuf.mean();
        double gyroMin = gyroBuf.min();
        double gyroStandardDev = gyroBuf.standardDev();
        double gyroAvgDev = gyroBuf.avgDev();
        singleSensorDefence(gyroMax, gyroMin, gyroStandardDev);
        gyroBuf.reset();
    }

    // Get data from the gyroscope, normalize and push it to the buffer
    gyroBuf.push(sqrt(pow(pGyroDataXYZ[0], 2) + pow(pGyroDataXYZ[1], 2) + pow(pGyroDataXYZ[2], 2)));
}

/**
 * Read the magnetometer readings and calculate the features
 * */
void read_mag()
{
    BSP_MAGNETO_GetXYZ(pDataXYZ);

    // Collect features when the buffer is full, print them and reset the buffer
    if (magnoBuf.full())
    {
        double magnoMax = magnoBuf.max();
        double magnoMean = magnoBuf.mean();
        double magnoMin = magnoBuf.min();
        double magnoSTD = magnoBuf.standardDev();
        double magnoAvgDev = magnoBuf.avgDev();
        magnoDefence(magnoMean, magnoMax);
        magnoBuf.reset();
    }

    // Get data from the gyroscope, normalize and push it to the buffer
    magnoBuf.push(sqrt(pow(pDataXYZ[0], 2) + pow(pDataXYZ[1], 2) + pow(pDataXYZ[2], 2)));
}

/**
 * Will calculate the LR for the single sensor defence.
 * @param pGyroDataXYZ array of readings from the gyroscop
 */
double calculateLR(float pGyroDataXYZ[3])
{
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
void controlServo(float pGyroDataXYZ[3])
{
    float xGyro = pGyroDataXYZ[0];
    _pwm.pulsewidth_us(1500 + xGyro / 1000.0);
    _pwm2.pulsewidth_us(1500 - xGyro / 1000.0);
}

/**
 * Read the data from the magnetometer and the gyroscope, call the sensor fusion algorithm and decide if an attack occurs
 */
void read_gyro_and_magnetometer()
{
    // Get data from the gyroscope and pass it to the servo
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    controlServo(pGyroDataXYZ);

    // Get data from the magnetometer
    BSP_MAGNETO_GetXYZ(pDataXYZ);

    // Compute the features when the mse buffer is full
    if (!magnoBuf.empty())
    {
        double *omegaCrossB = calculateOmegaCrossB(pGyroDataXYZ, pDataXYZ);
        double *timeDiffMagnetometer = calculateTimeDiffMagnetometer(pDataXYZ);
        double mse = computeMSE(omegaCrossB, timeDiffMagnetometer);
        sensorFusionDefence(mse);
        delete[] omegaCrossB;
        delete[] timeDiffMagnetometer;
    }
}


/**
 * Run sensor fusion defence. Getting the MSE and decide if the node
 * is under attack or not.
 */
void sensorFusionDefence(double mse)
{
    if (mse >= 71350000000000000)
    {
        led = 1;
    }
    else
    {
        led = 0;
    }
}

/**
 * Calculate the difference between two reading in the magnetometer.
 * @param pDataXYZ the current readings from the magnetometer.
 * @return an array in size of 3. Each cell holds the difference from the last reading.
 */
double *calculateTimeDiffMagnetometer(int16_t pDataXYZ[3])
{
    double *result = new double[3];
    int16_t *lastMagno;
    result[0] = (pDataXYZ[0] - lastMagno[0]) / 5;
    result[1] = (pDataXYZ[1] - lastMagno[1]) / 5;
    result[2] = (pDataXYZ[2] - lastMagno[2]) / 5;
    return result;
}

/**
 * Calculate the Omega X B vector.
 * @param pGyroDataXYZ array of gyroscope readings data.
 * @param pDataXYZ array of magnetometer reading data.
 * @return the result of the calculation of omega X B.
 */
double *calculateOmegaCrossB(float pGyroDataXYZ[3], int16_t pDataXYZ[3])
{
    float gyroX = pGyroDataXYZ[0];
    float gyroY = pGyroDataXYZ[1];
    float gyroZ = pGyroDataXYZ[2];
    int16_t magX = pDataXYZ[0];
    int16_t magY = pDataXYZ[1];
    int16_t magZ = pDataXYZ[2];

    double cx = -gyroX * magZ + gyroZ * magY;
    double cy = -gyroZ * magX + gyroX * magZ;
    double cz = -gyroX * magY + gyroY * magX;
    double *result = new double[3];
    result[0] = cx;
    result[1] = cy;
    result[2] = cz;
    return result;
}

/**
 * Compute the MSE between the readings of the gyroscope and the magnetometer
 */
double computeMSE(double *zeta, double *eta)
{
    return pow(zeta[0] - eta[0], 2) + pow(zeta[1] - eta[1], 2) + pow(zeta[2] - eta[2], 2);
}

/**
 * This is the implementation of the Machin Learning single sensor defence based on the readings of the gyro only.
 * It is a simple decision tree model. It based on readings measured for 8 seconds.
 * @param max maximum value from the gyro
 * @param min minimum value from the gyro
 * @param standardDev the standard deviation of the gyro readings
 * @return 1 if there is an attack and 0 if not
 */
void singleSensorDefence(double max, double min, double standardDev)
{
    if (max < 247406)
    {
        if (min < 4590.13)
        {
            led = 0;
        }
        else
        {
            if (standardDev < 63222.4)
            {
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

/**
 * Run single sensor defence based on the magnetometer features.
 */
void magnoDefence(double mean, double max){
    printf("mean: %f, max: %f ", mean, max);
    if (mean >= 756.224){
        if (max < 976.145){
            if(mean < 790.531){
                printf("Under attack\n");
            }
            else{
                printf("No attack\n");
            }
        }
        else{
            printf("No attack\n");
        }
    }
    else {
        printf("No attack\n");
    }
}
