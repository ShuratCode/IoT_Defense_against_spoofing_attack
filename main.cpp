/* Includes */
#include "mbed.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"


DigitalOut led(LED1);

/* Defines */
#define INTERVAL_SIZE 20
#define BUFFER_SIZE 10


/*Declaration */

void checkData();


int main() {
    /* Local variables declaration */
    int16_t pDataXYZ[3] = {0};
    float pGyroDataXYZ[3] = {0};
    int interval = 0;
    //magnetometer buffers
    CircularBuffer<int16_t, BUFFER_SIZE> magnoXBuf;
    CircularBuffer<int16_t, BUFFER_SIZE> magnoYBuf;
    CircularBuffer<int16_t, BUFFER_SIZE> magnoZBuf;

    //accelerometer buffers
    CircularBuffer<int16_t, BUFFER_SIZE> acceloXBuf;
    CircularBuffer<int16_t, BUFFER_SIZE> acceloYBuf;
    CircularBuffer<int16_t, BUFFER_SIZE> acceloZBuf;

    //gyro buffers
    CircularBuffer<float, BUFFER_SIZE> gyroXBuf;
    CircularBuffer<float, BUFFER_SIZE> gyroYBuf;
    CircularBuffer<float, BUFFER_SIZE> gyroZBuf;

    //Initiate the sensors
    BSP_MAGNETO_Init();
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();

    //main loop
    while (1) {
        if (interval == INTERVAL_SIZE) { //call the function to check circle queue
            led = 1;
            checkData();
            interval = 0;
            led = 0;
        } else { //read the sensors and fill the buffer queue
            BSP_MAGNETO_GetXYZ(pDataXYZ);
            magnoXBuf.push(pDataXYZ[0]);
            magnoYBuf.push(pDataXYZ[1]);
            magnoZBuf.push(pDataXYZ[2]);
           
            
            BSP_GYRO_GetXYZ(pGyroDataXYZ);
            gyroXBuf.push(pGyroDataXYZ[0]);
            gyroYBuf.push(pGyroDataXYZ[1]);
            gyroZBuf.push(pGyroDataXYZ[2]);
            
            BSP_ACCELERO_AccGetXYZ(pDataXYZ);
            acceloXBuf.push(pDataXYZ[0]);
            acceloYBuf.push(pDataXYZ[1]);
            acceloZBuf.push(pDataXYZ[2]);

            interval = interval + 1;

        }


    }

}

void checkData() {
    printf("\n\n\n THIS IS CHECK DATA \n\n\n");
}


