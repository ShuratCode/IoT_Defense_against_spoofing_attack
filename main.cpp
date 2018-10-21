/* Includes */
#include "mbed.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "Servo.h"


DigitalOut led(LED1);

/* Defines */
#define BUFFER_SIZE 10


/*Declaration */

void checkData();
void controlServo(Servo servo);



int main() {
    /* Local variables declaration */
    int16_t pDataXYZ[3] = {0};
    float pGyroDataXYZ[3] = {0};

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

    //Configure eventqueue to run checkData() every 20ms
    EventQueue queue;
    queue.call_every(20, checkData);

    //Configure Servo motor
    Servo servo(D0);

    //main loop
    while (1) {
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
    }

}

void checkData() {
    printf("\n\n\n THIS IS CHECK DATA \n\n\n");
}

void controlServo(Servo servo){
     for(int i=0; i<100; i++) {
             servo = i/100.0;
            wait(0.01);
         }
         for(int i=100; i>0; i--) {
             servo = i/100.0;
             wait(0.01);
         }
}


