#include "mbed.h"

// Sensors drivers present in the BSP library
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"

DigitalOut led(LED1);

//DEFINE
#define INTERVAL_SIZE 20

//Declaration

void checkData();

int main()
{
	//Initiate variables
  int16_t pDataXYZ[3] = {0};
  float pGyroDataXYZ[3] = {0};
	int interval = 0;
	
	//Initiate the sensors
  BSP_MAGNETO_Init();
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
	

   while(1) {
		 if (interval == INTERVAL_SIZE)
		 {
			 led = 1;
			 checkData();
			 interval = 0;
			 led = 0;
		 }
		 else {
			BSP_MAGNETO_GetXYZ(pDataXYZ);
			printf("\nMAGNETO_X = %d\n", pDataXYZ[0]);
			printf("MAGNETO_Y = %d\n", pDataXYZ[1]);
			printf("MAGNETO_Z = %d\n", pDataXYZ[2]);

			BSP_GYRO_GetXYZ(pGyroDataXYZ);
			printf("\nGYRO_X = %.2f\n", pGyroDataXYZ[0]);
			printf("GYRO_Y = %.2f\n", pGyroDataXYZ[1]);
			printf("GYRO_Z = %.2f\n", pGyroDataXYZ[2]);

			BSP_ACCELERO_AccGetXYZ(pDataXYZ);
			printf("\nACCELERO_X = %d\n", pDataXYZ[0]);
			printf("ACCELERO_Y = %d\n", pDataXYZ[1]);
			printf("ACCELERO_Z = %d\n", pDataXYZ[2]);
			interval = interval + 1;
		
		 }
		 
		 

    }
    
}

void checkData(){
	printf("\n\n\n THIS IS CHECK DATA \n\n\n");
}


