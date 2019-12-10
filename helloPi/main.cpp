#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "../../../LSM9DS1_RaspberryPi_library/include/LSM9DS1_Types.h"
#include "../../../LSM9DS1_RaspberryPi_library/include/LSM9DS1.h"


int main(int argc, char *argv[])
{
    long int startTime;
    long int deltaTime;
    struct timespec currentTime;

    LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);
    imu.begin();
    if (!imu.begin())
    {
        fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        exit(EXIT_FAILURE);
    }
    imu.calibrate();

    clock_gettime(CLOCK_REALTIME, &currentTime);
    startTime = currentTime.tv_nsec;

    for (;;)
    {
        while (!imu.gyroAvailable()) ;
        imu.readGyro();
        while(!imu.accelAvailable()) ;
        imu.readAccel();
        //while(!imu.magAvailable()) ;
        //imu.readMag();
        clock_gettime(CLOCK_REALTIME, &currentTime);
        deltaTime = currentTime.tv_nsec - startTime;
        startTime = currentTime.tv_nsec;
        //printf("====================================================================================\n");
        printf("%0.9lf ", deltaTime/1000000000.0F);
        printf("Gyro : %+2.04f, %+2.04f, %+2.04f [deg/s] ", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        printf("Accel: %+2.04f, %+2.04f, %+2.04f [Gs]\r", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
        //printf("Mag: %.4f, %.4f, %.4f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        //sleep(1.0);
    }

    exit(EXIT_SUCCESS);
}
