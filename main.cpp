// Coded by Luís Afonso 11-04-2019
#include "mbed.h"
#include "BufferedSerial.h"
#include "rplidar.h"
#include "Robot.h"
#include "Communication.h"

BufferedSerial pc(SERIAL_TX, SERIAL_RX);
RPLidar lidar;
BufferedSerial se_lidar(PA_9, PA_10);
PwmOut rplidar_motor(D3);

int main()
{
    float odomX, odomY, odomTheta;
    struct RPLidarMeasurement data;
    
    pc.set_baud(115200);
    init_communication(&pc);

    // Lidar initialization
    rplidar_motor.period(0.001f);
    lidar.begin(se_lidar);
    lidar.setAngle(0,360);

    pc.printf("Program started.\n");
        
    lidar.startThreadScan();
    
    while(1) {
        // poll for measurements. Returns -1 if no new measurements are available. returns 0 if found one.
        if(lidar.pollSensorData(&data) == 0)
        {
            pc.printf("%f\t%f\t%d\t%c\n", data.distance, data.angle, data.quality, data.startBit); // Prints one lidar measurement.
        }
       wait(0.01); 
    }
}