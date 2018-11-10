#include <cstdio>
#include <thread>
#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#endif
#ifdef __GNUC__
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
#endif

int main(int argc, char *argv[])
{
    ImuData d;

    // Gets a LpmsSensorManager instance
    LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
    // 
    // DEVICE_LPMS_B        LPMS-B (Bluetooth)
    // DEVICE_LPMS_U        LPMS-CU / LPMS-USBAL (USB)
    // DEVICE_LPMS_C        LPMS-CU / LPMS-CANAL(CAN bus)
    // DEVICE_LPMS_BLE      LPMS-BLE (Bluetooth low energy)
    // DEVICE_LPMS_RS232    LPMS-UARTAL (RS-232)
    // DEVICE_LPMS_B2       LPMS-B2
    // DEVICE_LPMS_U2       LPMS-CU2/URS2/UTTL2/USBAL2 (USB)
    // DEVICE_LPMS_C2       LPMS-CU2/CANAL2 (CAN)

    // Connects to LPMS-B2 sensor with address 00:11:22:33:44:55 
    //LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B2, "00:11:22:33:44:55");
    // Connects to LPMS-CURS2 sensor try virtual com port 
    LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U2, "/dev/ttyUSB0");


    while(1) 
    {		 
        // Checks, if sensor is connected
        if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
            lpms->hasImuData()) 
        {
            // Reads quaternion data
            d = lpms->getCurrentData();

            // Shows data
            printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f, x=%f, y=%f, z=%f\n", 
                d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3],
                d.r[0], d.r[1], d.r[2]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Removes the initialized sensor
    manager->removeSensor(lpms);

    // Deletes LpmsSensorManager object 
    delete manager;

    return 0;
}

