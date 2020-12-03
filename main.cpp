#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
//USLEEP
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <termios.h>
#include <curl/curl.h>
#include <thread>
#include <vector>

#include "Assets/i2c.cpp"
#include "Assets/crono.cpp"
#include "Assets/comms/coms.cpp"

#include "Assets/Mahony/MahonyAHRS.cpp"

#include "Modules/pca/pca.cpp"
#include "Modules/gps/gps.cpp"
#include "Modules/mpu6050/mpu6050.cpp"
#include "Modules/mpu9250/mpu9250.cpp"

using namespace std;
using namespace chrono;

//Global Timers
clp chrono_global;
clp chrono_main;
clp chrono_mpu1;
clp chrono_mpu2;
clp chrono_mpu3;

//Initialize IMU's
mpu6050 mpu_1(0x68);
mpu6050 mpu_2(0x69);
mpu9250 mpu_3(0x68);

//PCA
pca servo0(0, 70, 500, 180);
pca servo1(1, 70, 500, 180);
pca servo2(2, 70, 500, 180);
pca servo3(3, 70, 500, 180);
pca servo4(4, 70, 500, 180);
pca servo5(5, 70, 500, 180);
pca servo6(6, 70, 500, 180);

//Initialize GPS
serial gps("/dev/ttyS3", B19200);
const char delims = ',';

//Airplane State
float speed = 0;
float longitude = 0;
float latitude = 0;
float altitude = 0;
float time_p = 0;
int gps_state = 0;
int sats_using = 0;

void tokenize(string const &str, const char delim,
              vector<string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

void gps_listener()
{
    while (true)
    {
        string gps_line = gps.read_until("\n");
        size_t pos = 0;
        int i = 0;
        string token;
        string buff[20];
        while ((pos = gps_line.find(delims)) != string::npos)
        {
            buff[i] = gps_line.substr(0, pos);
            gps_line.erase(0, pos + 1);
            i++;
        }
        if (buff[0] == "$GNVTG")
        {
            //https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_VTG.html
            speed = stof(buff[7]);
        }
        else if (buff[0] == "$GNGGA")
        {
            //https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_GGA.html
            time_p = stof(buff[1]);
            latitude = stof(buff[2]);  //En direccion NS
            longitude = stof(buff[4]); //En direccion EW
            gps_state = stoi(buff[6]);
            sats_using = stoi(buff[7]);
        }
        else if (buff[0] == "$GNGST")
        {
            //https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_GST.html
        }
        else if (buff[0] == "$GNRMC")
        {
            //https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_RMC.html
        }
    }
}

void i2c_1()
{
    while (true)
    {
        mpu_1.read_raw();
        chrono_mpu1.end();
        chrono_mpu1.start();
    }
}

void i2c_2()
{
    while (true)
    {

        mpu_2.read_raw();
        chrono_mpu2.end();
        chrono_mpu2.start();
    }
}

void coms_handler()
{
    while (true)
    {
        communicate("supp");

    }
}

int main()
{
    //Global Timers
    chrono_global.start();

    //Initialize IMU's
    mpu_1.init();
    mpu_2.init();

    //Thread Initializer
    thread th_gps_list(gps_listener);
    thread th_i2c_1(i2c_1);
    thread th_i2c_2(i2c_2);
    thread th_communicate(coms_handler);
    usleep(100000000);
    /*while (true)
    {
        for (int i = 0; i < 10; i++)
        {
            chrono_main.start();
            usleep(10000);
            chrono_main.end();
        }
    }*/
    return 0;
}
