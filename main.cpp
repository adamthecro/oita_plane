#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
//USLEEP
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <string>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <termios.h>
#include <curl/curl.h>
#include <thread>
#include <vector>
//#include <json/json.h>

#include "Assets/i2c.cpp"
#include "Assets/tools.cpp"
#include "Assets/comms/coms.cpp"

#include "Assets/Mahony/MahonyAHRS.cpp"

#include "Modules/pca/pca.cpp"
#include "Modules/gps/gps.cpp"
#include "Modules/mpu6050/mpu6050.cpp"
#include "Modules/mpu9250/mpu9250.cpp"

using namespace std;
using namespace chrono;
//using namespace Json;

//Global Timers
clp chrono_global;
clp chrono_main;
clp chrono_mpu1;
clp chrono_mpu2;
clp chrono_mpu3;

//Initialize IMU's
mpu6050 mpu_1(0x68);
Mahony mpu_1_filter;
mpu6050 mpu_2(0x69);
Mahony mpu_2_filter;
mpu9250 mpu_3(0x68);
Mahony mpu_3_filter;

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
int packet_id = 0;

int m0 = 0;
int m1 = 0;

int s0 = 0;
int s1 = 0;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
int s6 = 0;

float pitch_gi = 0.00;
float roll_gi = 0.00;
float yaw_gi = 0.00;

float local_time = 0.00;

float speed = 0;
float longitude = 0;
float latitude = 0;
float altitude = 0;

float time_p = 0;
int gps_state = 0;
int sats_using = 0;

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
            string dt = gps_line.substr(0, pos);
            if (dt != "")
            {
                buff[i] = gps_line.substr(0, pos);
            }
            else
            {
                buff[i] = "0.00";
            }
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
        mpu_1_filter.updateIMU(mpu_1.gyroX, mpu_1.gyroY, mpu_1.gyroZ, mpu_1.accX, mpu_1.accY, mpu_1.accZ, chrono_mpu1.get());
        chrono_mpu1.start();

        mpu_2.read_raw();
        chrono_mpu2.end();
        mpu_2_filter.updateIMU(mpu_2.gyroX, mpu_2.gyroY, mpu_2.gyroZ, mpu_2.accX, mpu_2.accY, mpu_2.accZ, chrono_mpu2.get());
        chrono_mpu2.start();
    }
}

void i2c_2()
{ /*
    while (true)
    {
        usleep()
    }*/
}

void coms_handler()
{
    while (true)
    {
        string data_to_send = "pi=" + to_string(packet_id) +
                              "&local_time=" + to_string(time_p) +
                              "&m0=" + to_string(m0) +
                              "&m1=" + to_string(m1) +
                              "&s0=" + to_string(s0) +
                              "&s1=" + to_string(s1) +
                              "&s2=" + to_string(s2) +
                              "&s3=" + to_string(s3) +
                              "&s4=" + to_string(s4) +
                              "&s5=" + to_string(s5) +
                              "&s6=" + to_string(s6) +

                              // IMU 1
                              "&accx_m1=" + to_string(mpu_1.accX) +
                              "&accy_m1=" + to_string(mpu_1.accY) +
                              "&accz_m1=" + to_string(mpu_1.accZ) +
                              "&gyrox_m1=" + to_string(mpu_1.gyroX) +
                              "&gyroy_m1=" + to_string(mpu_1.gyroY) +
                              "&gyroz_m1=" + to_string(mpu_1.gyroZ) +
                              "&pitch_m1=" + to_string(mpu_1_filter.getPitchRadians()) +
                              "&roll_m1=" + to_string(mpu_1_filter.getRollRadians()) +
                              "&yaw_m1=" + to_string(/*mpu_1_filter.getYawRadians()*/ 0) +

                              // IMU 2
                              "&accx_m2=" + to_string(mpu_2.accX) +
                              "&accy_m2=" + to_string(mpu_2.accY) +
                              "&accz_m2=" + to_string(mpu_2.accZ) +
                              "&gyrox_m2=" + to_string(mpu_2.gyroX) +
                              "&gyroy_m2=" + to_string(mpu_2.gyroY) +
                              "&gyroz_m2=" + to_string(mpu_2.gyroZ) +
                              "&pitch_m2=" + to_string(mpu_2_filter.getPitchRadians()) +
                              "&roll_m2=" + to_string(mpu_2_filter.getRollRadians()) +
                              "&yaw_m2=" + to_string(/*mpu_2_filter.getYawRadians()*/ 0) +

                              //IMU3
                              "&accx_m3=" + to_string(mpu_3.accX) +
                              "&accy_m3=" + to_string(mpu_3.accY) +
                              "&accz_m3=" + to_string(mpu_3.accZ) +
                              "&gyrox_m3=" + to_string(mpu_3.gyroX) +
                              "&gyroy_m3=" + to_string(mpu_3.gyroY) +
                              "&gyroz_m3=" + to_string(mpu_3.gyroZ) +
                              "&magx_m3=" + to_string(mpu_3.magX) +
                              "&magy_m3=" + to_string(mpu_3.magY) +
                              "&magz_m3=" + to_string(mpu_3.magZ) +
                              "&pitch_m3=" + to_string(0) +
                              "&roll_m3=" + to_string(0) +
                              "&yaw_m3=" + to_string(0) +

                              //Globals IMU
                              "&pitch_gi=" + to_string(0) +
                              "&roll_gi=" + to_string(0) +
                              "&yaw_gi=" + to_string(0) +

                              //GPS
                              "&lat=" + to_string(latitude) +
                              "&long=" + to_string(longitude) +
                              "&alt=" + to_string(altitude) +
                              "&speed=" + to_string(speed) +
                              "&sats_used=" + to_string(sats_using) +
                              "&gps_state=" + to_string(gps_state);

        chrono_main.start();
        communicate(data_to_send);
        chrono_main.end();
        cout << chrono_main.get() << endl;
        usleep(500000);
        packet_id++;
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
    while (true)
    {
        for (int i = 0; i < 10; i++)
        {
            chrono_main.start();
            usleep(1000000);
            chrono_main.end();
        }
    }
    return 0;
}
