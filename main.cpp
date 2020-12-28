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
//File Handler
#include <fstream>
//#include <json/json.h>
#include "Assets/logger/loguru.cpp"
#include "Assets/i2c.cpp"
#include "Assets/tools.cpp"
#include "Assets/coms.cpp"
#include "Assets/pid/MiniPID.cpp"

#include "Assets/Mahony/MahonyAHRS.cpp"

#include "Modules/pca.cpp"
#include "Modules/gps.cpp"
#include "Modules/mpu6050.cpp"
#include "Modules/mpu9250.cpp"

#define s_l_mid 90
#define s_r_mid 90
#define s0_mid 90
#define s1_mid 90
#define s2_mid 90
#define s3_mid 90
#define s4_mid 90

using namespace std;
using namespace chrono;

//using namespace Json;
MiniPID altitude_pid_v = MiniPID(1.5, .05, 15);
MiniPID pitch_pid_v = MiniPID(1.5, .05, 15);
MiniPID roll_pid_v = MiniPID(1.5, .05, 15);
MiniPID yaw_pid_v = MiniPID(1.5, .05, 15);

MiniPID altitude_pid_h = MiniPID(1.5, .05, 15);
MiniPID pitch_pid_h = MiniPID(1.5, .05, 15);
MiniPID roll_pid_h = MiniPID(1.5, .05, 15);
MiniPID yaw_pid_h = MiniPID(1.5, .05, 15);

fstream logz;

//Global Timers
clp chrono_global;
clp chrono_main;
clp chrono_mpu1;
clp chrono_mpu2;
clp chrono_mpu3;

//Initialize IMU's
int mpu_1_freq = 300;
int mpu_2_freq = 300;
int mpu_3_freq = 250;
bool mpu3_in_pause = false;
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

pca servo_ml(5, 70, 500, 180);
pca servo_mr(6, 70, 500, 180);

pca motor_l(7, 70, 500, 180);
pca motor_r(8, 70, 500, 180);

//Initialize GPS
serial gps("/dev/ttyS3", B19200);
const char delims = ',';

//Airplane State
string server_ip = "192.168.1.22";
float buff_recv[20] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

string summarized_data;
int mode = 0;

int packet_id = 0;

float m_l = 0;
float m_r = 0;

float s0 = 0;
float s1 = 0;
float s2 = 0;
float s3 = 0;
float s4 = 0;
float s_l = 0;
float s_r = 0;

float pitch_gi = 0;
float roll_gi = 0;
float yaw_gi = 0;

float base_pitch = 0;
float base_roll = 0;
float base_yaw = 0;

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
            cout << "Altitude?:" << buff[9] << endl;
        }
        else if (buff[0] == "$GNGST")
        {
            cout << "Altitude?:" << buff[8] << endl;
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
    float time_mpu = (1000000 / mpu_1_freq);
    while (true)
    {
        if (mpu_1.ready)
        {
            chrono_mpu1.start();
            mpu_1.read_raw();
            mpu_1_filter.updateIMU(mpu_1.gyroX, mpu_1.gyroY, mpu_1.gyroZ, mpu_1.accX, mpu_1.accY, mpu_1.accZ);
            chrono_mpu1.end();
            if (chrono_mpu1.get() < time_mpu)
            {
                usleep(time_mpu - chrono_mpu1.get());
            }
            else
            {
                LOG_F(WARNING, "MPU1: %d", 1000000 / chrono_mpu1.get());
            }
        }
        else
        {
            mpu_1.read_raw();
        }
    }
}

void i2c_2()
{
    float time_mpu = (1000000 / mpu_2_freq);
    while (true)
    {
        if (mpu_2.ready)
        {
            chrono_mpu2.start();
            mpu_2.read_raw();
            mpu_2_filter.updateIMU(mpu_2.gyroX, mpu_2.gyroY, mpu_2.gyroZ, mpu_2.accX, mpu_2.accY, mpu_2.accZ);
            chrono_mpu2.end();
            if (chrono_mpu2.get() < time_mpu)
            {
                usleep(time_mpu - chrono_mpu2.get());
            }
            else
            {
                LOG_F(WARNING, "MPU2: %d", 1000000 / chrono_mpu2.get());
            }
        }
        else
        {
            mpu_2.read_raw();
        }
    }
}

void i2c_3()
{
    float time_mpu = (1000000 / mpu_3_freq);
    while (true)
    {
        if (mpu_3.ready)
        {
            chrono_mpu3.start();
            mpu_3.read_raw();
            mpu_3_filter.update(mpu_3.gyroX, mpu_3.gyroY, mpu_3.gyroZ, mpu_3.accX, mpu_3.accY, mpu_3.accZ, mpu_3.magY, mpu_3.magX, -mpu_3.magZ);

            servo_ml.rotate_deg(s_l);
            servo_mr.rotate_deg(s_r);
            motor_l.set_pwm(0, m_l);
            motor_r.set_pwm(0, m_r);

            chrono_mpu3.end();

            if (chrono_mpu3.get() < time_mpu)
            {
                mpu3_in_pause = true;
                usleep(time_mpu - chrono_mpu3.get());
                mpu3_in_pause = false;
                LOG_F(WARNING, "hmmmmmmmmmmmMM: %d", chrono_mpu3.get());
            }
            else
            {
                LOG_F(WARNING, "MPU3: %d", chrono_mpu3.get());
            }
        }
        else
        {
            mpu_3.read_raw();
        }
    }
}

string summarize_data()
{
    return to_string(packet_id) + ";" +  //0
           to_string(time_p) + ";" +     //1
           to_string(latitude) + ";" +   //2
           to_string(longitude) + ";" +  //3
           to_string(altitude) + ";" +   //4
           to_string(speed) + ";" +      //5
           to_string(sats_using) + ";" + //6
           to_string(gps_state) + ";" +  //7

           to_string(m_l) + ";" + //8
           to_string(m_r) + ";" + //9
           to_string(s0) + ";" +  //10
           to_string(s1) + ";" +  //11
           to_string(s2) + ";" +  //12
           to_string(s3) + ";" +  //13
           to_string(s4) + ";" +  //14
           to_string(s_l) + ";" + //15
           to_string(s_r) + ";" + //16

           to_string(mpu_1.accX) + ";" +                     //17
           to_string(mpu_1.accY) + ";" +                     //18
           to_string(mpu_1.accZ) + ";" +                     //19
           to_string(mpu_1.gyroX) + ";" +                    //20
           to_string(mpu_1.gyroY) + ";" +                    //21
           to_string(mpu_1.gyroZ) + ";" +                    //22
           to_string(mpu_1_filter.getPitchRadians()) + ";" + //23
           to_string(mpu_1_filter.getRollRadians()) + ";" +  //24
           to_string(mpu_1_filter.getYawRadians()) + ";" +   //25

           to_string(mpu_2.accX) + ";" +                     //26
           to_string(mpu_2.accY) + ";" +                     //27
           to_string(mpu_2.accZ) + ";" +                     //28
           to_string(mpu_2.gyroX) + ";" +                    //29
           to_string(mpu_2.gyroY) + ";" +                    //30
           to_string(mpu_2.gyroZ) + ";" +                    //31
           to_string(mpu_2_filter.getPitchRadians()) + ";" + //32
           to_string(mpu_2_filter.getRollRadians()) + ";" +  //33
           to_string(mpu_2_filter.getYawRadians()) + ";" +   //34

           to_string(mpu_3.accX) + ";" +                     //35
           to_string(mpu_3.accY) + ";" +                     //36
           to_string(mpu_3.accZ) + ";" +                     //37
           to_string(mpu_3.gyroX) + ";" +                    //38
           to_string(mpu_3.gyroY) + ";" +                    //39
           to_string(mpu_3.gyroZ) + ";" +                    //40
           to_string(mpu_3.magX) + ";" +                     //41
           to_string(mpu_3.magY) + ";" +                     //42
           to_string(mpu_3.magZ) + ";" +                     //43
           to_string(mpu_3_filter.getPitchRadians()) + ";" + //44
           to_string(mpu_3_filter.getRollRadians()) + ";" +  //45
           to_string(mpu_3_filter.getYawRadians()) + ";" +   //46

           to_string(pitch_gi) + ";" + //47
           to_string(roll_gi) + ";" +  //48
           to_string(yaw_gi);          //49
}
void coms_handler()
{
    while (true)
    {
        string data_to_send = server_ip + ":3000/plane?" + summarized_data;
        chrono_main.start();
        string rec_data = communicate(data_to_send);
        chrono_main.end();
        size_t pos = 0;
        int i = 0;
        string token;
        while ((pos = rec_data.find(delims)) != string::npos)
        {
            string dt = rec_data.substr(0, pos);
            if (dt != "")
            {
                buff_recv[i] = stof(rec_data.substr(0, pos));
            }
            else
            {
                buff_recv[i] = 0;
            }
            rec_data.erase(0, pos + 1);
            i++;
        }
        if (chrono_main.get() < 100000 && mode == 4)
        {
            usleep(100000 - chrono_main.get());
        }
        else
        {
            usleep(10000);
        }
        packet_id++;
    }
}

int main()
{
    string file_name = "logs/" + currentDateTime();
    loguru::add_file((file_name + ".log").c_str(), loguru::Append, loguru::Verbosity_MAX);
    LOG_F(INFO, "Initializing...");

    logz.open(file_name + "-data.csv", ios::out | ios::app);

    //Global Timers
    chrono_global.start();

    //Start PCA
    pca_init();

    //Initialize IMU's
    mpu_1.init();
    mpu_2.init();
    mpu_3.init();
    mpu_1_filter.begin(mpu_1_freq);
    mpu_2_filter.begin(mpu_2_freq);
    mpu_3_filter.begin(mpu_3_freq);

    //PID Controllers
    altitude_pid_v.setOutputLimits(500, 2000);
    pitch_pid_v.setOutputLimits(-90, 90);
    roll_pid_v.setOutputLimits(-500, 500);
    yaw_pid_v.setOutputLimits(-30, 30);

    altitude_pid_h.setOutputLimits(-20, 20);
    pitch_pid_h.setOutputLimits(-90, 90);
    roll_pid_h.setOutputLimits(0, 500);
    yaw_pid_h.setOutputLimits(0, 500);

    //Thread Initializer
    /*thread th_gps_list(gps_listener);
    thread th_i2c_1(i2c_1);
    thread th_i2c_2(i2c_2);
    thread th_i2c_3(i2c_3);*/
    thread th_communicate(coms_handler);
    while (true)
    {
        pitch_gi = (mpu_1_filter.getPitchRadians() + mpu_2_filter.getPitchRadians() + mpu_3_filter.getPitchRadians()) / 3;
        roll_gi = (mpu_1_filter.getRollRadians() + mpu_2_filter.getRollRadians() + mpu_3_filter.getRollRadians()) / 3;
        yaw_gi = mpu_3_filter.getYawRadians();
        summarized_data = summarize_data();
        logz << summarized_data << endl;
        if (mode != buff_recv[0])
        {
            if (buff_recv[0] == 4)
            {
                base_yaw = yaw_gi;
            }
            mode = buff_recv[0];
        }
        switch (mode)
        {
        case 0:
            m_l = 0;
            m_r = 0;
            s_l = 0;
            s_r = 0;
            s0 = 0;
            s1 = 0;
            s2 = 0;
            s3 = 0;
            s4 = 0;
            usleep(10000);
            break;

        case 1:
            for (int i = 45; i < 135; i++)
            {
                s0++;
                s1++;
                s2++;
                s3++;
                s4++;

                s_l++;
                s_r++;
                usleep(1000000 / 90);
            }
            for (int i = 180; i == 0; i--)
            {
                s0--;
                s1--;
                s2--;
                s3--;
                s4--;

                s_l--;
                s_r--;
                usleep(1000000 / 50);
            }
            break;

        case 2:
            m_l = 0;
            m_r = 0;
            s_l = s_l_mid + pitch_gi * 57.29578;
            s_r = s_r_mid + pitch_gi * 57.29578;
            s0 = s0_mid;
            s1 = s1_mid;
            s2 = s2_mid;
            s3 = s3_mid;
            s4 = s4_mid;
            break;

        case 3:
            m_l = buff_recv[1];
            m_r = buff_recv[2];
            s_l = buff_recv[8];
            s_r = buff_recv[9];
            s0 = buff_recv[3];
            s1 = buff_recv[4];
            s2 = buff_recv[5];
            s3 = buff_recv[6];
            s4 = buff_recv[7];
            break;

        case 4:
            base_yaw += buff_recv[8];
            float roll_out_pid = roll_pid_v.getOutput(roll_gi, base_roll);
            float yaw_out_pid = yaw_pid_v.getOutput(yaw_gi, base_yaw);
            m_l = buff_recv[1] + roll_out_pid;
            m_r = buff_recv[2] - roll_out_pid;
            s_l = buff_recv[8] + yaw_out_pid;
            s_r = buff_recv[9] - yaw_out_pid;

            break;

        case 99:
            system("reboot");
            break;

        default:
            break;
        }
        usleep(1000000 / 250);
    }
    return 0;
}