#include <iostream>
//Usleep
#include <unistd.h>
//Strings
#include <string>
//Serial
#include <termios.h>
//HTTP Requests
#include <curl/curl.h>
//Multithreading
#include <thread>
//stdout mes bunic
#include <iomanip>
//StringStream
#include <sstream>
//File Handler
#include <fstream>

//i2c
extern "C"
{
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
}
#include <sys/ioctl.h>
#include <fcntl.h>

//Own libraries
#include "Assets/tools.cpp"
#include "Assets/i2c.cpp"
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

#ifdef DEBUG
bool debuging = true;
#else
bool debuging = false;
#endif

//Loggers
string file_name = "logs/" + currentDateTime();
logs data_log(file_name + "-data.csv");
logs logs_log(file_name + "-logs.log");

MiniPID altitude_pid_v = MiniPID(1.5, .05, 15);
MiniPID pitch_pid_v = MiniPID(1.5, .05, 15);
MiniPID roll_pid_v = MiniPID(1.5, .05, 15);
MiniPID yaw_pid_v = MiniPID(1.5, .05, 15);

MiniPID altitude_pid_h = MiniPID(1.5, .05, 15);
MiniPID pitch_pid_h = MiniPID(1.5, .05, 15);
MiniPID roll_pid_h = MiniPID(1.5, .05, 15);
MiniPID yaw_pid_h = MiniPID(1.5, .05, 15);

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
string server_ip = "192.168.1.22:3000";
float buff_recv[20] = {0};

string summarized_data;
int mode = 0;
bool mode_1_phase = true;

int packet_id = 0;

float m_l, m_r = 0;
float s0, s1, s2, s3, s4, s_l, s_r = 0;

float pitch_gi, roll_gi, yaw_gi = 0;

float base_pitch, base_roll, base_yaw = 0;

float local_time, speed, longitude, latitude, altitude = 0.00;

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
            latitude = ddm_to_dd(stof(buff[2]));  //En direccion NS
            longitude = ddm_to_dd(stof(buff[4])); //En direccion EW
            gps_state = stoi(buff[6]);
            sats_using = stoi(buff[7]);
        }
        else if (buff[0] == "$GNGST" || "$GPGST" || "$GLGST")
        {
            //https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_GST.html
            cout << "Height error?:" << buff[8] << endl;
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
                logs_log.log(2, "main", "MPU1: " + to_string(1000000 / chrono_mpu1.get()));
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
                logs_log.log(2, "main", "MPU2: " + to_string(1000000 / chrono_mpu2.get()));
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
            chrono_mpu3.end();
            if (chrono_mpu3.get() < time_mpu)
            {
                usleep(time_mpu - chrono_mpu3.get());
            }
            else
            {
                logs_log.log(2, "main", "MPU3: " + to_string(chrono_mpu3.get()));
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
    stringstream ss;

    ss << packet_id << ","  //0
       << time_p << ","     //1
       << latitude << ","   //2
       << longitude << ","  //3
       << altitude << ","   //4
       << speed << ","      //5
       << sats_using << "," //6
       << gps_state << ","  //7

       << m_l << "," //8
       << m_r << "," //9
       << s0 << ","  //10
       << s1 << ","  //11
       << s2 << ","  //12
       << s3 << ","  //13
       << s4 << ","  //14
       << s_l << "," //15
       << s_r << "," //16

       << mpu_1.accX << ","                     //17
       << mpu_1.accY << ","                     //18
       << mpu_1.accZ << ","                     //19
       << mpu_1.gyroX << ","                    //20
       << mpu_1.gyroY << ","                    //21
       << mpu_1.gyroZ << ","                    //22
       << mpu_1_filter.getPitchRadians() << "," //23
       << mpu_1_filter.getRollRadians() << ","  //24
       << mpu_1_filter.getYawRadians() << ","   //25

       << mpu_2.accX << ","                     //26
       << mpu_2.accY << ","                     //27
       << mpu_2.accZ << ","                     //28
       << mpu_2.gyroX << ","                    //29
       << mpu_2.gyroY << ","                    //30
       << mpu_2.gyroZ << ","                    //31º
       << mpu_2_filter.getPitchRadians() << "," //32
       << mpu_2_filter.getRollRadians() << ","  //33
       << mpu_2_filter.getYawRadians() << ","   //34

       << mpu_3.accX << ","                     //35
       << mpu_3.accY << ","                     //36
       << mpu_3.accZ << ","                     //37
       << mpu_3.gyroX << ","                    //38
       << mpu_3.gyroY << ","                    //39
       << mpu_3.gyroZ << ","                    //40
       << mpu_3.magX << ","                     //41
       << mpu_3.magY << ","                     //42
       << mpu_3.magZ << ","                     //43
       << mpu_3_filter.getPitchRadians() << "," //44
       << mpu_3_filter.getRollRadians() << ","  //45
       << mpu_3_filter.getYawRadians() << ","   //46

       << pitch_gi << "," //47
       << roll_gi << ","  //48
       << yaw_gi << ","   //49
       << cpu_temp();     //50

    string s;
    ss >> s;
    return s;
}

void coms_handler()
{
    while (true)
    {
        string data_to_send = server_ip + "/plane?" + summarized_data;
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
    chrono_global.start();
    logs_log.log(1, "main", "'Initializing'");
    data_log.raw("packet_id,time_p,latitude,longitude,altitude,speed,sats_using,gps_state,m_l,m_r,s0,s1,s2,s3,s4,s_l,s_r,mpu1_ax,mpu1_ay,mpu1_az,mpu1_gx,mpu1_gy,mpu1_gz,mpu1_p,mpu1_r,mpu1_y,mpu2_ax,mpu2_ay,mpu2_az,mpu2_gx,mpu2_gy,mpu2_gz,mpu2_p,mpu2_r,mpu2_y,mpu3_ax,mpu3_ay,mpu3_az,mpu3_gx,mpu3_gy,mpu3_gz,mpu3_mx,mpu3_my,mpu3_mz,mpu3_p,mpu3_r,mpu3_y,pitch_gi,roll_gi,yaw_gi,cpu_temp");

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
    thread th_gps_list(gps_listener);
    thread th_i2c_1(i2c_1);
    thread th_i2c_2(i2c_2);
    thread th_i2c_3(i2c_3);
    thread th_communicate(coms_handler);
    while (true)
    {
        pitch_gi = (mpu_1_filter.getPitchRadians() + mpu_2_filter.getPitchRadians() + mpu_3_filter.getPitchRadians()) / 3;
        roll_gi = (mpu_1_filter.getRollRadians() + mpu_2_filter.getRollRadians() + mpu_3_filter.getRollRadians()) / 3;
        yaw_gi = mpu_3_filter.getYawRadians();
        summarized_data = summarize_data();
        data_log.raw(summarized_data);

        if (mode != buff_recv[0])
        {
            if (buff_recv[0] == 4)
            {
                base_yaw += yaw_gi;
            }
            mode = buff_recv[0];
        }
        switch (mode)
        {
        case 0:
        {
            m_l, m_r, m_r, s_l, s_r, s0, s1, s2, s3, s4 = 0;
            usleep(10000);
            break;
        }

        case 1:
        {
            if (s0 >= 135)
            {
                mode_1_phase = false;
            }
            if (s0 <= 45)
            {
                mode_1_phase = true;
            }
            if (mode_1_phase)
            {
                s0++;
                s1++;
                s2++;
                s3++;
                s4++;

                s_l++;
                s_r++;
            }
            else
            {

                --s0;
                --s1;
                --s2;
                --s3;
                --s4;

                --s_l;
                --s_r;
            }
            usleep(1000000 / 25);
            break;
        }

        case 2:
        {
            m_l, m_r = 0;
            s_l = s_l_mid + pitch_gi * 57.29578;
            s_r = s_r_mid + pitch_gi * 57.29578;
            s0 = s0_mid;
            s1 = s1_mid;
            s2 = s2_mid;
            s3 = s3_mid;
            s4 = s4_mid;
            usleep(1000000 / 500);
            break;
        }

        case 3:
        {
            m_l = buff_recv[1];
            m_r = buff_recv[2];
            s_l = buff_recv[8];
            s_r = buff_recv[9];
            s0 = buff_recv[3];
            s1 = buff_recv[4];
            s2 = buff_recv[5];
            s3 = buff_recv[6];
            s4 = buff_recv[7];
            usleep(1000000 / 500);
            break;
        }

        case 4:
        {
            base_yaw += buff_recv[8];
            float roll_out_pid = roll_pid_v.getOutput(roll_gi, base_roll);
            float yaw_out_pid = yaw_pid_v.getOutput(yaw_gi, base_yaw);
            m_l = buff_recv[1] + roll_out_pid;
            m_r = buff_recv[2] - roll_out_pid;
            s_l = s_l_mid + pitch_gi * 57.29578;
            s_r = s_r_mid + pitch_gi * 57.29578;
            servo_ml.rotate_deg(s_l);
            servo_mr.rotate_deg(s_r);
            motor_l.set_pwm(0, m_l);
            motor_r.set_pwm(0, m_r);
            usleep(1000000 / 500);
            break;
        }

        case 99:
        {
            system("reboot");
            break;
        }

        default:
        {
            break;
        }
        }
        //Uk
    }
    return 0;
}