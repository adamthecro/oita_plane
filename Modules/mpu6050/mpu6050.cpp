using namespace std;

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_CONFIG 0x1A

class mpu6050
{
public:
    int bus;
    int addr;
    int file;
    char *filename = "/dev/i2c-0";
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
    bool ready = false;

    mpu6050(int a) : addr(a)
    {
    }

    int init()
    {
        file = open(filename, O_RDWR);
        file = open(filename, O_RDWR);
        if (ioctl(file, I2C_SLAVE, addr) < 0)
        {
            cout << "Couldnt initialize MPU6050 at: " << addr;
            ready = false;
            return 0;
        }
        i2c_write(file, MPU6050_RA_PWR_MGMT_1, 0x00);
        i2c_write(file, MPU6050_RA_PWR_MGMT_1, 0x01);
        i2c_write(file, MPU6050_RA_SMPLRT_DIV, 0x00);
        i2c_write(file, MPU6050_RA_GYRO_CONFIG, 0x00);  // 250º -> /131.072
        i2c_write(file, MPU6050_RA_ACCEL_CONFIG, 0x08); // 4g -> /8192.0
        i2c_write(file, MPU6050_RA_CONFIG, 0x00);
        return 0;
    }
    void read_raw()
    {
        short x, y, z;
        unsigned char buf[14];
        buf[0] = 0x3B;

        if ((write(file, buf, 1)) != 1)
        {
            cout << "Error writing to MPU6050 at: " << addr;
        }
        if (read(file, buf, 14) != 14)
        {
            cout << "Error reading from MPU6050 at: " << addr;
        }
        x = ((int16_t)buf[0] << 8) + buf[1];
        y = ((int16_t)buf[2] << 8) + buf[3];
        z = ((int16_t)buf[4] << 8) + buf[5];
        accZ = z / 8192.0;
        accX = x / 8192.0;
        accY = y / 8192.0;
        temp = (((int16_t)buf[6] << 8) + buf[7]) / 340.0 + 36.53;
        x = ((int16_t)buf[8] << 8) + buf[9];
        y = ((int16_t)buf[10] << 8) + buf[11];
        z = ((int16_t)buf[12] << 8) + buf[13];
        gyroZ = z / 131.072;
        gyroX = x / 131.072;
        gyroY = y / 131.072;
        //read_accel();
        //read_gyro();
    }
};