using namespace std;
#define PWR_MGMT_1 0x6B
#define USER_CTRL 0x6A
#define AK8963_ST1 0x02
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define INT_PIN_CFG 0x37
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

extern logs logs_log;

class mpu9250
{

public:
	int bus;
	int file;
	int mpu_file;
	char *filename = "/dev/i2c-1";
	char *mpu_filename = "/dev/i2c-1";
	float mres = 4912.0 / 32760.0;
	float magXc;
	float magYc;
	float magZc;
	float magX;
	float magY;
	float magZ;
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float gyroXoff;
	float gyroYoff;
	float gyroZoff;
	int addr;
	int st1;
	int st2;
	bool ready = false;

	mpu9250(int a) : addr(a)
	{
	}

	bool init()
	{
		file = open(filename, O_RDWR);
		mpu_file = open(mpu_filename, O_RDWR);
		if (ioctl(mpu_file, I2C_SLAVE, addr) < 0)
		{
			logs_log.log(3, "MPU9250", "Couldn't initialize MPU9250");
			ready = false;
			return false;
		}

		//MPU CONIG
		i2c_write(mpu_file, PWR_MGMT_1, 0x00);
		i2c_write(mpu_file, PWR_MGMT_1, 0x01);
		i2c_write(mpu_file, CONFIG, 0x00);
		i2c_write(mpu_file, SMPLRT_DIV, 0x00);
		i2c_write(mpu_file, GYRO_CONFIG, 0x00);	 // 250º -> /131.072
		i2c_write(mpu_file, ACCEL_CONFIG, 0x08); // 4g -> /8192.0
		i2c_write(mpu_file, ACCEL_CONFIG_2, 0x00);
		i2c_write(mpu_file, INT_PIN_CFG, 0x02);
		i2c_write(mpu_file, USER_CTRL, 0x00);

		//AK CONFIG
		if (ioctl(file, I2C_SLAVE, 0x0c) < 0)
		{
			logs_log.log(3, "MPU9250", "Couldn't initialize AK8963");
			ready = false;
			return false;
		}
		i2c_write(file, AK8963_CNTL1, 0x00);
		i2c_write(file, AK8963_CNTL1, 0x0F);
		unsigned char buf[5];
		buf[0] = 0x10;
		if ((write(file, buf, 1)) != 1)
		{
			logs_log.log(3, "MPU9250", "Couldn't write to AK8963");
		}
		if (read(file, buf, 3) != 3)
		{
			logs_log.log(3, "MPU9250", "Couldn't read from AK8963");
		}
		magXc = (buf[0] - 128) / 256.0 + 1.0;
		magYc = (buf[1] - 128) / 256.0 + 1.0;
		magZc = (buf[2] - 128) / 256.0 + 1.0;

		i2c_write(file, AK8963_CNTL1, 0x00);
		i2c_write(file, AK8963_CNTL1, 0x16);

		ready = true;
		return true;
	}
	void read_raw()
	{
		if (!ready)
		{
			if (!init())
			{

				usleep(100000);
			}
		}
		else
		{
			short x, y, z;
			unsigned char buf[15];
			buf[0] = 0x3B;

			if ((write(mpu_file, buf, 1)) != 1)
			{
				logs_log.log(3, "MPU9250", "Couldn't write to MPU9250");
			}
			if (read(mpu_file, buf, 14) != 14)
			{
				logs_log.log(3, "MPU9250", "Couldn't read from MPU9250");
			}
			x = ((int16_t)buf[0] << 8) + buf[1];
			y = ((int16_t)buf[2] << 8) + buf[3];
			z = ((int16_t)buf[4] << 8) + buf[5];
			accX = x / 8192.0;
			accY = y / 8192.0;
			accZ = z / 8192.0;
			x = ((int16_t)buf[8] << 8) + buf[9];
			y = ((int16_t)buf[10] << 8) + buf[11];
			z = ((int16_t)buf[12] << 8) + buf[13];
			gyroX = x / 131.072;
			gyroY = y / 131.072;
			gyroZ = z / 131.072;
			read_mag();
		}
	}
	void read_mag()
	{
		st1 = i2c_read(file, AK8963_ST1);
		if (st1 & 0x01)
		{
			int16_t buf[7];
			buf[0] = i2c_read(file, 0x03);
			buf[1] = i2c_read(file, 0x04);
			buf[2] = i2c_read(file, 0x05);
			buf[3] = i2c_read(file, 0x06);
			buf[4] = i2c_read(file, 0x07);
			buf[5] = i2c_read(file, 0x08);
			buf[6] = i2c_read(file, 0x09);
			st2 = buf[6];
			if (buf[6] == 16)
			{
				magX = dataConv(buf[0], buf[1]);
				magY = dataConv(buf[2], buf[3]);
				magZ = dataConv(buf[4], buf[5]);

				magX = magX * mres * magXc;
				magY = magY * mres * magYc;
				magZ = magZ * mres * magZc;
			}
		}
	}
};