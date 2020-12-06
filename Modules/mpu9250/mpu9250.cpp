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
	int st1;
	int st2;
	bool ready = false;

	mpu9250(int a) : bus(a)
	{
	}

	int init()
	{
		file = open(filename, O_RDWR);
		mpu_file = open(mpu_filename, O_RDWR);
		if (ioctl(mpu_file, I2C_SLAVE, 0x68) < 0)
		{
			cout << "Couldn't initialize MPU9250" << endl;
			ready = false;
			return 0;
		}

		//MPU CONIG
		i2c_write(mpu_file, PWR_MGMT_1, 0x00);
		i2c_write(mpu_file, PWR_MGMT_1, 0x01);
		i2c_write(mpu_file, CONFIG, 0x00);
		i2c_write(mpu_file, SMPLRT_DIV, 0x00);
		i2c_write(mpu_file, GYRO_CONFIG, 0x00);	 // 250º -> /131.072
		i2c_write(mpu_file, ACCEL_CONFIG, 0x00); // 4g -> /8192.0
		i2c_write(mpu_file, ACCEL_CONFIG_2, 0x00);
		i2c_write(mpu_file, INT_PIN_CFG, 0x02);
		i2c_write(mpu_file, USER_CTRL, 0x00);

		//AK CONFIG
		if (ioctl(file, I2C_SLAVE, 0x0c) < 0)
		{
			cout << "Couldn't initialize AK8963" << endl;
			ready = false;
			return 0;
		}
		i2c_write(file, AK8963_CNTL1, 0x00);
		i2c_write(file, AK8963_CNTL1, 0x0F);

		__u8 buf[3];
		buf[0] = i2c_read(file, AK8963_ASAX);
		buf[1] = i2c_read(file, AK8963_ASAX + 1);
		buf[2] = i2c_read(file, AK8963_ASAX + 2);
		magXc = (buf[0] - 128) / 256.0 + 1.0;
		magYc = (buf[1] - 128) / 256.0 + 1.0;
		magZc = (buf[2] - 128) / 256.0 + 1.0;

		i2c_write(file, AK8963_CNTL1, 0x00);
		i2c_write(file, AK8963_CNTL1, 0x16);

		ready = true;
		return 0;
	}
	void read_raw()
	{
		if (!ready)
		{
			init();
		}
		else
		{
			read_mag();
			read_accel();
			read_gyro();
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
	void read_accel()
	{
		int16_t buf[6];
		buf[0] = i2c_read(mpu_file, 0x3B);
		buf[1] = i2c_read(mpu_file, 0x3C);
		buf[2] = i2c_read(mpu_file, 0x3D);
		buf[3] = i2c_read(mpu_file, 0x3E);
		buf[4] = i2c_read(mpu_file, 0x3F);
		buf[5] = i2c_read(mpu_file, 0x40);
		accX = dataConv(buf[1], buf[0]) / 8192.0;
		accY = dataConv(buf[3], buf[2]) / 8192.0;
		accZ = dataConv(buf[5], buf[4]) / 8192.0;
	}
	void read_gyro()
	{
		int16_t buf[6];
		buf[0] = i2c_read(mpu_file, 0x43);
		buf[1] = i2c_read(mpu_file, 0x44);
		buf[2] = i2c_read(mpu_file, 0x45);
		buf[3] = i2c_read(mpu_file, 0x46);
		buf[4] = i2c_read(mpu_file, 0x47);
		buf[5] = i2c_read(mpu_file, 0x48);
		gyroX = dataConv(buf[1], buf[0]) / 131.072;
		gyroY = dataConv(buf[3], buf[2]) / 131.072;
		gyroZ = dataConv(buf[5], buf[4]) / 131.072;
	}
};