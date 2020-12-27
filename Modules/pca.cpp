using namespace std;
#define MODE1 0x00
#define MODE2 0x01
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4
#define PRE_SCALE 0xFE
#define MAX_PWM_RES 4096
#define CLOCK_FREQ 27000000.0

char *pca_filename = "/dev/i2c-1";
int pca_file;
int pca_ready = false;

void set_pwm_freq(int freq)
{
    uint8_t prescale = (CLOCK_FREQ / MAX_PWM_RES / freq) + 0.5;
    uint8_t oldmode = i2c_read(pca_file, MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; //sleep
    i2c_write(pca_file, MODE1, newmode);       // go to sleep
    i2c_write(pca_file, PRE_SCALE, prescale);
    i2c_write(pca_file, MODE1, oldmode);
    usleep(100000);
    i2c_write(pca_file, MODE1, oldmode | 0xA1);
}

void pca_reset()
{
    i2c_write(pca_file, MODE2, 0x04);
    i2c_write(pca_file, MODE1, 0x01);
    usleep(100000);
    i2c_write(pca_file, MODE1, 0x00);
}

int pca_init()
{
    pca_file = open(pca_filename, O_RDWR);
    if (ioctl(pca_file, I2C_SLAVE, 0x40) < 0)
    {
        LOG_F(ERROR, "Couldn't initialize PCA9685");
    }

    pca_reset();
    usleep(100000);
    set_pwm_freq(300);
    LOG_F(INFO, "PCA9685 READY!");
    pca_ready = true;
    return 0;
}

class pca
{

public:
    int min_pwm;
    int max_pwm;
    int max_deg;
    int pca_bus;

    pca(int a, int b, int c, int d) : pca_bus(a), min_pwm(b), max_pwm(c), max_deg(d)
    {
    }

    void set_pwm(int on_value, int off_value)
    { /*
        i2c_write(pca_file, LED0_ON_L + LED_MULTIPLYER * pca_bus, on_value & 0xFF);
        i2c_write(pca_file, LED0_ON_H + LED_MULTIPLYER * pca_bus, on_value >> 8);
        i2c_write(pca_file, LED0_OFF_L + LED_MULTIPLYER * pca_bus, off_value & 0xFF);
        i2c_write(pca_file, LED0_OFF_H + LED_MULTIPLYER * pca_bus, off_value >> 8); */
        unsigned char buf[5];
        buf[0] = LED0_ON_L + LED_MULTIPLYER * pca_bus;
        buf[1] = on_value & 0xFF;
        buf[2] = on_value >> 8;
        buf[3] = off_value & 0xFF;
        buf[4] = off_value >> 8;
        if (write(pca_file, buf, 5) != 5)
        {
            LOG_F(ERROR, "Couldn't write to PCA9685 at: %d", pca_bus);
        }
    }
    void rotate_deg(double deg)
    {
        if (pca_ready)
        {
            if (deg < 0)
            {
                set_pwm(0, min_pwm);
            }
            else if (deg > max_deg)
            {
                set_pwm(0, max_pwm);
            }
            else
            {
                int calc_pwm = min_pwm + deg * (max_pwm - min_pwm) / max_deg;
                set_pwm(0, calc_pwm);
            }
        }
    }
};