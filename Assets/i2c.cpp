using namespace std;

extern logs logs_log;

char false_char = char(32);
int16_t dataConv(int16_t data1, int16_t data2)
{
    int16_t value = data1 | (data2 << 8);
    if (value & (1 << 16 - 1))
    {
        value -= (1 << 16);
    }
    return value;
}
bool i2c_write(int privfile, __u8 reg_address, __u8 val)
{
    char buf[2];
    buf[0] = reg_address;
    buf[1] = val;
    if (write(privfile, buf, 2) != 2)
    {
        logs_log.log(0, "i2c_tools", "Unable to write to: " + to_string(reg_address));
        return false;
    }
    return true;
}

char i2c_read(int privfile, uint8_t reg_address)
{
    char buf[1];
    buf[0] = reg_address;
    if (read(privfile, buf, 1) != 1)
    {
        return false_char;
    }
    return buf[0];
}

int16_t i2c_readword(int privfile, uint8_t MSB, uint8_t LSB)
{

    uint8_t msb = i2c_read(privfile, MSB);
    uint8_t lsb = i2c_read(privfile, LSB);
    return ((int16_t)msb << 8) + lsb;
}

string Convert(float number)
{
    ostringstream buff;
    buff << number;
    return buff.str();
}