using namespace std;

class serial
{

public:
    int ser;
    char *filename;
    speed_t baudrate;

    serial(char *a, speed_t br) : filename(a), baudrate(br)
    {
        ser = open(filename, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
        struct termios config;
        tcgetattr(ser, &config);
        cfsetispeed(&config, baudrate);
        tcsetattr(ser, TCSANOW, &config);
    }
    string read_until(string delim)
    {
        string final_data;
        while (true)
        {
            char buffer[1];
            int buff_char = read(ser, buffer, 1);
            if (buff_char > 0)
            {
                string data_ns = bufferToString(buffer, 1);
                if (data_ns == delim)
                {
                    break;
                }
                else
                {
                    final_data += data_ns;
                }
            }
            else
            {
                usleep(1000000 / 40);
            }
        }
        return final_data;
    }
};