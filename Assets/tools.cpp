using namespace std;
using namespace chrono;

#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"

extern bool debuging;

class clp
{
public:
    system_clock::time_point st;
    system_clock::time_point en;

    clp(){};
    void start()
    {
        st = system_clock::now();
    };
    void end()
    {
        en = system_clock::now();
    };
    int get()
    {
        return duration_cast<std::chrono::microseconds>(en - st).count();
    };
    int get_now()
    {
        return duration_cast<std::chrono::microseconds>(system_clock::now() - st).count();
    };
};

string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, 20, "%d-%m-%y %X", &tstruct);
    return buf;
}

float cpu_temp()
{
    string val;
    ifstream ifile("/sys/class/thermal/thermal_zone0/temp");
    if (!ifile.fail())
    {

        ifile >> val;
        ifile.close();
        return stoi(val) / 1000;
    }
    else
    {
        return 0;
    }
}

float ddm_to_dd(float coords)
{
    int inputDegrees = int(coords / 100);
    float inputMinutes = coords - inputDegrees * 100;
    float fcoords = inputDegrees + (inputMinutes / 60);
    return fcoords;
}

float round_uk(float var)
{
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

string bufferToString(char *buffer, int bufflen)
{
    string ret(buffer, bufflen);
    return ret;
};

extern clp chrono_global;

class logs
{
public:
    fstream file;
    logs(string file_name)
    {
        file.open(file_name, ios::out | ios::app);
    };

    void raw(string data)
    {
        file << data << endl;
    }

    void log(int verbosity, string func_name, string data)
    {
        string log_string;
        stringstream global;
        global << setfill('-') << setw(10) << chrono_global.get_now();
        string s_global;
        global >> s_global;

        stringstream f_name;
        f_name << setfill('-') << setw(10) << func_name;
        string s_name;
        f_name >> s_name;

        file << currentDateTime() << " [" + s_global + "] - " + s_name + " ||\t" << data << endl;

        if (debuging)
        {
            string print_string;
            switch (verbosity)
            {
            case 0:
            {
                print_string = "\e[1;0m" + currentDateTime() + " [" + s_global + "] - " + s_name + " ||\t";
                break;
            }

            case 1:
            {
                print_string = "\e[1;32m" + currentDateTime() + " [" + s_global + "] - " + s_name + " ||\t";
                break;
            }

            case 2:
            {
                print_string = "\e[1;33m" + currentDateTime() + " [" + s_global + "] - " + s_name + " ||\t";
                break;
            }

            case 3:
            {
                print_string = "\e[1;31m" + currentDateTime() + " [" + s_global + "] - " + s_name + " ||\t";
                break;
            }

            case 4:
            {
                print_string = "\e[1;34m" + currentDateTime() + " [" + s_global + "] - " + s_name + " ||\t";
                break;
            }

            default:
            {
                break;
            }
            }
            print_string += "\e[0;0m" + data + "\n";
            printf(print_string.c_str());
        }
    }
};
