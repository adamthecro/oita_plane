using namespace std;
using namespace chrono;
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
};

void tokenize(string const &str, const char delim,
              vector<string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}
string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, 20, "%d.%m.%y - %X", &tstruct);
    return buf;
}

float cpu_temp()
{
    string val;
    ifstream ifile("/sys/class/thermal/thermal_zone0/temp");
    ifile >> val;
    ifile.close();
    return stoi(val) / 1000;
}