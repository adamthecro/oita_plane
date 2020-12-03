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