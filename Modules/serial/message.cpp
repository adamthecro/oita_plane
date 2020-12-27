#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <cstring>
#include <algorithm>
#include <iterator>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace std;
//GGA
string utc;
float lat = 0;
string lat_dir;
float longi = 0;
string longi_dir;
int gps_qual_ind = 0;
float altitude = 0;
int fix_sats = 0;
int vis_sats = 0;
float speed = 0;

string bufferToString(char *buffer, int bufflen)
{
    string ret(buffer, bufflen);
    return ret;
}

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
int main()
{
    const char delim = ',';
    string::size_type sz;

    int serial = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_SYNC);
    struct termios config;
    tcgetattr(serial, &config);
    cfsetispeed(&config, B19200);
    tcsetattr(serial, TCSANOW, &config);
    while (true)
    {
        char buffer[1];
        string final;
        while (true)
        {
            int n = read(serial, buffer, 1);
            string data = bufferToString(buffer, 1);
            final += data;
            if (data == "\n")
            {
                break;
            }
        }
        vector<string> out;
        tokenize(final, delim, out);
        string id = out[0];
        if (id == "$GNTXT")
        {
            /* MOLESTA */
        }
        else if (id == "\n")
        {
            /* MOLESTA */
        }
        else
        {
            cout << final;
        }
    }
    return 0;
}