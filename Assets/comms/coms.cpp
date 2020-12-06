using namespace std;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
}
string communicate(string data)
{
    CURL *curl;
    CURLcode res;
    string readBuffer;
    string url = string("https://omsintheairserver.herokuapp.com/plane?") + data;
    curl = curl_easy_init();
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);

        cout << "Received: " << readBuffer << endl;
    }
    return readBuffer;
}