#define GSM Serial1
#define GSMCOMMAND GSM.println
#define LOG Serial.println

// RESPONSE CODES
#define ECHOMODE 101
#define CONN 102
#define SIMCHECK 103
#define SIGNALCHECK 104
#define RSSICHECK 105
class GSMFunc
{
public:
    // Variables
    int retries;
    int period = 1000;
    unsigned long time_now = 0;
    // GSM based HW commands
    bool Initialize();

    bool checkModemCon();
    bool checkModemCon(int retries);

    bool modemEchoOff();
    bool modemEchoOff(int retries);

    bool checkSIM();
    bool checkSIM(int retries);

    bool checkSignal();

    float getsignalStrength();
    bool waitForResponse(int code, int mDelay);
    bool waitForResponse();

private:
protected:
};