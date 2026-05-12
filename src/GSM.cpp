#include "GSM.h"
#include <Arduino.h>

bool GSMFunc::modemEchoOff()
{

    GSMCOMMAND("ATE0");
    LOG("ATE0");
    if (waitForResponse(ECHOMODE, 100))
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool GSMFunc::modemEchoOff(int retries)
{
    this->retries = retries;
    for (int i = 0; i <= retries; i++)
    {
        if (modemEchoOff())
        {
            return true;
            break;
        }
        else
        {
            LOG("Retrying again");
        }
        delay(1000);
    }
}
bool GSMFunc::checkModemCon()
{
    GSMCOMMAND("AT");
    LOG("AT");
    if (waitForResponse(ECHOMODE, 500))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GSMFunc::checkModemCon(int retries)
{
    this->retries = retries;
    for (int i = 0; i <= retries; i++)
    {
        if (checkModemCon())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
bool GSMFunc::checkSIM()
{
    GSMCOMMAND("AT+CPIN?");
    LOG("AT+CPIN?");
    if (waitForResponse(SIMCHECK, 500))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GSMFunc::checkSIM(int retries)
{
    this->retries = retries;
    for (int i = 0; i <= retries; i++)
    {
        if (checkSIM())
        {
            LOG("Sim detected in retry");
            return true;
        }
        else
        {
            LOG("Sim not detected Retrying");
            // return false;
        }
    }
}

bool GSMFunc::checkSignal()
{
    GSMCOMMAND("AT+QSPN");
    LOG("AT+QSPN");
    if (waitForResponse(SIGNALCHECK, 500))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GSMFunc::waitForResponse(int code, int mDelay)
{
    // LOG("waiting response");
    String dataReceived = "";

    unsigned long current_time = millis();
    while ((millis() - current_time) < mDelay)
    {
        // Serial.println("test");
        if (GSM.available() > 2)
        {
            // LOG("Got data from SERIAL");
            dataReceived = GSM.readStringUntil('\1');

            // LOG(dataReceived);
            if (dataReceived.length() > 2)
            {
                switch (code)
                {
                case ECHOMODE:
                    if (dataReceived[2] == 'O' && dataReceived[3] == 'K')
                    {

                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    break;
                case CONN:
                    if (dataReceived[2] == 'O' && dataReceived[3] == 'K')
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    break;
                case SIMCHECK:
                    if (dataReceived[9] == 'R' && dataReceived[10] == 'E' && dataReceived[11] == 'A' && dataReceived[12] == 'D' && dataReceived[13] == 'Y')
                    {
                        LOG("SIM detected");
                        return true;
                    }
                    else
                    {
                        LOG("SIM Not Detected");
                        return false;
                    }
                    break;
                case SIGNALCHECK:
                    // for (int i = 0; i < dataReceived.length(); i++)
                    // {
                    //     LOG("Position :" + String(i) + " Item : " + dataReceived[i]);
                    // }
                    if (dataReceived[2] == '+' && dataReceived[3] == 'Q' && dataReceived[4] == 'S' && dataReceived[5] == 'P' && dataReceived[6] == 'N')
                    {
                        LOG("Signal found");
                        return true;
                    }
                    else
                    {
                        LOG("No Signal found");
                        return false;
                    }
                    break;

                default:
                    break;
                }
            }
            else
            {
                LOG("No data received from the module ");
            }
        }
    }
    LOG("Ended timer While loop");
    return false;
}

// overloaded to run a plain reader
bool GSMFunc::waitForResponse()
{

    if (GSM.available() > 2)
    {
        LOG("Got data from SERIAL");
        String data = GSM.readStringUntil('\1');
        delay(1000);
        LOG(data);
    }
}
