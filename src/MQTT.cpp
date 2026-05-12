#include <MQTT.h>

void MQTT::setMQTTHOST(String hostname, unsigned int port)
{
    _hostname = hostname;
    _port = port;
    LOG("HOST set");
}

void MQTT::setMQTTCred(String username, String password)
{
    _username = username;
    _password = password;
}

void MQTT::setMqttSub(String mqttsub)
{
    _mqttSub = mqttsub;
}

bool MQTT::connectMQTT()
{
    LOG("AT+QMTOPEN=0,\"");
    GSMCOMMANDL("AT+QMTOPEN=0,\"");
    GSMCOMMANDL(_hostname);
    GSMCOMMANDL("\",");
    GSMCOMMAND(_port);
    if (waitForResponse(MQTTPORT, 1000))
    {
        if (establishConnection())
        {
            return true;
        }
        else
        {

            LOG("Error in connection entering retry method");

            if (establishConnection(5))
            {
                return true;
            }
            else
            {
                // Need to write a code to restart the Quectel module  - Pending
                // return false;
                LOG("Fallback");
                return false;
            }
        }
    }
    else
    {
        // retry policy
        LOG("Error in connection at first phase");
    }
    return true;
}

bool MQTT::establishConnection()
{
    GSMCOMMAND("AT+QMTCONN=0,\"Device97\",\"" + _username + "\",\"" + _password + "\"");
    LOG("AT+QMTCONN=0,\"3\",\"" + _username + "\",\"" + _password + "\"");
    if (waitForResponse(MQTTCONN, 1000))
    {
        LOG("Successfully connected");
        return true;
    }
    else
    {
        LOG("Connection failed");
        return false;
    }
}
bool MQTT::establishConnection(int retries)
{
    this->retries = retries;
    for (int i = 0; i <= retries; i++)
    {
        GSMCOMMAND("AT+QMTCONN=0,\"USdevicewewe2\",\"" + _username + "\",\"" + _password + "\"");
        LOG("AT+QMTCONN=0,\"3\",\"" + _username + "\",\"" + _password + "\"");
        if (waitForResponse(MQTTCONN, 1000))
        {
            LOG("Successfully connected");
            return true;
        }
        else
        {
            LOG("Connection failed");
            // return false;
        }
    }
    return false;
}

// The following functions subscribes the Mqtt topic

bool MQTT::mqttSub()
{
    LOG("AT+QMTSUB=0,1,\"");
    LOG(_mqttSub);
    LOG("\",0");
    GSMCOMMANDL("AT+QMTSUB=0,1,\"");
    GSMCOMMANDL(_mqttSub);
    GSMCOMMAND("\",0");
    if (waitForResponse(MQTTSUB, 2000))
    {
        LOG("Subscription success");
    }
    else
    {
        LOG("Failed to subscribe");
    }
}

bool MQTT::mqttPub(String mqttMsg, String topic)
{
    this->mqttMsg = mqttMsg;
    this->topic = topic;

    LOG("AT+QMTPUBEX=0,0,0,0,\"" + topic + "\"," + mqttMsg.length());
    GSMCOMMAND("AT+QMTPUBEX=0,0,0,0,\"" + topic + "\"," + mqttMsg.length());
    delay(800);
    GSMCOMMAND(String(mqttMsg));
    // if (waitForResponse(MQTTPUB, 1000))
    // {
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

bool MQTT::waitForResponse(int code, int mDelay)
{
    String dataReceived = "";
    unsigned long current_time = millis();
    while ((millis() - current_time) < mDelay)
    {
        if (GSM.available() > 2)
        {
            dataReceived = GSM.readStringUntil('\1');

            if (dataReceived.length() > 2)
            {
                switch (code)
                {
                case MQTTPORT:

                    if (dataReceived[2] == 'O' && dataReceived[3] == 'K')
                    {
                        LOG("Port established");
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    break;
                case MQTTCONN:
                    // for (int i = 0; i < dataReceived.length(); i++)
                    // {
                    //     LOG("Position :" + String(i) + " Item : " + dataReceived[i]);
                    // }
                    if (dataReceived[9] == 'Q' && dataReceived[10] == 'M' && dataReceived[11] == 'T' && dataReceived[18] == '0' && dataReceived[20] == '0' && dataReceived[22] == '0')
                    {
                        LOG("Connection established");
                        return true;
                    }
                    else if (dataReceived[2] == 'E' && dataReceived[3] == 'R' && dataReceived[4] == 'R' && dataReceived[5] == 'O' && dataReceived[6] == 'R')
                    {
                        LOG("Error is MQTT Connection");
                    }
                    else
                    {
                    }
                    return false;
                    break;
                case MQTTSUB:
                    LOG("Inside sub");
                    // for (int i = 0; i < dataReceived.length(); i++)
                    // {
                    //     LOG("Position :" + String(i) + " Item : " + dataReceived[i]);
                    // }
                    if (dataReceived[12] == 'S' && dataReceived[13] == 'U' && dataReceived[14] == 'B' && dataReceived[17] == '0' && dataReceived[19] == '1' && dataReceived[21] == '0' && dataReceived[23] == '0')
                    {
                        LOG("Subscription Success");
                        return true;
                    }
                    else
                    {
                        LOG("Subscription failed");
                        return false;
                    }
                    break;

                case MQTTPUB:
                    LOG("Inside PUB");
                    break;

                default:
                    return false;
                    break;
                }
            }
        }
    }
}