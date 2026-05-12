#ifndef __MQTT_H_
#define __MQTT_H_
#include <Arduino.h>
#define GSM Serial1
#define GSMCOMMAND GSM.println
#define LOG Serial.println
#define GSMCOMMANDL GSM.print

#define MQTTPORT 201
#define MQTTCONN 202
#define MQTTSUB 203
#define MQTTPUB 204

class MQTT
{
    int period = 1000;
    unsigned long time_now = 0;

    int retries;
    String mqttMsg;
    String topic;

private:
    String _hostname;
    unsigned int _port;

    String _username;
    String _password;

    String _mqttSub;

public:
    void setMQTTHOST(String hostname, unsigned int port);
    void setMQTTCred(String username, String password);
    bool connectMQTT();
    bool establishConnection(int retries);
    bool establishConnection();

    void setMqttSub(String mqttsub);
    bool mqttSub();

    bool waitForResponse(int code, int mDelay);

    bool mqttPub(String mqttMsg, String topic);
};
#endif //__MQTT_H_
