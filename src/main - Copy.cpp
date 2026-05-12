
#include <Arduino.h>
#include "TeensyThreads.h"
#include <ArduinoJson.h>
#include <InternalTemperature.h>
#include <TeensyID.h>
//#include <NativeEthernet.h>
#include <PubSubClient.h>
#include <Audio.h>
#include <TinyGPS++.h>
#include "USBHost_t36.h"
#include "HIDDumper.h"
#include "USBDeviceInfo.h"
#include <Watchdog_t4.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
// #include <EEPROM.h>
#include <imxrt.h>
#include <SPI.h>
#include <SdFat.h>
#include <SD.h>
#include "FXUtil.h"
extern "C"
{
#include "FlashTxx.h"
}

// ========== Thread Safety - Mutex Declarations ==========
Threads::Mutex connectionMutex;    // Protects: wifiConnected, mqttConnected, gsmMqttConnected, eg25moduleconnected
Threads::Mutex deviceStatusMutex;  // Protects: deviceBusyStatus, deviceLogStatus, qrScannerConnected
Threads::Mutex dataMutex;          // Protects: lastQRCode, nfcTicketID, gsmJSONResponse, tempGsmJsonResponse
Threads::Mutex publishMutex;       // Protects: isPublishing

// ========== Serial Buffer Management - Circular Buffer Implementation ==========
#define SERIAL_BUFFER_SIZE 2048

class CircularBuffer {
private:
  uint8_t buffer[SERIAL_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  Threads::Mutex bufferMutex;

public:
  CircularBuffer() : head(0), tail(0) {}

  bool write(uint8_t data) {
    Threads::Scope lock(bufferMutex);
    uint16_t nextHead = (head + 1) % SERIAL_BUFFER_SIZE;
    if (nextHead == tail) {
      return false; // Buffer full
    }
    buffer[head] = data;
    head = nextHead;
    return true;
  }

  int read() {
    Threads::Scope lock(bufferMutex);
    if (head == tail) {
      return -1; // Buffer empty
    }
    uint8_t data = buffer[tail];
    tail = (tail + 1) % SERIAL_BUFFER_SIZE;
    return data;
  }

  int available() {
    Threads::Scope lock(bufferMutex);
    if (head >= tail) {
      return head - tail;
    }
    return SERIAL_BUFFER_SIZE - tail + head;
  }

  int peek() {
    Threads::Scope lock(bufferMutex);
    if (head == tail) {
      return -1; // Buffer empty
    }
    return buffer[tail];
  }

  void clear() {
    Threads::Scope lock(bufferMutex);
    head = 0;
    tail = 0;
  }
};

// Serial buffer instances for buffered serial communication
// These buffers prevent data loss during high-traffic periods
//
// USAGE GUIDELINES:
// - Use serial1Buffer for GSM module (Serial1) communication
// - Use serial2Buffer for WiFi module (Serial2) communication
// - In interrupt/callback: Call buffer.write(byte) to store incoming data
// - In main thread: Call buffer.read() to retrieve data
// - Check buffer.available() before reading
// - The buffers are thread-safe with internal mutex protection
//
// Example ISR pattern:
//   void serialEvent1() {
//     while (Serial1.available()) {
//       serial1Buffer.write(Serial1.read());
//     }
//   }
//
// Example read pattern:
//   if (serial1Buffer.available() > 0) {
//     char c = serial1Buffer.read();
//     // process character
//   }

CircularBuffer serial1Buffer; // For GSM (Serial1)
CircularBuffer serial2Buffer; // For WiFi (Serial2)

File hexFile;
#define U9_RED A14
#define U9_GREEN A13
#define U8_RED A16
#define U8_GREEN A17
#define U8_BLUE A15
#define U5_RED A9
#define U5_GREEN A8
#define U5_BLUE A4
#define BUZZ A12
#define FLASHERX_VERSION "FlasherX v2.3"
#define HEX_FILE_NAME "flashX50.hex"
#define HEX_FILE_TEMP "temp.hex"
#define USBBAUD 115200
#define PORT_PCR_MUX(n)   (((n) & 0x7) << 8)
const char *wifiConfigFile = "/wifi.txt";
const char *bleConfigFile = "/bleConfig.txt";
const char *displaybrightessFile ="/displaybrightess.txt";

__attribute__((section(".fwid"))) String firmware_id = "fw_teensy41";

#define LARGE_ARRAY (0) // 1 = define large array to test large hex file

#if (LARGE_ARRAY)
// nested arrays of integers to add code size for testing
#define A0 \
  {        \
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15} // 16  elements 64
#define A1 \
  {        \
      A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0} // 256 elements 1KB
#define A2 \
  {        \
      A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1} // 4K  elements 16KB
#define A3 \
  {        \
      A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2} // 64K elements 256KB
#define A4 \
  {        \
      A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3} // 1M  elements 4MB

// const variables reside in flash and get optimized out if never accessed
// use uint8_t -> 1MB, uint16_t -> 2MB, uint32_t -> 4MB, uint64_t -> 8MB)
PROGMEM const uint8_t a[16][16][16][16][16] = A4;
#endif



uint8_t mac[6];
uint8_t serial[4];
 uint8_t serial1Buffer_hw[2048];  // Dedicated RX buffer for EG25 (Serial1)
 uint8_t serial2Buffer_hw[2048];  // Dedicated RX buffer for ESP32 WiFi (Serial2)
 uint8_t serial3Buffer_hw[2048];  // Dedicated RX buffer for GPS (Serial3)

unsigned long nextAllowedScan = 0;     
const unsigned long scanDelay = 1200;  

File wavFile;
File inputfile;
bool updateforvalid = false;
bool updatedSound = false;
String validUrl;
long validSize ;
String validFilename; 
bool updateforinvalid = false;
String invalidUrl;
long invalidSize;
String invalidFilename;


String urldata;
long Sizedataforbytes;
String filename;
long fileSize;
int totalbyteswritten = 0;


const char *INPUT_FILE = "/rawdata.txt";
const char *OUTPUT_FILE = "/output.wav";
unsigned long dataSize = 0;


 uint32_t buffer_addr, buffer_size;

  String response1 = "";




#define USBBAUD 115200
uint32_t baud = USBBAUD;
uint32_t format = USBHOST_SERIAL_8N1;

// USB Host setup for QR Scanner
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBDeviceInfo dinfo(myusb); // Device info collector
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);

// HID Dump Controllers for QR Scanner
HIDDumpController hdc1(myusb, 1);
HIDDumpController hdc2(myusb, 2);
HIDDumpController hdc3(myusb, 3);
HIDDumpController hdc4(myusb, 4);
HIDDumpController hdc5(myusb, 5);

USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2, &hid3, &hid4, &hid5};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2", "HID1", "HID2", "HID3", "HID4", "HID5"};
bool driver_active[CNT_DEVICES] = {false, false, false, false, false, false, false};

// HID Input drivers for QR Scanner
USBHIDInput *hiddrivers[] = {&hdc1, &hdc2, &hdc3, &hdc4, &hdc5};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_HIDDEVICES] = {"hdc1", "hdc2", "hdc3", "hdc4", "hdc5"};
bool hid_driver_active[CNT_HIDDEVICES] = {false, false, false, false, false};

// QR Scanner global variables
bool qrScannerConnected = false;
String qrScannerManufacturer = "";
String qrScannerProduct = "";
String qrScannerSerial = "";
String lastQRCode = ""; // Store last scanned QR code to prevent duplicates

// PN532 NFC Reader
#define PN532_IRQ   (2)
#define PN532_RESET (3)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
bool pn532Connected = false;
String pn532ChipInfo = "";
String pn532FirmwareVersion = "";

// Watchdog Timer
WDT_timings_t watchdogConfig;
WDT_T4<WDT1> wdt;

float validStoredVersion;
float invalidStoredVersion;
bool isvalid = false;
bool isinvalid = false;
float  validaudioupdate;
float  invalidaudioupdate;
float requiredGPSSpeedThreshold = 3.70; // in mhp
float firmwareVersion;
int fileIndex;
int allSerialBraudrate = 115200;
int GPSBaud = 9600;
int displayBaud = 9600;
int mqttPort = 1883;
int wifiLogPublishInterval = 120000;
int gsmLogPublishInterval = 10000;
int validTiceketDelay = 1500;
int invalidTicketDelay = 250;
int stopTicketDelay = 2000;
int validSpecialTiceketDelay = 1500;
int multipleTicketDelay = 500;
int specialTicketType = 1;
int mqttLogDelay = 50000;
int bleTxPower = 0;
int ibeaconMajor = 100;
int ibeaconMinor = 24;
int rssiat1m = -61;
int deviceLogStatus = 0;
int deviceBusyStatus = 0;
int gainValue = 10;
int gsmRSSI = -1;
int nfcType = 0;
int wifiRSSI = -1;
int requiredBleScanRssi = -40;
int deviceTicketTelematricHybridMode = 0; // 0 for Only TicketMode , 1 for only GPS mode , 2 hybridMode
long unsigned int requiredGPSsatellitesThreshold = 4;
unsigned int retrievedBLEMajor = 100;
unsigned int retrievedBLEMinor = 3001;
int retrievedBLETxPower = 4;
int retrivedrssi = -61;
const int cs = BUILTIN_SDCARD; // SD chip select pin
const int led = LED_BUILTIN;   // LED pin
int displaybrightnessvalue;
int startDeviceMode = 0;
// TOF Mode Variables
int TofModes = 0;

bool displaybrightcotrolcenter = true;

bool timeouterrorhappen = false;

int displayBrightnessLevel ;
bool displayBrightnessCondition;

unsigned char TOF_data[32] = {0}; // store 2 TOF frames
unsigned char TOF_length = 16;
unsigned char TOF_header[3]{0x57, 0x00, 0xFF};
unsigned long TOF_system_time = 0;
unsigned long TOF_distance = 0;
unsigned char TOF_status = 0;
unsigned int TOF_signal = 0;
unsigned char TOF_check = 0;
int peopleCount = 0;
bool personInsideThreshold = false;
unsigned long advertiseInterval = 10000; // 10 seconds in milliseconds
float tofSensorThreshold = 4;            // in feet
unsigned long lastAdvertiseTime = 0;
int peopleHold = 0;
unsigned long benchmarkEntryStartTime = 0;
bool benchmarkTimerStarted = false;
bool bleAdvertiseCalled = false;
int sendOutCardDataCount = 0;

String displaybrigtnesserror = "OK";
String TicketDisplaybrightnesserror ="OK";

String deviceTofTopic = "TOF/count";

Stream *serialIn = &Serial; // Serial (USB) or Serial1, Serial2, etc. (UART)


bool eg25moduleconnected = false;
bool wifiConnected = false;
bool mqttConnected = false;
bool isSdCardPresent = false;
bool gpsEnabled = false;
bool bleDfuMode = false;
bool bleIbeaconMode = true;
bool bleConnectMode = true;
bool enableMultipleLights = false;
bool bleAdStatus = false;
bool gsmSimFound = false;
bool gsmNetworkFound = false;
bool wifiApiDataState = false;
bool gsmMqttConnected = false;
bool sendOnlyGpsLog = true;
volatile bool enableWifiGsm = true; // false for gsm & true for wifi
bool buzzerEnable = true;
bool GsmProcessJsonStarted = false;
bool totalDeviceLog = true;
bool bleScanMode = false;
bool firstTimeDeviceLog = true;
bool gsmApnStatus = false;
bool gsmInternetStatus = false;
bool noInternetbleAdvertise = false;
bool deviceTicketTelematriceMode = false; // false for ticket mode , True of Telematrics mode
bool deviceHomePageStatus = true;
bool deviceInfoPageStatus = true;
bool displayLastTicket = true;
bool isDeviceInGPSTelemetryMode = false;
bool bleStartOnlyOneTime = true;
bool enableTofSensor = true;
bool isEthernetConnected = false;
bool enableToggleDisplayLines = false;
bool useSpeakerSound = false;
bool playSpeaker = false;
volatile bool isPublishing = false;
volatile bool espSerialBusy = false;      // Flag to pause listener during loop() reconnection
bool eg25ReadyForFailover = false;        // True if 4G MQTT verified available during WiFi mode (Phase 1)
bool isPlayingAudio = false;                     // Flag to track if audio is playing
const unsigned long audioPlaybackTimeout = 5000; // 10 seconds timeout for audio playback
unsigned long audioPlaybackStartTime = 0;

const char *wavefile[20] = {"Your_ticket_has_been_validated.wav", "microsoftsound.wav", "bell_notification.wav", "bluettoth_pair_sound.wav", "buzzer.wav", "callingbell.wav",
                            "low_battery.wav", "microsoftsound.wav", "oldalarm_sound.wav", "police_siren.wav", "spam_notifiaction.wav",
                            "vibration soundwav.wav", "wifi_connected.wav", "wifi_disconnected.wav", "windows notification sound.wav", "Invalid_entry_1.wav", "sos_signal.wav", "Invalid_entry_2.wav", "new_your_ticket_has_been_validated.wav"};

const char *ssid = "jayakand";
const char *password = "jai03072";
const char *mqttServer = "mqtt.zig-web.com";
const char *mqttServerAlt = "mq.zig-web.com";
const char *firmwareUrl;

// Audio needs

 AudioPlaySdWav playSdWav1; // xy=167,173      //FUNCTION TO PLAY SDCARD FILE
AudioMixer4 mixer1;        // xy=400,175      //MIXER FOR LEFT CHANNEL
AudioMixer4 mixer2;        // xy=440,284      //MIXER FOR RIGHT CHANNEL
AudioOutputI2S2 i2s2_1;    // xy=584,183

AudioConnection patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection patchCord2(playSdWav1, 1, mixer2, 0);
AudioConnection patchCord3(mixer1, 0, i2s2_1, 0);
AudioConnection patchCord4(mixer2, 0, i2s2_1, 1);

// ethernet
//byte etherNetmac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
//IPAddress ip(192, 168, 68, 177);

//EthernetClient ethClient;
//PubSubClient mqttClient(ethClient);

String primarySSID = "Zed_34";
String primaryPassword = "Wireless4U!";
String secondarySSID = "Zed";
String secondaryPassword = "Wireless4U!";
const unsigned long WifiTimeout = 30000;
// const char* mqttServer = "44.203.38.172";

unsigned long lastWifiLogPublishTime = 0; // Variable to store the last time the Wi-Fi log data was published
unsigned long lastGsmLogPublishTime = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long mqttReconnectInterval = 30000; // Try MQTT reconnect every 30 seconds
unsigned long lastMqttHealthCheck = 0;
unsigned long mqttHealthCheckInterval = 60000; // Check MQTT health every 60 seconds
unsigned long busySinceTime = 0;               // When device entered busy mode
unsigned long busyWatchdogTimeout = 120000;    // 2 min WiFi down + ESP32 hung → reboot
unsigned long mqttDownRebootTimeout = 600000;  // 10 min WiFi up but no internet/MQTT → reboot
unsigned long gsmBusySinceTime = 0;            // When 4G MQTT entered disconnected state
unsigned long gsmMqttDownRebootTimeout = 600000; // 10 min 4G MQTT down → reboot
unsigned long wifiDownSince = 0;               // When WiFi first dropped (Phase 2 failover timer)
unsigned long wifiFailoverTimeout = 120000;    // 2 min first attempt, 10 min after first fail
bool firstFailoverAttempted = false;           // True after first failed 4G switch — increases retry timeout

// ===== WiFi MQTT Publish Queue (single-thread serial access) =====
#define WIFI_QUEUE_SIZE 5
#define QUEUE_PRIORITY_HIGH 1  // Tickets/cards - time critical
#define QUEUE_PRIORITY_LOW  0  // Device logs - can wait

struct WifiQueueItem {
  String topic;
  String payload;
  int priority;
  bool occupied;
};

WifiQueueItem wifiQueue[WIFI_QUEUE_SIZE];
volatile bool wifiQueueLock = false;

void addToWifiQueue(const String &topic, const String &payload, int priority)
{
  // Simple spinlock
  while (wifiQueueLock) { delay(1); }
  wifiQueueLock = true;

  int slot = -1;
  int lowestSlot = -1;

  for (int i = 0; i < WIFI_QUEUE_SIZE; i++)
  {
    if (!wifiQueue[i].occupied)
    {
      slot = i;
      break;
    }
    if (lowestSlot == -1 || wifiQueue[i].priority < wifiQueue[lowestSlot].priority)
    {
      lowestSlot = i;
    }
  }

  // High priority ticket replaces low priority log if queue full
  if (slot == -1 && priority == QUEUE_PRIORITY_HIGH && lowestSlot != -1 && wifiQueue[lowestSlot].priority == QUEUE_PRIORITY_LOW)
  {
    slot = lowestSlot;
    Serial.println("WiFi queue full - ticket replacing log");
  }

  if (slot != -1)
  {
    wifiQueue[slot].topic = topic;
    wifiQueue[slot].payload = payload;
    wifiQueue[slot].priority = priority;
    wifiQueue[slot].occupied = true;
    Serial.println("Queued to WiFi [" + String(priority == QUEUE_PRIORITY_HIGH ? "HIGH" : "LOW") + "]: " + topic);
  }
  else
  {
    Serial.println("WiFi queue full - message dropped");
  }

  wifiQueueLock = false;
}

// Returns index of highest priority item, -1 if empty
int getNextWifiQueueItem()
{
  int best = -1;
  for (int i = 0; i < WIFI_QUEUE_SIZE; i++)
  {
    if (wifiQueue[i].occupied)
    {
      if (best == -1 || wifiQueue[i].priority > wifiQueue[best].priority)
      {
        best = i;
      }
    }
  }
  return best;
}
// ===== End WiFi MQTT Queue =====

// ===== GSM ACK Queue (outgoing /ack publishes for 4G path) =====
#define GSM_ACK_QUEUE_SIZE 5

struct GsmAckQueueItem {
  String payload;
  bool occupied;
};

GsmAckQueueItem gsmAckQueue[GSM_ACK_QUEUE_SIZE];
volatile bool gsmAckQueueLock = false;

void addToGsmAckQueue(const String &payload)
{
  while (gsmAckQueueLock) { delay(1); }
  gsmAckQueueLock = true;

  int slot = -1;
  for (int i = 0; i < GSM_ACK_QUEUE_SIZE; i++)
  {
    if (!gsmAckQueue[i].occupied)
    {
      slot = i;
      break;
    }
  }

  if (slot != -1)
  {
    gsmAckQueue[slot].payload = payload;
    gsmAckQueue[slot].occupied = true;
    Serial.println("GSM ACK queued: slot " + String(slot));
  }
  else
  {
    Serial.println("GSM ACK queue full - dropped");
  }

  gsmAckQueueLock = false;
}

int getNextGsmAckQueueItem()
{
  for (int i = 0; i < GSM_ACK_QUEUE_SIZE; i++)
  {
    if (gsmAckQueue[i].occupied)
      return i;
  }
  return -1;
}
// ===== End GSM ACK Queue =====


// ===== Buffered incoming MQTT messages (caught during AT command waits) =====
#define MQTT_RECV_BUFFER_SIZE 3
String mqttRecvBuffer[MQTT_RECV_BUFFER_SIZE];
int mqttRecvBufferCount = 0;

void bufferMqttRecv(const String &msg)
{
  if (mqttRecvBufferCount < MQTT_RECV_BUFFER_SIZE)
  {
    mqttRecvBuffer[mqttRecvBufferCount] = msg;
    mqttRecvBufferCount++;
    Serial.println("Buffered incoming MQTT message during publish");
  }
  else
  {
    Serial.println("MQTT recv buffer full - message lost");
  }
}
void processBufferedMqttMessages(); // Forward declaration - defined after ProcessDisplayData
void drainingDelay(unsigned long ms); // Forward declaration - defined after espSerial
// ===== End Buffered MQTT =====

// ===== Buffered incoming GSM MQTT messages (caught during AT command waits) =====
#define GSM_MQTT_RECV_BUFFER_SIZE 3
String gsmMqttRecvBuffer[GSM_MQTT_RECV_BUFFER_SIZE];
int gsmMqttRecvBufferCount = 0;

void bufferGsmMqttRecv(const String &msg)
{
  if (gsmMqttRecvBufferCount < GSM_MQTT_RECV_BUFFER_SIZE)
  {
    gsmMqttRecvBuffer[gsmMqttRecvBufferCount] = msg;
    gsmMqttRecvBufferCount++;
    Serial.println("Buffered incoming GSM MQTT message during publish");
  }
  else
  {
    Serial.println("GSM MQTT recv buffer full - message lost");
  }
}
void processBufferedGsmMqttMessages(); // Forward declaration
// ===== End Buffered GSM MQTT =====

String lastTicketTID = "";
String lastNFC = ""; // Variable to store the last time the GSM log data was published
String response = "";
String atVersion;
String dateString;
String timeString;
String storedOutputString = "";
String EepromWifiSsid = "";
String EepromWifiPass = "";
String bleCardMacAddress;
String retrievedWifiSsid;
String retrievedWifiPass;
String pushApiDataMqttStatus = "GET";
String reloadApiDataMqttStatus = "RELOAD";
String homepageHeadingStatus1 = "READY TO VALIDATE";
String homepageHeadingStatus2 = "PLEASE WAIT";
String homepageHeadingStatus3 = "OFFLINE MODE";
String homepageHeadingStatus4 = "CARD DETECTED";
String homepageHeadingStatus5 = "BENHCMARK MODE";
String previousBleCardNo = "";  // Variable to store the previous card number
String detectedDeviceInfo = ""; // Global string to store product name and serial number
String nfcTicketID = "";
String simNumber = "";
String nfcName = "";
String latitude = "N/A";
String longitude = "N/A";
String speed = "N/A";
String noOfStatilte = "N/A";
String deviceDate = "N/A";
String deviceTime = "N/A";
String bleVersion = "N/A";
String bleMacAddress = "N/A";
String GsmOperatorName = "N/A";
String GsmNetworkTypeString = "N/A";
String deviceLogPayload;
String gsmJSONResponse = "";
String tempGsmJsonResponse = "";
String insertSimDisplayPage = "page 5";          // checked
String deviceUnauthorizedDisplayPage = "page 6"; // checked
String invaildDataDisplayPage = "page 11";
String zigNetworkFoundDisplayPage = "page 12"; // checked
String zigNetworkFailedDisplayPage = "page 13";
String switchingTo4GDisplayPage = "page 15";
String gpsData = "N/A";
String displayLine1 = "Device";
String displayLine2 = "OFFLINE";
String tofDisplayPrefix = "Distance: ";
String eepromErrorStatus = "OK";
String lastUserId = "";
String currentOperator = "";
String connectedSSID = "";
String codeVersion = "9.8"; // Change the version when the code is pushed
String phoneNumber = "Not available";
// String deviceMacAddress = "98:CD:AC:51:4A:BC";
String deviceMacAddress = teensyMAC();
String deviceTeensyMacAddress = teensyMAC();
String deviceTopic = deviceMacAddress + "/nfc";
String deviceReactTopic = deviceMacAddress + "/react";
String deviceLogTopic = deviceMacAddress + "/log";
String deviceBleCardTopic = deviceMacAddress + "/card";
String deviceBenchMarkTopic = deviceMacAddress + "/ben";
String deviceNetworkTopic = deviceMacAddress + "/networks";
String deviceErrorTopic = deviceMacAddress + "/error";
String deviceAckTopic = deviceMacAddress + "/ack";
String ethernetError = "";
String validTicketStatus = "201";
String invalidTicketStatus = "301";
String sosStatus = "SOS";
String stopStatus = "STOP";
String bleStartCommand = "start";
String blename = "BIBO 1.1 A";
String bleNrf52Command = bleStartCommand + "#" + blename + "#" + String(bleTxPower) + "#" +
                         (bleDfuMode ? "true" : "false") + "#" + String(ibeaconMajor) + "#" +
                         String(ibeaconMinor) + "#" + (bleIbeaconMode ? "true" : "false") + "#" + (bleScanMode ? "true" : "false") + "#" +
                         (bleConnectMode ? "true" : "false") + "#"+String(rssiat1m) ;
String apiResponse; // Global variable to store the API response
String TofData = deviceMacAddress + "#" + String(peopleCount);
String deviceApiUrl =  "https://zig-app.com/bjcta/hardware/proxy/api/device-route/v1/get-individual-zig-device?device_id=" + String(teensyMAC());
String apiError = "OK";
String audioUpdateErrorValid = "OK";
String audioUpdateErrorInvalid = "OK";
// String bleNrf52Command = "start#QLE_V1#-8#false#100#37#true#true";
//#define espSerial Serial2
HardwareSerial &espSerial = Serial2;     // Define Serial2 as ESP32
HardwareSerial &nrf52 = Serial7;         // Define Serial7 as nrf5238
HardwareSerial &eg25 = Serial1;          // Define Serial1 as EG25-G
HardwareSerial &gpsSerial = Serial3;     // Define Serial4 as gps module
HardwareSerial &displaySerial = Serial6; // Define Serial6 as display
HardwareSerial &tofSerial = Serial4;     // Define Serial4 as TOF sensor

// Draining wait - reads serial and buffers MQTT messages instead of blind delay()
void drainingDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    if (espSerial.available())
    {
      String line = espSerial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0 && line.indexOf("+MQTTSUBRECV:") != -1)
      {
        bufferMqttRecv(line);
      }
    }
  }
}

void publishLogFireAndForget(const String &topic, const String &payload); // Forward declaration

TinyGPSPlus gps;
File file;

// ========== Thread-Safe Accessor Functions ==========
// These inline functions provide thread-safe access to shared variables
//
// USAGE GUIDELINES:
// - For NEW code: Always use these accessor functions instead of direct variable access
// - For EXISTING code: Variables can still be accessed directly (backward compatible)
// - The accessor functions prevent race conditions in multi-threaded access
//
// Protected Variables:
// - wifiConnected, mqttConnected, gsmMqttConnected, eg25moduleconnected (connectionMutex)
// - deviceBusyStatus, qrScannerConnected (deviceStatusMutex)
// - lastQRCode, nfcTicketID, gsmJSONResponse (dataMutex)
// - isPublishing (publishMutex)
//
// Example: Instead of "deviceBusyStatus = 2", use "setDeviceBusyStatus(2)"

// Connection state accessors
inline void setWifiConnected(bool value) {
  Threads::Scope lock(connectionMutex);
  wifiConnected = value;
}

inline bool getWifiConnected() {
  Threads::Scope lock(connectionMutex);
  return wifiConnected;
}

inline void setMqttConnected(bool value) {
  Threads::Scope lock(connectionMutex);
  mqttConnected = value;
}

inline bool getMqttConnected() {
  Threads::Scope lock(connectionMutex);
  return mqttConnected;
}

inline void setGsmMqttConnected(bool value) {
  Threads::Scope lock(connectionMutex);
  gsmMqttConnected = value;
}

inline bool getGsmMqttConnected() {
  Threads::Scope lock(connectionMutex);
  return gsmMqttConnected;
}

inline void setEg25ModuleConnected(bool value) {
  Threads::Scope lock(connectionMutex);
  eg25moduleconnected = value;
}

inline bool getEg25ModuleConnected() {
  Threads::Scope lock(connectionMutex);
  return eg25moduleconnected;
}

// Device status accessors
inline void setDeviceBusyStatus(int value) {
  Threads::Scope lock(deviceStatusMutex);
  deviceBusyStatus = value;
}

inline int getDeviceBusyStatus() {
  Threads::Scope lock(deviceStatusMutex);
  return deviceBusyStatus;
}

inline void setQrScannerConnected(bool value) {
  Threads::Scope lock(deviceStatusMutex);
  qrScannerConnected = value;
}

inline bool getQrScannerConnected() {
  Threads::Scope lock(deviceStatusMutex);
  return qrScannerConnected;
}

// Data accessors
inline void setLastQRCode(const String& value) {
  Threads::Scope lock(dataMutex);
  lastQRCode = value;
}

inline String getLastQRCode() {
  Threads::Scope lock(dataMutex);
  return lastQRCode;
}

inline void setNfcTicketID(const String& value) {
  Threads::Scope lock(dataMutex);
  nfcTicketID = value;
}

inline String getNfcTicketID() {
  Threads::Scope lock(dataMutex);
  return nfcTicketID;
}

inline void setGsmJSONResponse(const String& value) {
  Threads::Scope lock(dataMutex);
  gsmJSONResponse = value;
}

inline String getGsmJSONResponse() {
  Threads::Scope lock(dataMutex);
  return gsmJSONResponse;
}

// Publishing flag accessors
inline void setIsPublishing(bool value) {
  Threads::Scope lock(publishMutex);
  isPublishing = value;
}

inline bool getIsPublishing() {
  Threads::Scope lock(publishMutex);
  return isPublishing;
}

// EG25 Functions
void threadNFC();
void Httpcget();
void setInternetRegistration();
void activateInternet();
void gsmMqttConnectFlow();
void getConnectedNetwork();
void resetMQTT();
void setMQTTMode();
void openMQTT();
void connectMQTT(bool reconnectUntilSuccessful);
void subscribeMQTT(const String &topic);
void restartEg25MqttFlow();
void bleAdvertisebegin();
void nrf52SerialPortListener();
void getApiResponse(const String &apiUrl);
int getGsmRssiData();
int getWifiRSSI();
String getWifiBSSID();
void disconnectPreviousMqtt();
void sendHTTPCGET(const char *url);
void verifyHexData(const char *url, const char *fileName);
void hexInitDownload(const char *url, const char *fileName);
void initializeSerial();
void initializeSDCard();
void deleteHexFileIfExists();
void versionCheck();
void publishDeviceLogData();
void publishRawToTopic(const String &topic, const String &payload, int size);
void publishRawToTopicEG25(const String &topic, const String &payload);
void publishLogFireAndForgetEG25(const String &topic, const String &payload);
void processBufferedGsmMqttMessages();
void removeFirstLine(const char *fileName);
void publishNFCData(String nfcName, String nfcTicketID);
void displayAllConnectedPage();
void playWavFile(const char *filename);
bool loadBLEConfig();
void ethernetBleData();
void updatefirmware();
void ProcessDisplayData(String str);
void detectPN532();
String generateRandomString(size_t length);
void ethernetConnectToMqtt();
void mqttCallback(char *topicReceived, byte *payload, unsigned int length);
String getESP32MAC();

void bleNrf52Init(String command);
String sendATCommand3(const String &command, const String &expectedResponse, unsigned long timeout);
bool checkWifiAndMqttStatus();
void goOffline();
void goMqttOffline();
void goGsmMqttOffline();
void gsmMqttConnectFlowSingleAttempt();
void processBufferedMqttMessages();
String getWifiBSSID();
// Function to connect to the MQTT server

/*void ethernetConnectToMqtt()
{
  Serial.print("Connecting to MQTT server... ");
  String revicevedMqttClientId = generateRandomString(4);
  String clientId = deviceTeensyMacAddress + "-" + "ETHERNET" + "-" + revicevedMqttClientId + teensySN();
  if (mqttClient.connect(clientId.c_str()))
  {
    Serial.println("Connected to MQTT server!");
    mqttClient.subscribe(deviceTopic.c_str());
    Serial.print("Subscribed to topic: ");
    Serial.println(deviceTopic);
    Serial.println("Connected to MQTT server!");
    mqttClient.subscribe(deviceReactTopic.c_str());
    Serial.print("Subscribed to topic: ");
    Serial.println(deviceReactTopic);
    isEthernetConnected = true;
  }
  else
  {
    Serial.print("Failed to connect, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" - trying again in 5 seconds...");
    delay(5000);
    isEthernetConnected = false;
  }
}*/

// MQTT callback function to handle incoming messages
void mqttCallback(char *topicReceived, byte *payload, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topicReceived);
  Serial.print(". Message: ");

  // Convert payload to String
  String message;
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println(message);
  ProcessDisplayData(message);
}

void ethernetBleData()
{
  if (loadBLEConfig())
  {
    // Use retrievedBLEMajor, retrievedBLEMinor, and retrievedBLETxPower as needed
    Serial.println("Retrieved BLE Data from BLE CREDS:");
    Serial.print("Retrieved BLE Major: ");
    Serial.println(retrievedBLEMajor);
    ibeaconMajor = retrievedBLEMajor;
    Serial.print("Retrieved BLE Minor: ");
    Serial.println(retrievedBLEMinor);
    ibeaconMinor = retrievedBLEMinor;
    Serial.print("Retrieved BLE TX Power: ");
    Serial.println(retrievedBLETxPower);
    bleTxPower = retrievedBLETxPower;
    Serial.print("Retrieved Rssiat1m: ");
    Serial.println(retrivedrssi);
    rssiat1m = retrivedrssi;
    bleNrf52Command = bleStartCommand + "#" + blename + "#" + String(bleTxPower) + "#" +
                      (bleDfuMode ? "true" : "false") + "#" + String(ibeaconMajor) + "#" +
                      String(ibeaconMinor) + "#" + (bleIbeaconMode ? "true" : "false") + "#" + (bleScanMode ? "true" : "false") + "#" +
                      (bleConnectMode ? "true" : "false")+ "#"+String(rssiat1m) ;
  }
  else
  {
    Serial.println("Retrieved BLE Data from STATIC DATA:");
    Serial.print("Retrieved BLE Major: ");
    Serial.println(retrievedBLEMajor);
    ibeaconMajor = retrievedBLEMajor;
    Serial.print("Retrieved BLE Minor: ");
    Serial.println(retrievedBLEMinor);
    ibeaconMinor = retrievedBLEMinor;
    Serial.print("Retrieved BLE TX Power: ");
    Serial.println(retrievedBLETxPower);
    bleTxPower = retrievedBLETxPower;
     Serial.print("Retrieved Rssiat1m: ");
    Serial.println(retrivedrssi);
    rssiat1m = retrivedrssi;
    bleNrf52Command = bleStartCommand + "#" + blename + "#" + String(bleTxPower) + "#" +
                      (bleDfuMode ? "true" : "false") + "#" + String(ibeaconMajor) + "#" +
                      String(ibeaconMinor) + "#" + (bleIbeaconMode ? "true" : "false") + "#" + (bleScanMode ? "true" : "false") + "#" +
                      (bleConnectMode ? "true" : "false")+ "#"+String(rssiat1m) ;
  }
  if (bleStartOnlyOneTime)
  {
    Serial.println("+++++++++++++BLE process begin++++++++++++++");
    bleNrf52Init(bleNrf52Command);
    // threads.addThread(nrf52SerialPortListener);
    bleStartOnlyOneTime = false;
  }
}
bool loadBLEConfig()
{
  File file = SD.open(bleConfigFile, FILE_READ);
  if (file)
  {
    retrievedBLEMajor = file.parseInt();
    retrievedBLEMinor = file.parseInt();
    retrievedBLETxPower = file.parseInt();
    retrivedrssi = file.parseInt();

    Serial.println("BLE configuration loaded from SD card:");
    Serial.print("BLE Major: ");
    Serial.println(retrievedBLEMajor);
    Serial.print("BLE Minor: ");
    Serial.println(retrievedBLEMinor);
    Serial.print("BLE Tx Power: ");
    Serial.println(retrievedBLETxPower);
    Serial.print("Retrieved Rssiat1m: ");
    Serial.println(retrivedrssi);


    file.close();
    return true;
  }
  else
  {
    Serial.println("Failed to open file for reading.");
    return false;
  }
}

void saveBLEConfig(int bleMajor, int bleMinor, int bleTxPower, int rssilevel)
{
  if (SD.exists(bleConfigFile))
  {
    SD.remove(bleConfigFile);
  }

  File file = SD.open(bleConfigFile, FILE_WRITE);
  if (file)
  {
    file.println(bleMajor);
    file.println(bleMinor);
    file.println(bleTxPower);
    file.println(rssilevel);
    file.close();
    Serial.println("BLE configuration saved to SD card.");
  }
  else
  {
    Serial.println("Failed to open file for writing.");
  }
}






void playWavFile()
{
   
CORE_PIN2_CONFIG  = 2;
CORE_PIN3_CONFIG  = 2;
CORE_PIN4_CONFIG  = 2;
CORE_PIN5_CONFIG  = 2;
CORE_PIN33_CONFIG = 2;
 

  if (isPlayingAudio)
  {
    Serial.println("Audio is already playing. Skipping request.");
    return; // Exit if audio is already playing
  }

  const char *filename;
  // Select the file based on fileIndex
  switch (fileIndex)
  {
  case 1:
    filename = "buzzer.wav";
    break;
  case 2:
    filename = "Invalid_entry_2.wav";
    break;
  case 3:
    filename = "low_battery.wav";
    break;
  case 4:
    filename = "wifi_connected.wav";
    break;
  case 5:
    filename = "sos_signal.wav";
    break;
  case 6:
    filename = "beep.wav";
    break;
  case 7:
    filename = "buzzer_double_sound.wav";
    break;
  case 8:
    filename = "buzzer_triple_sound.wav";
    break;
  default:
    Serial.println("Invalid file index. Please choose between 1 and 5.");
    playSpeaker = false;
    return;
  }

  isPlayingAudio = true;             // Set the flag to indicate audio is playing
  audioPlaybackStartTime = millis(); // Record the time when playback starts
  Serial.print("Playing file: ");
  Serial.println(filename);

  if (!playSdWav1.play(filename))
  {
    Serial.println("Error: Failed to start playback.");
    playSpeaker = false;
    isPlayingAudio = false; // Reset the flag if playback fails
    return;
  }
  delay(5);

  // Wait until the audio finishes playing
  while (playSdWav1.isPlaying())
  {
    wdt.feed();
  }

  delay(500);
  Serial.println("Done playing file");
  isPlayingAudio = false; // Reset the flag to indicate audio has stopped
  playSpeaker = false;  
  delay(10);
CORE_PIN2_CONFIG  = 0;
CORE_PIN3_CONFIG  = 0;
CORE_PIN4_CONFIG  = 0;
CORE_PIN5_CONFIG  = 0;
CORE_PIN33_CONFIG = 0;

}






void displayCommand(String command)
{
  displaySerial.print(command);
  displaySerial.write(0xff);
  displaySerial.write(0xff);
  displaySerial.write(0xff);
}

void writedisplayvalueepprom(int value)
{
  
  if (SD.exists(displaybrightessFile))
  {
    SD.remove(displaybrightessFile);
  }

  File file = SD.open(displaybrightessFile, FILE_WRITE);
  if (file)
  {
    file.println(value);
    file.close();
    Serial.println("Brightness value saved to SD card.");
  }
  else
  {
    Serial.println("Failed to open file for writing for brightess value.");
  }
}








void  displaybrightesscontrol(int brightessvaluefordiplay)
{
  displayCommand("dim=" + String(brightessvaluefordiplay));
 // Serial.println(brightessvaluefordiplay);
  if(displaybrightcotrolcenter == true)
  {
  writedisplayvalueepprom(brightessvaluefordiplay);
  }
  else{
    
  }
}




String getIMEI()
{
  eg25.println("AT+GSN");
  unsigned long timeout = millis() + 10000; 
  while (millis() < timeout)
  {
    if (eg25.available())
    {
      String response = eg25.readStringUntil('\n');
      response.trim(); // Remove whitespace

       if (response.length() >= 14 && isDigit(response.charAt(0))) {
        return response;  // Return IMEI
        
      }
      else if (response.indexOf("ERROR") != -1)
      {
        return "ERROR";
      
      }
    }
  }
  return "TIMEOUT";
  
}






String getESP32MAC()
{
  espSerial.println("AT+CIPSTAMAC?");      // Query the MAC address
  unsigned long timeout = millis() + 5000; // Set a timeout
  while (millis() < timeout)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      response.trim(); // Remove whitespace

      if (response.startsWith("+CIPSTAMAC:"))
      {
        // Example response: +CIPSTAMAC:"24:6F:28:12:34:56"
        int macStart = response.indexOf('"') + 1;
        int macEnd = response.indexOf('"', macStart);
        if (macStart > 0 && macEnd > macStart)
        {
          return response.substring(macStart, macEnd); // Extract MAC address
        }
      }
      else if (response.indexOf("ERROR") != -1)
      {
        return "ERROR";
      }
    }
  }
  return "TIMEOUT";
}

// Function to send Wi-Fi AT commands and process responses
String getWifiNetworksJSON(unsigned long timeout)
{
  // Send AT command to list available Wi-Fi networks
  espSerial.println("AT+CWLAP");
  unsigned long startTime = millis();

  // Create a JSON document to hold the results
  const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(10) * JSON_OBJECT_SIZE(2) * 2 + 1024;
  DynamicJsonDocument doc(capacity);

  // Create JSON arrays for open and secured networks
  JsonArray openNetworks = doc.createNestedArray("openNetworks");
  JsonArray securedNetworks = doc.createNestedArray("securedNetworks");

  while (millis() - startTime < timeout)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      response.trim(); // Remove whitespace characters

      // Look for "+CWLAP" response line
      if (response.startsWith("+CWLAP:"))
      {
        // Parse the response line
        // Example: +CWLAP:(3,"SSID",-54,"54:af:97:c4:e2:c7",1,-1,-1,4,4,7,0)
        int encTypeStart = response.indexOf('(') + 1;
        int encTypeEnd = response.indexOf(',', encTypeStart);
        int ssidStart = response.indexOf('"') + 1;
        int ssidEnd = response.indexOf('"', ssidStart);
        int rssiStart = response.indexOf(',', ssidEnd) + 1;

        // Extract encryption type, SSID, and RSSI
        int encType = response.substring(encTypeStart, encTypeEnd).toInt();
        String ssid = "";
        if (ssidStart > 0 && ssidEnd > ssidStart)
        {
          ssid = response.substring(ssidStart, ssidEnd); // Extract SSID
        }
        String rssiStr = response.substring(rssiStart, response.indexOf(',', rssiStart));
        int rssi = rssiStr.toInt(); // Extract RSSI as integer

        // Add to the appropriate JSON array based on encryption type
        JsonObject network;
        if (encType == 0) // Open network
        {
          network = openNetworks.createNestedObject();
        }
        else // Secured network
        {
          network = securedNetworks.createNestedObject();
        }

        network["ssid"] = ssid;
        network["rssi"] = rssi;
      }
      else if (response.indexOf("ERROR") != -1)
      {
        doc["error"] = "Failed to scan Wi-Fi networks";
        break;
      }
    }
  }

  // Serialize JSON to a string
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString; // Return the JSON string
}

void handleTofModes(int TofModes)
{
  switch (TofModes)
  {
  case 0:
    enableTofSensor = false;
    bleAdvertisebegin();
    Serial.println("++++++Default BLE mode++++++");
    break;
  case 1:
    enableTofSensor = true;
    Serial.println("++++++BLE on off with tof ++++++");
    break;
  case 2:
    enableTofSensor = true;
    Serial.println("++++++BLE on off with tof (With People Count) ++++++");
    break;
  case 3:
    enableTofSensor = true;
    bleAdvertisebegin();
    Serial.println("++++++(With People Count with illegal detection) ++++++");
    break;
  case 4:
    enableTofSensor = true;
    bleAdvertisebegin();
    Serial.println("++++++(BENCHMARK MODE) ++++++");
    break;
  case 5:
    enableTofSensor = true;
    bleAdvertisebegin();
    Serial.println("++++++(With People Count) ++++++");
    break;
  case 6:
    enableTofSensor = true;
    bleAdvertisebegin();
    Serial.println("++++++(CALIBRATION MODE) ++++++");
    break;
  default:
    enableTofSensor = false;
    bleAdvertisebegin();
    Serial.println("++++++TOF API INPUT ERROR++++++");
    break;
  }
}
void lightUpLED(int U9, int U5, int U8, int BZ)
{
  // Port registers for each port
  volatile uint32_t *portU9 = portOutputRegister(digitalPinToPort(U9_RED));
  volatile uint32_t *portU5 = portOutputRegister(digitalPinToPort(U5_RED));
  volatile uint32_t *portU8 = portOutputRegister(digitalPinToPort(U8_RED));
  volatile uint32_t *portBZ = portOutputRegister(digitalPinToPort(BUZZ));

  // Pin masks for each pin
  uint32_t maskU9_RED = digitalPinToBitMask(U9_RED);
  uint32_t maskU9_GREEN = digitalPinToBitMask(U9_GREEN);
  uint32_t maskU5_RED = digitalPinToBitMask(U5_RED);
  uint32_t maskU5_GREEN = digitalPinToBitMask(U5_GREEN);
  uint32_t maskU5_BLUE = digitalPinToBitMask(U5_BLUE);
  uint32_t maskU8_RED = digitalPinToBitMask(U8_RED);
  uint32_t maskU8_GREEN = digitalPinToBitMask(U8_GREEN);
  uint32_t maskU8_BLUE = digitalPinToBitMask(U8_BLUE);
  uint32_t maskBZ = digitalPinToBitMask(BUZZ);

  // U9
  if (U9 == 0)
  {
    *portU9 &= ~(maskU9_RED | maskU9_GREEN);
  }
  else if (U9 == 1)
  {
    *portU9 = (*portU9 & ~maskU9_GREEN) | maskU9_RED;
  }
  else if (U9 == 2)
  {
    *portU9 = (*portU9 & ~maskU9_RED) | maskU9_GREEN;
  }

  // U5
  if (U5 == 0)
  {
    *portU5 &= ~(maskU5_RED | maskU5_GREEN | maskU5_BLUE);
  }
  else if (U5 == 1)
  {
    *portU5 = (*portU5 & ~(maskU5_GREEN | maskU5_BLUE)) | maskU5_RED;
  }
  else if (U5 == 2)
  {
    *portU5 = (*portU5 & ~(maskU5_RED | maskU5_BLUE)) | maskU5_GREEN;
  }
  else if (U5 == 3)
  {
    *portU5 = (*portU5 & ~(maskU5_RED | maskU5_GREEN)) | maskU5_BLUE;
  }
  else if (U5 == 4)
  {
    *portU5 = (*portU5 & ~maskU5_BLUE) | (maskU5_RED | maskU5_GREEN);
  }

  // U8
  if (U8 == 0)
  {
    *portU8 &= ~(maskU8_RED | maskU8_GREEN | maskU8_BLUE);
  }
  else if (U8 == 1)
  {
    *portU8 = (*portU8 & ~(maskU8_GREEN | maskU8_BLUE)) | maskU8_RED;
  }
  else if (U8 == 2)
  {
    *portU8 = (*portU8 & ~(maskU8_RED | maskU8_BLUE)) | maskU8_GREEN;
  }
  else if (U8 == 3)
  {
    *portU8 = (*portU8 & ~(maskU8_RED | maskU8_GREEN)) | maskU8_BLUE;
  }
  else if (U8 == 4)
  {
    *portU8 = (*portU8 & ~maskU8_BLUE) | (maskU8_RED | maskU8_GREEN);
  }

  // BUZZ
  *portBZ = (BZ == 0) ? (*portBZ & ~maskBZ) : (*portBZ | maskBZ);
}



void saveWifiConfig(const String &ssid, const String &password)
{
  if (SD.exists(wifiConfigFile))
  {
    SD.remove(wifiConfigFile);
  }

  File file = SD.open(wifiConfigFile, FILE_WRITE);
  if (file)
  {
    file.println(ssid);
    file.println(password);
    file.close();
    Serial.println("WiFi credentials saved to SD card.");
  }
  else
  {
    Serial.println("Failed to open file for writing.");
  }
}

void DisplayHomepage(String msg1, String msg2)
{

  if (deviceHomePageStatus)
  {
    
    displayCommand("page 1");
    displaybrightcotrolcenter = false;
     displaybrightesscontrol(displayBrightnessLevel);
  }
  if (deviceBusyStatus == 1)
  {
    displayCommand("p4.pic=16");
    displayCommand("t2.txt=\"" + homepageHeadingStatus1 + "\"");
  }
  else if (deviceBusyStatus == 0)
  {
    if (enableToggleDisplayLines)
    {
      displayCommand("t2.txt=\"" + homepageHeadingStatus2 + "\"");
    }
    else
    {
      displayCommand("t2.txt=\"" + homepageHeadingStatus1 + "\"");
    }

    displayCommand("p4.pic=14");
    
  }
  else if (deviceBusyStatus == 2)
  {
    displayCommand("t2.txt=\"" + homepageHeadingStatus4 + "\"");
    displayCommand("p4.pic=14");
  }

  if (wifiConnected && mqttConnected)
  { // Wifi or GSM status icon
    displayCommand("p1.pic=9");
  }
  else if (gsmInternetStatus && gsmMqttConnected)
  {
    displayCommand("p1.pic=8");
  }
  else
  {
    displayCommand("t2.txt=\"" + homepageHeadingStatus3 + "\"");
    displayCommand("p1.pic=10");
    displayCommand("p4.pic=15");
  }
  if (bleAdStatus)
  { // Ble ad status icon
    displayCommand("p2.pic=7");
  }
  else
  {
    displayCommand("p2.pic=17");
  }
  if (deviceTicketTelematriceMode || deviceTicketTelematricHybridMode == 1)
  { // device mode status icon
    displayCommand("p3.pic=12");
  }
  else
  {
    displayCommand("p3.pic=11");
  }
  if (gps.location.isValid())
  { // device GPS status status icon
    displayCommand("p5.pic=13");
  }
  else 
  {
    displayCommand("p5.pic=17");
  }
  // if(enableWifiGsm){
  //   displayCommand("p1.pic=12");
  // }else{
  //   displayCommand("p1.pic=7");
  // }
  delay(200);
  displayCommand("t0.txt=\"" + msg1 + "\"");
  displayCommand("t1.txt=\"" + codeVersion + "\"");
  deviceHomePageStatus = false;
  deviceInfoPageStatus = true;
}

void DisplayInfo(String msg1, String msg2, String msg3)
{
  if (deviceInfoPageStatus)
  {
    displayCommand("page 10");
  }
  delay(200);
  displayCommand("t0.txt=\"" + msg1 + "\"");
  displayCommand("t1.txt=\"" + msg2 + "\"");
  displayCommand("t2.txt=\"" + msg3 + "\"");
  deviceInfoPageStatus = false;
  deviceHomePageStatus = true;
}
void DiplayScreenTicketID(String TicketUserNameSend, int TicketIDSend, int NoOfTicketSend, String backGroundColor, String fontColor, String TicketContent, String TranspartentColor)
{
  displayCommand("page 14");
  displayCommand("page14.bco=" + backGroundColor);
  displayCommand("n0.bco=" + backGroundColor);
  displayCommand("n0.pco=" + fontColor);
  displayCommand("n0.val=" + String(NoOfTicketSend));
  displayCommand("t3.bco=" + backGroundColor);
  displayCommand("t3.pco=" + fontColor);
  displayCommand("t2.bco=" + backGroundColor);
  displayCommand("t2.pco=" + fontColor);
  displayCommand("t2.txt=\"" + TicketUserNameSend + "\"");
  displayCommand("t1.bco=" + backGroundColor);
  displayCommand("t1.pco=" + fontColor);
  displayCommand("t1.txt=\"" + TicketContent + "\"");
  displayCommand("t0.bco=" + backGroundColor);
  displayCommand("t0.pco=" + TranspartentColor);
}
void getGpsInfo()
{
 
  if (gps.location.isValid())
  {
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6);
    speed = String(gps.speed.mph());
    noOfStatilte = String(gps.satellites.value());
    // Serial.println("speed: " + speed + " mph");
    // Serial.println("sat: " + noOfStatilte);
   
    if (deviceTicketTelematricHybridMode == 2)
    {
      if (gps.speed.isValid() && gps.speed.mph() > requiredGPSSpeedThreshold && gps.satellites.value() >= requiredGPSsatellitesThreshold)
      {
        if (!isDeviceInGPSTelemetryMode)
        {
          deviceTicketTelematriceMode = true;
          
          // Serial.println("Device in (Hybrid) GPS telemetry mode" + String(gps.speed.mph()) + " mph");
          isDeviceInGPSTelemetryMode = true;
          DisplayHomepage(displayLine1, displayLine2);
          
        }
      }
      else
      {
        if (isDeviceInGPSTelemetryMode)
        {
          deviceTicketTelematriceMode = false;
          // Serial.println("Device in (Hybrid) ticket validation mode" + String(gps.speed.mph()) + " mph");
          isDeviceInGPSTelemetryMode = false;
          DisplayHomepage(displayLine1, displayLine2);

        }
      }
    }
    else if (deviceTicketTelematricHybridMode == 1)
    {
      deviceTicketTelematriceMode = true;
      
      // Serial.println("Device in (Static) GPS telemetry mode" + String(gps.speed.mph()) + " mph");
    }
    else if (deviceTicketTelematricHybridMode == 0)
    {
      deviceTicketTelematriceMode = false;
      // Serial.println("Device in (Static) Ticket telemetry mode" + String(gps.speed.mph()) + " mph");
    }
  }
  else
  {
    // Serial.println("Location: Not Available");
  }

  // Serial.print("Date: ");
  if (gps.date.isValid())
  {
    dateString = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
    // Serial.println(dateString);
  }
  else
  {
    dateString = "Not Available";
    // Serial.println(dateString);
  }

  // Serial.print("Time: ");
  if (gps.time.isValid())
  {
    String hourString = (gps.time.hour() < 10) ? "0" + String(gps.time.hour()) : String(gps.time.hour());
    String minuteString = (gps.time.minute() < 10) ? "0" + String(gps.time.minute()) : String(gps.time.minute());
    String secondString = (gps.time.second() < 10) ? "0" + String(gps.time.second()) : String(gps.time.second());
    String centisecondString = (gps.time.centisecond() < 10) ? "0" + String(gps.time.centisecond()) : String(gps.time.centisecond());

    timeString = hourString + ":" + minuteString + ":" + secondString + "." + centisecondString;
    // Serial.println(timeString);
  }
  else
  {
    timeString = "Not Available";
    // Serial.println(timeString);
  }
  // Serial.println();
  delay(1000);
}
void SOSflash(int duration)
{
  digitalWrite(BUZZ, HIGH);
  delay(duration);
  digitalWrite(BUZZ, LOW);
  delay(duration);
}
void playSOSSound()
{
  int durations[] = {200, 200, 200, 300, 500, 500, 500, 200, 200, 200};
  int numFlashes = sizeof(durations) / sizeof(durations[0]);
  for (int i = 0; i < numFlashes; i++)
  {
    SOSflash(durations[i]);
  }
  delay(1000);
}
void bleNrf52Init()
{
  const int timeout_ms = 5000; // 5 seconds timeout
  String response;

  // Execute "BLERESET" command
  nrf52.println("BLERESET");

  // Wait for a response with timeout
  long startTime = millis();
  while ((millis() - startTime) < timeout_ms)
  {
    if (nrf52.available())
    {
      response += (char)nrf52.read();
    }
    // Check for the expected response to indicate successful reset
    if (response.indexOf("BLERESET: OK") != -1)
    {
      Serial.println("BLE module reset successfully.");
      break; // Exit the loop if the expected response is received
    }
  }

  // If there's no response or the expected response is not received, print a message and return
  if (response.length() == 0 || response.indexOf("BLERESET: OK") == -1)
  {
    Serial.println("Failed to reset BLE module.");
    return;
  }

  // Clear the response for the next command
  response = "";
  delay(4000);
  // Execute "BLEVER" command
  nrf52.println("BLEVER");

  // Wait for a response with timeout
  startTime = millis();
  while ((millis() - startTime) < timeout_ms)
  {
    if (nrf52.available())
    {
      response += (char)nrf52.read();
    }
  }

  // If there's no response, print a message and return
  if (response.length() == 0)
  {
    Serial.println("No response from NRF52 for BLEVER");
    return;
  }

  // Extract and store the BLE version
  int versionStartIndex = response.indexOf("BLEVER: ") + 7; // Skip "BLEVER:"
  int versionEndIndex = response.indexOf('\n', versionStartIndex);
  bleVersion = response.substring(versionStartIndex, versionEndIndex);
  bleVersion.trim();
  Serial.println("BLE Version: " + bleVersion);

  // Clear the response for the next command
  response = "";

  // Execute "BLEMAC" command
  nrf52.println("BLEMAC");

  // Wait for a response with timeout
  startTime = millis();
  while ((millis() - startTime) < timeout_ms)
  {
    if (nrf52.available())
    {
      response += (char)nrf52.read();
    }
  }

  // If there's no response, print a message and return
  if (response.length() == 0)
  {
    Serial.println("No response from NRF52 for BLEMAC");
    return;
  }

  // Extract and store the BLE MAC address
  int macStartIndex = response.indexOf("BLEMAC: ") + 7; // Skip "BLEMAC:"
  int macEndIndex = response.indexOf('\n', macStartIndex);
  bleMacAddress = response.substring(macStartIndex, macEndIndex);
  bleMacAddress.trim();
  Serial.println("BLE MAC Address: " + bleMacAddress);
  bleAdStatus = true;
}

void playTicketSoundAndLight(int ticketValidity)
{
  // Turn off all lights and buzzer
  lightUpLED(0, 0, 0, 0);

  if (ticketValidity == 1)
  {
    // Valid ticket: Green light and short beep

    if (buzzerEnable)
    {
      lightUpLED(0, 0, 2, 0);
      fileIndex = 6;
      playSpeaker = true;
      delay(validTiceketDelay);
    }
    else
    {
      lightUpLED(0, 0, 2, 0);
      delay(validTiceketDelay);
    }
  }
  else if (ticketValidity == 0)
  {
    // Invalid ticket: Red light and three short beeps
      if (buzzerEnable && useSpeakerSound || !buzzerEnable)
      {
          
         lightUpLED(1, 0, 0, 0);
        delay(invalidTicketDelay);
        lightUpLED(0, 0, 0, 0);
       // delay(invalidTicketDelay);
      }
     else 
      {
       lightUpLED(1, 0, 0, 0);
       fileIndex = 8;
       playSpeaker = true;
        delay(invalidTicketDelay);
        
      }
    
  }
  else if (ticketValidity == 3)
  {
    // Yellow light: Single long beep
    if (buzzerEnable)
    {
      lightUpLED(0, 4, 0, 0);
      fileIndex = 6;
      playSpeaker = true;
      delay(validTiceketDelay);
    }
    else
    {
      lightUpLED(0, 4, 0, 0);
      delay(validTiceketDelay);
    }
  }
  else if (ticketValidity == 5)
  {
    // Yellow light: for notification
    for (int i = 0; i < 2; i++)
    {
      lightUpLED(0, 4, 0, 0);
      if (buzzerEnable)
      {
        delay(100);
        lightUpLED(0, 0, 0, 0);
      }
      else
      {
        delay(100);
      }
    }
  }
  if (ticketValidity == 4)
  {
    // Stop sound
    if (buzzerEnable)
    {
      lightUpLED(1, 0, 0, 0);
      fileIndex = 6;
      playSpeaker = true;
      delay(stopTicketDelay);
    }
    else
    {
      lightUpLED(1, 0, 0, 0);
      delay(stopTicketDelay);
    }
  }
  // Turn off all lights and buzzer
  lightUpLED(0, 0, 0, 0);
}


float checkAudioversion(String filename) {
  File wavFile = SD.open(filename.c_str(), FILE_READ);
  float storedVersion = 0.0f;

  if (wavFile && wavFile.size() >= sizeof(float)) {
    wavFile.seek(wavFile.size() - sizeof(float));  
    wavFile.read((uint8_t*)&storedVersion, sizeof(float));
    wavFile.close();

    Serial.print(F("Retrieved version from "));
    Serial.print(filename);
    Serial.print(": ");
    Serial.println(storedVersion, 1);  

    if (storedVersion == 0.0f) {
        Serial.println("Audio update is done for first time");
        
         
    } else {
        Serial.println("Stored version is available");
    }
  } else {
    Serial.print(F("Failed to open file: "));
    Serial.println(filename);
  }

  return storedVersion;
}


bool waitForResponses(const char *expected, unsigned long timeout = 10000) {
  unsigned long start = millis();
  String line;
  while (millis() - start < timeout) {
    while (espSerial.available()) {
      char c = espSerial.read();
      Serial.write(c);
      line += c;
      if (line.indexOf(expected) >= 0) {
        return true;
      }
    }
  }
  return false;
}

void sendAT(const char *cmd) {
  espSerial.println(cmd);
  Serial.print(F("> "));
  Serial.println(cmd);
}

  
  void parseApiResponse(const String &apiResponse)
{  

 DynamicJsonDocument jsonDocument(8192); // Adjust size if needed
  DeserializationError jsonError = deserializeJson(jsonDocument, apiResponse);
   
  Serial.println(jsonError.f_str());

  if (jsonError)
  {
    Serial.println("Failed to parse JSON: " + String(jsonError.c_str()));
    apiError = String(jsonError.c_str());
    bleAdvertisebegin();
    displayCommand(invaildDataDisplayPage);
    deviceInfoPageStatus = true;
    while (true) { } // Halt
    return;
  }
  
  JsonObject root = jsonDocument["data"];



  // Check if root is valid
  if (root.isNull())
  {
    apiError = "NULL res";
    Serial.println("JSON data is null for all keys.");
    bleAdvertisebegin();
    displayCommand(deviceUnauthorizedDisplayPage);
    while (true) { } // Halt
    return;
  }

  // ---------------- Display Brightness ----------------
  if (root["Displaybrightness"] != nullptr) {
    JsonObject brightnessObj = root["Displaybrightness"];

    auto isValidString = [](JsonVariant val) {
      return !val.isNull() && String(val) != "" && String(val) != "NA";
    };

    bool levelFound = isValidString(brightnessObj["displaybrightnesslevel"]);
    bool conditionFound = isValidString(brightnessObj["ticketpagedisplaybrightnesscondition"]);

    // Defaults
    displayBrightnessLevel = 100;
    displayBrightnessCondition = true;
    displaybrigtnesserror = "";
    TicketDisplaybrightnesserror = "";

    if (levelFound && conditionFound) {
      displayBrightnessLevel = String(brightnessObj["displaybrightnesslevel"]).toInt();
      displaybrigtnesserror = String(displayBrightnessLevel);
      if (displayBrightnessLevel < 0 || displayBrightnessLevel > 100) {
        displayBrightnessLevel = 100;
        Serial.println("Brightness level out of range, set to 100.");
        displaybrigtnesserror = "Brightness level out of range";
      }

      String condStr = brightnessObj["ticketpagedisplaybrightnesscondition"];
      TicketDisplaybrightnesserror = condStr ? "true" : "false";
      if (condStr == "true" || condStr == "1") {
        displayBrightnessCondition = true;
      } else if (condStr == "false" || condStr == "0") {
        displayBrightnessCondition = false;
      } else {
        displayBrightnessCondition = true;
        Serial.println("Ticket display condition invalid, set to true.");
        TicketDisplaybrightnesserror = "Ticket display condition invalid";
      }
    } else {
      displayBrightnessLevel = 100;
      displayBrightnessCondition = true;
      Serial.println("Brightness or condition missing/NA. Defaults applied.");
      displaybrigtnesserror = "Brightness or condition missing/NA";
      TicketDisplaybrightnesserror = "Brightness or condition missing/NA";
    }
  } else {
    displayBrightnessLevel = 100;
    displayBrightnessCondition = true;
    Serial.println("Displaybrightness object not found. Defaults applied.");
    displaybrigtnesserror = "Display brightness object not found";
    TicketDisplaybrightnesserror = "Display brightness object not found";
  }

  // Apply brightness
  displaybrightesscontrol(displayBrightnessLevel);

  // ---------------- BLE ----------------
  ibeaconMajor = root["ble"]["ibeaconMajor"];
  ibeaconMinor = root["ble"]["ibeaconMinor"];
  String nrf52BleName = root["ble"]["blename"].as<String>();
  blename = nrf52BleName;
  bleTxPower = root["ble"]["bleTxPower"];
  bleDfuMode = root["ble"]["bleDfuMode"];
  bleIbeaconMode = root["ble"]["bleIbeaconMode"];
  bleConnectMode = root["ble"]["bleConnectMode"];
  requiredBleScanRssi = root["ble"]["bleCardScanRssi"];
  bleScanMode = root["ble"]["bleScanMode"];
  if (root["ble"]["rssiat1m"].is<const char*>()) {
  const char* val = root["ble"]["rssiat1m"];
  if (strcmp(val, "NA") != 0 && strlen(val) > 0) {
    rssiat1m = atoi(val);   
  }
} 
else if (root["ble"]["rssiat1m"].is<int>()) {
  rssiat1m = root["ble"]["rssiat1m"].as<int>();  
}

  // ---------------- MQTT ----------------
  mqttLogDelay = root["mqtt"]["mqttLogDelay"];
  wifiLogPublishInterval = root["mqtt"]["wifiLogPublishInterval"];
  gsmLogPublishInterval = root["mqtt"]["gsmLogPublishInterval"];
  totalDeviceLog = root["mqtt"]["totalDeviceLog"];
  sendOnlyGpsLog = root["mqtt"]["sendOnlyGpsLog"];
  deviceTopic = root["mqtt"]["deviceTopic"].as<String>();
  deviceReactTopic = root["mqtt"]["deviceReactTopic"].as<String>();
  deviceLogTopic = root["mqtt"]["deviceLogTopic"].as<String>();
  deviceBleCardTopic = root["mqtt"]["bleCardTopic"].as<String>();

  // ---------------- Validation ----------------
  validTiceketDelay = root["validation"]["validTiceketDelay"];
  invalidTicketDelay = root["validation"]["invalidTicketDelay"];
  validSpecialTiceketDelay = root["validation"]["validSpecialTiceketDelay"];
  multipleTicketDelay = root["validation"]["multipleTicketDelay"];
  specialTicketType = root["validation"]["specialTicketType"];
  startDeviceMode = root["validation"]["startDeviceMode"];

  // startDeviceMode 0-3: mqtt.zig-web.com | 4-7: mq.zig-web.com
  // 0/4=Disabled, 1/5=NFC Only, 2/6=QR Only, 3/7=NFC+QR
  if (startDeviceMode >= 4)
  {
    mqttServer = mqttServerAlt; // Switch to mq.zig-web.com
    startDeviceMode = startDeviceMode - 4; // Normalize to 0-3 for input device logic
    Serial.println("MQTT Server: mq.zig-web.com (mode 4-7)");
  }
  else
  {
    Serial.println("MQTT Server: mqtt.zig-web.com (mode 0-3)");
  }

  buzzerEnable = root["validation"]["buzzerEnable"];
  displayLastTicket = root["validation"]["displayLastTicketScreen"];
  enableMultipleLights = root["validation"]["enableMultipleLights"];
  String homepageHeadingStatus1Str = root["validation"]["homepageHeadingStatus1"];
  String homepageHeadingStatus2Str = root["validation"]["homepageHeadingStatus2"];
  String homepageHeadingStatus3Str = root["validation"]["homepageHeadingStatus3"];
  String homepageHeadingStatus4Str = root["validation"]["homepageHeadingStatus4"];
  homepageHeadingStatus1 = homepageHeadingStatus1Str;
  homepageHeadingStatus2 = homepageHeadingStatus2Str;
  homepageHeadingStatus3 = homepageHeadingStatus3Str;
  homepageHeadingStatus4 = homepageHeadingStatus4Str;
  enableToggleDisplayLines = root["validation"]["enableToggleDisplayLines"];

  // ---------------- Firmware ----------------
  firmwareUrl = root["frimware"]["firmwareUrl"];
  firmwareVersion = root["frimware"]["firmwareVersion"];

  // ---------------- GPS ----------------
  requiredGPSSpeedThreshold = root["gps"]["requiredSpeedLimit"];
  requiredGPSsatellitesThreshold = root["gps"]["requiredNosatellites"];
  deviceTicketTelematricHybridMode = root["gps"]["deviceTicketTelematricHybridMode"];
  sendOutCardDataCount = root["gps"]["sendOutCardDataCount"];

  // ---------------- TOF ----------------
  TofModes = root["tof"]["tofMode"];
  String homepageHeadingStatus5Str = root["tof"]["benchmarkDisplayTittle"];
  advertiseInterval = root["tof"]["advertiseInterval"];
  tofSensorThreshold = root["tof"]["tofSensorThreshold"];

  // ---------------- Wifi ----------------
  EepromWifiSsid = root["wifi"]["primaryWifi"]["ssid"].as<String>();
  EepromWifiPass = root["wifi"]["primaryWifi"]["password"].as<String>();
  Serial.println("WiFi credentials loaded from API");
  Serial.print("SSID: "); Serial.println(EepromWifiSsid);
  Serial.print("Password: "); Serial.println(EepromWifiPass);

  // ---------------- Speaker ----------------
  if (root["speaker"] == nullptr) {
    Serial.println("Speaker Config not found");
  } else {
    useSpeakerSound = root["speaker"]["useSpeaker"];
    gainValue = root["speaker"]["speakerGain"];
    mixer1.gain(0, gainValue);
    mixer2.gain(0, gainValue);
    updatedSound = root["speaker"]["updatedSound"];

    // Valid audio
    updateforvalid = root["speaker"]["validEntryAudio"]["updatevalidurl"].as<bool>();
    validaudioupdate = root["speaker"]["validEntryAudio"]["audioValidVersion"].as<float>();
    validUrl = root["speaker"]["validEntryAudio"]["validurl"].as<String>();
    validSize = root["speaker"]["validEntryAudio"]["size"].as<long>();
    validFilename = root["speaker"]["validEntryAudio"]["filename"].as<String>();

    // Invalid audio
    updateforinvalid = root["speaker"]["invalidEntryAudio"]["updateinvalidurl"].as<bool>();
    invalidaudioupdate = root["speaker"]["invalidEntryAudio"]["audioinValidVersion"].as<float>();
    invalidUrl = root["speaker"]["invalidEntryAudio"]["invalidurl"].as<String>();
    invalidSize = root["speaker"]["invalidEntryAudio"]["size"].as<long>();
    invalidFilename = root["speaker"]["invalidEntryAudio"]["filename"].as<String>();
  }
saveWifiConfig(EepromWifiSsid, EepromWifiPass);

  Serial.println("//________________Api Ble data________________//");
  Serial.print("ble_major: ");
  Serial.println(ibeaconMajor);
  Serial.print("ble_minor: ");
  Serial.println(ibeaconMinor);
  Serial.print("ble_name: ");
  Serial.println(blename);
  Serial.print("ble_tx_power: ");
  Serial.println(bleTxPower);
  Serial.print("ble_dfu_mode: ");
  Serial.println(bleDfuMode);
  Serial.print("ble_ibeacon_mode: ");
  Serial.println(bleIbeaconMode);
  Serial.print("ble_connect_mode: ");
  Serial.println(bleConnectMode);
  Serial.print("bleCardScanRssi: ");
  Serial.println(requiredBleScanRssi);
  Serial.print("bleScanMode: ");
  Serial.println(bleScanMode);
  Serial.print("Rssi_At_1m: ");
  Serial.println(rssiat1m);
  Serial.println("//________________Api mqtt data________________//");
  Serial.print("mqtt_log_delay: ");
  Serial.println(mqttLogDelay);
  Serial.print("wifi_log_publish_interval: ");
  Serial.println(wifiLogPublishInterval);
  Serial.print("gsm_log_publish_interval: ");
  Serial.println(gsmLogPublishInterval);
  Serial.print("total_device_log: ");
  Serial.println(totalDeviceLog);
  Serial.print("send_only_gps_log: ");
  Serial.println(sendOnlyGpsLog);
  Serial.print("device_topic: ");
  Serial.println(deviceTopic);
  Serial.print("device_react_topic: ");
  Serial.println(deviceReactTopic);
  Serial.print("device_log_topic: ");
  Serial.println(deviceLogTopic);
  Serial.print("device_bleCard_topic: ");
  Serial.println(deviceBleCardTopic);
  Serial.println("//________________Api validation data________________//");
  Serial.print("valid_ticket_delay: ");
  Serial.println(validTiceketDelay);
  Serial.print("invalid_ticket_delay: ");
  Serial.println(invalidTicketDelay);
  Serial.print("valid_special_ticket_delay: ");
  Serial.println(validSpecialTiceketDelay);
  Serial.print("multiple_ticket_delay: ");
  Serial.println(multipleTicketDelay);
  Serial.print("special_ticket_type: ");
  Serial.println(specialTicketType);
  Serial.print("buzzer_enable: ");
  Serial.println(buzzerEnable);
  Serial.print("homepageHeadingStatus1: ");
  Serial.println(homepageHeadingStatus1);
  Serial.print("homepageHeadingStatus2: ");
  Serial.println(homepageHeadingStatus2);
  Serial.print("homepageHeadingStatus3: ");
  Serial.println(homepageHeadingStatus3);
  Serial.print("homepageHeadingStatus4: ");
  Serial.println(homepageHeadingStatus4);
  Serial.print("enableToggleDisplayLines: ");
  Serial.println(enableToggleDisplayLines);
  Serial.print("displaylastTicketscreen: ");
   Serial.println(displayLastTicket);
  Serial.println("//________________Api firmware data________________//");
  Serial.print("firmwareUrl: ");
  Serial.println(firmwareUrl);
  Serial.print("firmware Version: ");
  Serial.println(firmwareVersion);
  if (wifiConnected)
  {
    if (!gsmSimFound)
    {
      versionCheck();
    }
  }
  if(eg25moduleconnected)
  {
    versionCheck();
  }
  

  Serial.println("//________________Api gps data________________//");
  Serial.print("deviceTicketTelematricHybridMode: ");
  Serial.println(deviceTicketTelematricHybridMode);
  Serial.print("requiredNosatellites: ");
  Serial.println(requiredGPSsatellitesThreshold);
  Serial.print("requiredSpeedLimit: ");
  Serial.println(requiredGPSSpeedThreshold);
  Serial.print("sendOutCardDataCount: ");
  Serial.println(sendOutCardDataCount);
  Serial.println("//________________Api Wifi data________________//");
  Serial.print("primaryWifi: ");
  Serial.println(EepromWifiSsid);
  Serial.print("primaryPassword: ");
  Serial.println(EepromWifiPass);
  Serial.println("//________________TOF Data________________//");
  Serial.print("TOF Mode: ");
  Serial.println(TofModes);
  Serial.print("homepageHeadingStatus5Str: ");
  Serial.println(homepageHeadingStatus5Str);
  Serial.print("advertiseInterval: ");
  Serial.println(advertiseInterval);
  Serial.print("tofSensorThreshold: ");
  Serial.println(tofSensorThreshold);
  Serial.println("//________________Speaker Data________________//");
  Serial.print("GainMode Mode: ");
  Serial.println(gainValue);
   Serial.print(F("updatedSound: ")); Serial.println(updatedSound ? "true" : "false");
   Serial.print(F(" AudioVersionforvalid:"));Serial.println(validaudioupdate);
   Serial.print(F(" AudioVersionforinvalid:"));Serial.println(invalidaudioupdate);
   Serial.println(F("Valid Entry Audio:"));
  Serial.print(F("  updatevalidurl: ")); Serial.println(updateforvalid ? "true" : "false");
  Serial.print(F("  URL: ")); Serial.println(validUrl);
  Serial.print(F("  Size: ")); Serial.println(validSize);
  Serial.print(F("  Filename: ")); Serial.println(validFilename);
  Serial.println(F("Invalid Entry Audio:"));
  Serial.print(F("  updateinvalidurl: ")); Serial.println(updateforinvalid ? "true" : "false");
  Serial.print(F("  URL: ")); Serial.println(invalidUrl);
  Serial.print(F("  Size: ")); Serial.println(invalidSize);
  Serial.print(F("  Filename: ")); Serial.println(invalidFilename);


if(!SD.begin(BUILTIN_SDCARD))
{
  Serial.println("Sd card is missed");
}
else
{
if(wifiConnected)
{
   

  

  
 validStoredVersion = checkAudioversion(validFilename);
 invalidStoredVersion = checkAudioversion(invalidFilename);

   if (updatedSound && updateforvalid )
   {
    
  if(validaudioupdate > validStoredVersion)
  {
    urldata = validUrl;
    Sizedataforbytes = validSize;
    filename = validFilename;
    isvalid = true;
    Httpcget();
  }
  }
//checkAudioversion(invalidFilename);

  if (updatedSound && updateforinvalid)
  {
    if(invalidaudioupdate > invalidStoredVersion)
  {
    urldata = invalidUrl;
    Sizedataforbytes = invalidSize;
    filename = invalidFilename;
    isinvalid = true;
    Httpcget();
    
  }

  }  
 

}
else
{
  validStoredVersion = checkAudioversion(validFilename);
  invalidStoredVersion = checkAudioversion(invalidFilename); 
  Serial.println("connected with 4G module");
}
}
  Serial.print("advertiseInterval: ");
  saveBLEConfig(ibeaconMajor, ibeaconMinor, bleTxPower, rssiat1m);
  bleNrf52Command = bleStartCommand + "#" + blename + "#" + String(bleTxPower) + "#" +
                    (bleDfuMode ? "true" : "false") + "#" + String(ibeaconMajor) + "#" +
                    String(ibeaconMinor) + "#" + (bleIbeaconMode ? "true" : "false") + "#" + (bleScanMode ? "true" : "false") + "#" +
                    (bleConnectMode ? "true" : "false")+ "#"+String(rssiat1m);
  Serial.println("API ok");
  wifiApiDataState = true;
  handleTofModes(TofModes);
}

void displayAllConnectedPage()
{
  displayCommand(zigNetworkFoundDisplayPage);
  if (useSpeakerSound)
  {
    // playWavFile();
    fileIndex = 3;
    playSpeaker = true;
  }
  else
  {
    lightUpLED(2, 2, 2, 1);
    delay(3000);
    wdt.feed();
    lightUpLED(0, 0, 0, 0);
  }

  deviceHomePageStatus = true;
  if (isEthernetConnected)
  {
    displayLine1 = "ETHERNET";
    displayLine2 = "ETHERNET";
    homepageHeadingStatus3 = "ETHERNET";
    deviceBusyStatus = 1;
    DisplayHomepage(displayLine1, displayLine2);
  }
  else
  {
    if (enableWifiGsm)
    {
      displayLine1 = connectedSSID;
      displayLine2 = ssid;
      deviceBusyStatus = 1;
      DisplayHomepage(displayLine1, displayLine2);
    }
    else
    {
      displayLine1 = GsmNetworkTypeString;
      displayLine2 = GsmOperatorName;
      deviceBusyStatus = 1;
      DisplayHomepage(displayLine1, displayLine2);
    }
  }
}

String sendATCommand(String command) {
  Serial1.println(command);
  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {
    wdt.feed();
    delay(100);
    while (Serial1.available()) {
      response += (char)Serial1.read();
    }
  }

  response.trim();

  if (response.length() == 0) {
    return "TIMEOUT";
  }

  return response;
}


void displayError(int errorCode) {
   displayCommand("page 18");                  // Switch to error page
  displayCommand("t0.txt=\"ERROR\"");                 // Fixed label
  displayCommand("t1.txt=\"" + String(errorCode) + "\"");
   // Show error number
}

void checkhexdatafromsdcard(int expectedLength, int bytesWritten) {

  if(timeouterrorhappen)
  {
  apiError = "Timeout reached.Ending download";
         displayError(206);
  }
   else{
  File verifyFile = SD.open(HEX_FILE_NAME, FILE_READ);
  if (!verifyFile) {
    Serial.println("Failed to open hex for verification!");
    apiError = "Failed to open hex for verification!";
    displayError(207);
    return;
  }

  String firstLine = verifyFile.readStringUntil('\n');
  firstLine.trim();

  // Read line-by-line to find the last valid non-empty line
  String lastLine = "";
  String lineBuffer = "";

  while (verifyFile.available()) {
    char c = verifyFile.read();

    if (c == '\n') {
      if (lineBuffer.length() > 0) {
        lastLine = lineBuffer;
        lineBuffer = "";
      }
    } else {
      lineBuffer += c;
    }
  }

  // If file ends without a newline
  if (lineBuffer.length() > 0) {
    lastLine = lineBuffer;
  }

  lastLine.trim();
  verifyFile.close();

  // Log info for debugging
  Serial.println("Verifying HEX file...");
  Serial.print("First line: ");
  Serial.println(firstLine);
  Serial.print("Last line: ");
  Serial.println(lastLine);
  Serial.print("Expected bytes: ");
  Serial.println(expectedLength);
  Serial.print("Actual written bytes: ");
  Serial.println(bytesWritten);

  // Final verification conditions
  if (firstLine.startsWith(":0200000460009A") &&
      lastLine == ":00000001FF" &&
      bytesWritten == expectedLength) {
      
    Serial.println("HEX file verification passed. Proceeding with firmware update...");


    Stream* serialeg25 = &Serial;

    if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0) {
      serialeg25->printf("Unable to create buffer\n");
      apiError ="unable to create buffer";
      displayError(216);
      serialeg25->flush();
      while (1);  // Halt execution
    }

    serialeg25->printf("Created buffer = %luK %s (%08lX - %08lX)\n",
        buffer_size / 1024,
        IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
        buffer_addr, buffer_addr + buffer_size);

    serialeg25->println("SD initialization OK");

    hexFile = SD.open(HEX_FILE_NAME, FILE_READ);
    if (!hexFile) {
      serialeg25->println("SD file open failed");
      apiError ="SD file open failed";
      displayError(213);
      return;
    }
    serialeg25->println("SD file open OK");
     DisplayInfo("Installing Update ...!", "Please wait", codeVersion);
    update_firmware(&hexFile, serialeg25, buffer_addr, buffer_size);

    serialeg25->printf("Erase FLASH buffer / free RAM buffer...\n");
    firmware_buffer_free(buffer_addr, buffer_size);
    serialeg25->flush();

    REBOOT;  // Restart device after update
    eg25moduleconnected = false;

  } else {
    Serial.println("HEX file verification failed. Firmware update aborted.");
     eg25moduleconnected = false;
     apiError ="HEX file verification failed";
     displayError(208);
  }
}
}

void downloadFirmware(int checkbytesfordata) {
  Serial.println("Sending HTTP READ...");
  Serial1.println("AT+QHTTPREAD");

  // Wait for initial response (+QHTTPREAD or CONNECT)
  while (true) {
    wdt.feed();
    if (Serial1.available()) {
      String line = Serial1.readStringUntil('\n');
      line.trim();
      Serial.println(line);

      if (line.startsWith("+QHTTPREAD:") || line == "CONNECT") {
        break;
      }
      if (line.startsWith("ERROR") || line.startsWith("+CME ERROR")) {
        Serial.println("HTTP read failed.");
        apiError = "HTTP READ FAILED";
         displayError(109);
      }
    }
  }

  // Open file to write only payload (not headers)
  File hexFile = SD.open(HEX_FILE_NAME, FILE_WRITE);
  if (!hexFile) {
    Serial.println("Failed to open hex for writing!");
    apiError = "Failed to open hex for writing!";
     displayError(205);
  }
else
{
  Serial.println("Starting data download...");

  bool headerEnded = false;
  String headerBuffer = "";
  String dataBuffer = "";
  unsigned long lastByteTime = millis();
  int bytesWritten = 0;
  int checkdatavalue = checkbytesfordata;

  while (true) {
    wdt.feed();
    if (Serial1.available()) {
      char c = Serial1.read();
      lastByteTime = millis();

      if (!headerEnded) {
        headerBuffer += c;

        // Check for end of headers
        if (headerBuffer.endsWith("\r\n\r\n")) {
          headerEnded = true;
          Serial.println("Headers skipped. Starting to write payload...");
        }

        continue; // skip writing headers
      } else {
        dataBuffer += c;

        // Check for trailing OK (end of payload)
        int okIndex = dataBuffer.indexOf("\r\nOK\r\n");
        if (okIndex != -1) {
          // Write only up to the OK
          if (okIndex > 0) {
            hexFile.write((const uint8_t*)dataBuffer.substring(0, okIndex).c_str(), okIndex);
            bytesWritten += okIndex;
          }
          break;
        }

        // Write in chunks
        while (dataBuffer.length() >= 256) {
          hexFile.write((const uint8_t*)dataBuffer.c_str(), dataBuffer.length());
          bytesWritten += dataBuffer.length();
          dataBuffer = "";
         // hexFile.write((const uint8_t*)dataBuffer.c_str(), 256);
          // bytesWritten += 256;
          // dataBuffer = dataBuffer.substring(256);
        }
      }
    } else {
      if (millis() - lastByteTime > 50000) {
        wdt.feed();
        Serial.println("Timeout reached. Ending download.");
        timeouterrorhappen = true;
        break;
      }
    }
  }

  hexFile.flush();
  hexFile.close();

  Serial.println("Download complete.");
  Serial.print("Total bytes written: ");
  Serial.println(bytesWritten);
checkhexdatafromsdcard(checkdatavalue,bytesWritten);
}


}


 String deviceApiUrl1 = firmwareUrl; 

void modemSetup() {

   
response = sendATCommand("AT+QHTTPCFG=\"closed/ind\",0");
  if (response.indexOf("OK") != -1) {
    Serial.println("Disabled HTTPS closed indication successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while disabling HTTPS closed indication.");
    apiError = "Timeout disabling HTTPS closed indication";
    displayError(119);
  } else {
    Serial.println("Failed to disable HTTPS closed indication: " + response);
    apiError = "Failed to disable HTTPS closed indication";
     displayError(101);
  }

  response = sendATCommand("AT+QHTTPSTOP");
  if (response.indexOf("OK") != -1) {
    Serial.println("Stopped previous HTTPS session successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while stopping HTTPS session.");
    apiError = "Timeout stopping HTTPS session";
    displayError(119);
  } else {
    Serial.println("Failed to stop HTTPS session: " + response);
    apiError = "Failed to stop HTTPS session";
      displayError(102);
  }

  response = sendATCommand("AT+QHTTPCFG=\"closed/ind\",1");
  if (response.indexOf("OK") != -1) {
    Serial.println("Enabled HTTPS closed indication successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while enabling HTTPS closed indication.");
    apiError = "Timeout enabling HTTPS closed indication";
    displayError(119);
  } else {
    Serial.println("Failed to enable HTTPS closed indication: " + response);
    apiError = "Failed to enable HTTPS closed indication";
     displayError(103);
  }

  response = sendATCommand("AT+QHTTPCFG=\"contextid\",1");
  if (response.indexOf("OK") != -1) {
    Serial.println("Set HTTPS context ID successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while setting HTTPS context ID.");
    apiError = "Timeout setting HTTPS context ID";
    displayError(119);
  } else {
    Serial.println("Failed to set HTTPS context ID: " + response);
    apiError = "Failed to set HTTPS context ID";
     displayError(104);
  }

  response = sendATCommand("AT+QHTTPCFG=\"sslctxid\",1");
  if (response.indexOf("OK") != -1) {
    Serial.println("Set HTTPS SSL context ID successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while setting HTTPS SSL context ID.");
    apiError = "Timeout setting HTTPS SSL context ID"; 
    displayError(119);
  } else {
    Serial.println("Failed to set HTTPS SSL context ID: " + response);
    apiError = "Failed to set HTTPS SSL context ID";
     displayError(105);
  }

  response = sendATCommand("AT+QSSLCFG=\"sslversion\",1,4");
  if (response.indexOf("OK") != -1) {
    Serial.println("Configured SSL version (TLS 1.2) successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while configuring SSL version.");
    apiError = "Timeout configuring SSL version";
    displayError(119);
  } else {
    Serial.println("Failed to configure SSL version: " + response);
    apiError = "Failed to configure SSL version";
     displayError(106);
  }

  response = sendATCommand("AT+QSSLCFG=\"seclevel\",1,0");
  if (response.indexOf("OK") != -1) {
    Serial.println("Configured SSL security level successfully.");
  } else if (response == "TIMEOUT") {
    Serial.println("Timeout while configuring SSL security level.");
    apiError = "Timeout configuring SSL security level";
    displayError(119);
  } else {
    Serial.println("Failed to configure SSL security level: " + response);
    apiError = "Failed to configure SSL security level";
     displayError(107);
  }


// === Set URL ===
 String urlSize = String(strlen(firmwareUrl));
  String setUrlLength1 = sendATCommand("AT+QHTTPURL=" + urlSize);
  Serial.println(setUrlLength1);
    if (setUrlLength1.indexOf("CONNECT") != -1) {
      Serial.println("URL length set successfully.");
    } else {
      Serial.println("setting URL length,failed");
      apiError ="setting URL length,failed";
      displayError(108);
    }

   
    String url1 = firmwareUrl; // Example URL
    String setUrl1 = sendATCommand(url1);
    if (setUrl1.indexOf("OK") != -1) {
      Serial.println("URL set successfully.");
    } else {
      Serial.println("Error setting URL.");
      apiError = "Error setting URL.";
    }

    String getRequest1 = sendATCommand("AT+QHTTPGET");
    delay(5000);
    Serial.println(getRequest1);
    if(getRequest1.startsWith("+QHTTPGET:"))
    {
       Serial.println("GET request sent successfully.");
    }
    else{
       Serial.println("Error sending GET request.");
       
    }


    int expectedBytes = -1;

// Extract from +QHTTPGET: 0,200,668314
int startIndex = getRequest1.lastIndexOf("+QHTTPGET:");
if (startIndex != -1) {
  int commaIndex1 = getRequest1.indexOf(',', startIndex);
  int commaIndex2 = getRequest1.indexOf(',', commaIndex1 + 1);
  if (commaIndex2 != -1) {
    String byteString = getRequest1.substring(commaIndex2 + 1);
    byteString.trim();
    expectedBytes = byteString.toInt();
  }
}

if (expectedBytes == -1) {
  Serial.println("Failed to parse expected byte count.");
  apiError="Failed to parse expected byte count";
   displayError(111);
} else {
  Serial.print("Expected bytes to receive: ");
  Serial.println(expectedBytes);
   DisplayInfo("Downloading update ...!", "Please wait", codeVersion);
  downloadFirmware(expectedBytes);
}


// === Proceed to firmware download ===

//downloadFirmware();

 

}

void versionCheck()
{
  if (firmwareVersion < codeVersion.toFloat())
  {
    Serial.println("No updates available");
  }
  else if (firmwareVersion == codeVersion.toFloat())
  {
    Serial.println("Already in current version - no updates available");
  }
  else
  {
    if(eg25moduleconnected)
    {
      DisplayInfo("Update available ...!", "Please wait", codeVersion);
         modemSetup(); 
    }
    else{
    Serial.println("New firmware update available");
    // hexInitDownload(firmwareUrl, HEX_FILE_NAME);
    verifyHexData(firmwareUrl, HEX_FILE_NAME);
    DisplayInfo("Update available ...!", "Please wait", codeVersion);
    delay(3000);
    }

  }
}

void get4GApiReponse()
{
  // Send the AT+QHTTPSTOP command to stop any ongoing HTTP(S) request.
  String command = "AT+QHTTPSTOP";
  String expectedResponse = "OK";
  unsigned long timeout = 5000;

  String response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "OK")
  {
    Serial.println("HTTP(S) request stopped successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + "Timeout while waiting for response.");
  }
  else
  {
    Serial.println("Error: " + response);
  }

  wdt.feed();

  // Send the AT+QHTTPCFG="contextid",1 command.
  command = "AT+QHTTPCFG=\"contextid\",1";
  expectedResponse = "OK";
  timeout = 5000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "OK")
  {
    Serial.println("Context ID configured successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + "Timeout while waiting for response.");
    apiError = "Context ID failed";
  }
  else
  {
    Serial.println("Error: " + response);
    apiError = "Context ID failed";
  }

  wdt.feed();

  command = "AT+QHTTPCFG=\"responseheader\",1";
  expectedResponse = "OK";
  timeout = 5000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "OK")
  {
    Serial.println("Response header configured successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + "Timeout while waiting for response.");
    apiError = "Response header failed";
  }
  else
  {
    Serial.println("Error: " + response);
    apiError = "Response header failed";
  }

  wdt.feed();

  command = "AT+QIACT=1";
  expectedResponse = "OK";
  timeout = 15000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "OK")
  {
    Serial.println("PDP context activated successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + "Timeout while waiting for response.");
  }
  else
  {
    Serial.println("Error: " + response);
  }

  wdt.feed();

  String urlSize = String(deviceApiUrl.length());
  // Set the URL using the AT+QHTTPURL command.
  command = "AT+QHTTPURL=" + urlSize + ",80";
  expectedResponse = "CONNECT";
  timeout = 10000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "CONNECT")
  {
    Serial.println("AT+QHTTPURL request is ready to accept the URL.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println("Timeout while waiting for the CONNECT response.");
    apiError = "AT+QHTTPURL TIMEOUT";
  }
  else
  {
    Serial.println("Error: " + response);
    apiError = "AT+QHTTPURL failed";
  }

  wdt.feed();

  command = deviceApiUrl;
  expectedResponse = "OK";
  timeout = 10000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response == "OK")
  {
    Serial.println("URL set successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + "Timeout while waiting for response.");
    apiError = "URL timeout failed";
  }
  else
  {
    Serial.println("Error: " + response);
    apiError = "URL timeout failed";
  }

  wdt.feed();

  command = "AT+QHTTPGET=20";
  expectedResponse = "+QHTTPGET: 0,200";
  timeout = 30000;

  response = sendATCommand3(command, expectedResponse, timeout);

  if (response.indexOf("+QHTTPGET: 0,200") != -1)
  {
    Serial.println("HTTP GET request sent successfully.");
  }
  else if (response == "TIMEOUT")
  {
    Serial.println(command + " Timeout while waiting for response.");
    apiError = "Get API data timeout failed";
    Serial.println("4G API GET failed — returning to caller");
    return;
  }
  else
  {
    Serial.println("Error: " + response);
    apiError = "URL timeout failed";
    Serial.println("4G API GET failed — returning to caller");
    return;
  }

  wdt.feed();

  command = "AT+QHTTPREAD=80";
  timeout = 15000;
  eg25.println(command);
  tempGsmJsonResponse = "";
  unsigned long readStart = millis();
  while (millis() - readStart < timeout)
  {
    wdt.feed();
    if (eg25.available())
    {
      String responseLine = eg25.readStringUntil('\n');

      if (responseLine.startsWith("{"))
      {
        GsmProcessJsonStarted = true;
      }

      if (GsmProcessJsonStarted)
      {
        tempGsmJsonResponse += responseLine;
        if (responseLine.indexOf("}") != -1)
        {
          gsmJSONResponse = tempGsmJsonResponse;
          tempGsmJsonResponse = "";
          GsmProcessJsonStarted = false;
          break;
        }
      }
      if (responseLine == "OK" || responseLine == "+QHTTPREAD: 0")
      {
      }
    }
  }
  Serial.println("4G API Response: " + gsmJSONResponse);
  parseApiResponse(gsmJSONResponse);
}
String sendATCommand3(const String &command, const String &expectedResponse, unsigned long timeout)
{
  eg25.println(command);
  // Serial.println("Command sent: " + command); // Debug print

  unsigned long startTime = millis();

  while (millis() - startTime < timeout)
  {
    wdt.feed();
    if (eg25.available())
    {
      String response = eg25.readStringUntil('\n');
      response.trim(); // Remove whitespace characters
      // Serial.println("The reponse is "+response);
      if (response.length() > 0)
      { // Ignore empty lines
        // Serial.println("Response received: " + response); // Debug print

        if (response.indexOf(expectedResponse) != -1)
        {
          return response;
        }
        else if (response.indexOf("+QMTRECV") != -1)
        {
          bufferGsmMqttRecv(response);
        }
        else if (response.indexOf("ERROR") != -1)
        {
          return "ERROR";
        }
      }
    }
  }

  return "TIMEOUT";
}
String sendWifiATCommands(const String &command, const String &expectedResponse, unsigned long timeout)
{
  espSerial.println(command);
  // Serial.println("Command sent: " + command); // Debug print
  unsigned long startTime = millis();

  while (millis() - startTime < timeout)
  {
    wdt.feed();
    if (espSerial.available())
    {

      String response = espSerial.readStringUntil('\n');
      response.trim(); // Remove whitespace characters
      if (response.length() > 0)
      { // Ignore empty lines
        // Serial.println("Response received: " + response); // Debug print

        if (response.indexOf(expectedResponse) != -1)
        {
          return response;
        }
        else if (response.indexOf("+MQTTSUBRECV:") != -1)
        {
          // Incoming MQTT message arrived during AT command - buffer it for later
          bufferMqttRecv(response);
        }
        else if (response.indexOf("ERROR") != -1)
        {
          return "ERROR";
        }
        else if (response.indexOf("+MQTTDISCONNECTED") != -1)
        {
          Serial.println("MQTT Disconnected (during AT command)!");
          mqttConnected = false;
          deviceBusyStatus = 0;
          lightUpLED(1, 2, 2, 0);
          DisplayHomepage(displayLine1, displayLine2);
          return "MQTT_DISCONNECT";
        }
        else if (response.indexOf("WIFI DISCONNECT") != -1)
        {
          Serial.println("Wi-Fi Disconnected (during AT command)!");
          wifiConnected = false;
          mqttConnected = false;
          return "WIFI_DISCONNECT";
        }
      }
    }
  }

  return "TIMEOUT";
}

// Actively check WiFi connection status by querying ESP32
// Returns true if WiFi + MQTT are both connected, false otherwise
bool checkWifiAndMqttStatus()
{
  // Flush stale data
  delay(50);

  espSerial.println("AT+CWSTATE?");
  unsigned long startTime = millis();
  bool wifiOk = false;

  while (millis() - startTime < 3000)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      response.trim();

      if (response.startsWith("+CWSTATE:"))
      {
        // +CWSTATE:<state>,<"ssid">
        // state 2 = connected with IP, anything else = not connected
        int state = response.substring(9, 10).toInt();
        if (state == 2)
        {
          wifiOk = true;
        }
        else
        {
          Serial.println("WiFi check: NOT connected (state=" + String(state) + ")");
          wifiOk = false;
        }
        break;
      }
      else if (response.indexOf("+MQTTSUBRECV:") != -1)
      {
        bufferMqttRecv(response);
      }
      else if (response.indexOf("WIFI DISCONNECT") != -1)
      {
        wifiOk = false;
        break;
      }
    }
  }

  if (!wifiOk)
  {
    wifiConnected = false;
    mqttConnected = false;
    return false;
  }

  // Check MQTT status
  espSerial.println("AT+MQTTCONN?");
  startTime = millis();
  bool mqttOk = false;

  while (millis() - startTime < 3000)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      response.trim();

      if (response.startsWith("+MQTTCONN:"))
      {
        // +MQTTCONN:0,<state>,... state 3 = connected, 4 = connected with subscription
        int commaIdx = response.indexOf(',');
        int secondComma = response.indexOf(',', commaIdx + 1);
        int state = response.substring(commaIdx + 1, secondComma).toInt();
        if (state >= 3)
        {
          mqttOk = true;
        }
        else
        {
          Serial.println("MQTT check: NOT connected (state=" + String(state) + ")");
          mqttOk = false;
        }
        break;
      }
      else if (response.startsWith("+MQTTSUBRECV:"))
      {
        bufferMqttRecv(response);
      }
    }
  }

  if (!mqttOk)
  {
    mqttConnected = false;
  }

  wifiConnected = wifiOk;
  mqttConnected = mqttOk;
  return (wifiOk && mqttOk);
}

// Switch device to offline mode and update display
// Full offline - WiFi is actually gone
void goOffline()
{
  Serial.println("*** Going OFFLINE - WiFi lost ***");
  wifiConnected = false;
  mqttConnected = false;
  lightUpLED(1, 2, 2, 0);
  deviceBusyStatus = 0;
  deviceHomePageStatus = true;
  DisplayHomepage(displayLine1, displayLine2);
}

// MQTT only lost - WiFi still connected but can't reach broker
void goMqttOffline()
{
  Serial.println("*** MQTT connection lost (WiFi still up) ***");
  mqttConnected = false;
  lightUpLED(1, 2, 2, 0);
  deviceBusyStatus = 0;
  deviceHomePageStatus = true;
  DisplayHomepage(displayLine1, displayLine2);
}

// 4G MQTT lost - go offline and start watchdog timer
void goGsmMqttOffline()
{
  Serial.println("*** 4G MQTT connection lost ***");
  gsmMqttConnected = false;
  lightUpLED(1, 2, 2, 0);
  deviceBusyStatus = 0;
  deviceHomePageStatus = true;
  DisplayHomepage(displayLine1, displayLine2);
  if (gsmBusySinceTime == 0)
  {
    gsmBusySinceTime = millis();
  }
}

// Fire-and-forget publish using AT+MQTTPUB (no quotes in data = no parser issues)
// Use ONLY for periodic device logs (GPS data etc.) - NOT for ticket/card data
void publishLogFireAndForget(const String &topic, const String &payload)
{
  espSerialBusy = true;
  drainingDelay(50);

  String command = "AT+MQTTPUB=0,\"" + topic + "\",\"" + payload + "\",1,0";
  espSerial.println(command);
  Serial.println("WIFI - Device log sent.");
  deviceLogStatus = 1;

  espSerialBusy = false;
}

void resetAndDisconnect()
{
  unsigned long timeout = 15000;

  // Flush any stale data before reset
  while (espSerial.available())
  {
    espSerial.read();
  }

  // Send AT+RST
  String response = sendWifiATCommands("AT+RST", "ready", timeout);

  if (response.indexOf("ready") != -1 || response != "TIMEOUT")
  {
    Serial.println("ESP32 Successful (Ready)");
    delay(1000); // Give ESP32 time to fully initialize after reset

    // Proceed to disconnect from any previously remembered WiFi
    response = sendWifiATCommands("AT+CWQAP", "OK", timeout);

    if (response.indexOf("OK") != -1)
    {
      Serial.println("WIFI reset Successful");
    }
    else
    {
      Serial.println("WIFI reset Failed");
    }
  }
  else
  {
    Serial.println("WIFI Init failed - retrying reset");
    delay(2000);
    while (espSerial.available()) { espSerial.read(); }
    response = sendWifiATCommands("AT+RST", "ready", timeout);
    if (response.indexOf("ready") != -1)
    {
      Serial.println("ESP32 reset succeeded on retry");
      delay(1000);
      sendWifiATCommands("AT+CWQAP", "OK", timeout);
    }
    else
    {
      Serial.println("WIFI Init failed after retry");
    }
  }
}

void setUpEps32Ble()
{
  unsigned long timeout = 5000; // 5 seconds
  String command = "AT+BLEINIT=1";
  String response = sendWifiATCommands(command, "OK", timeout);

  if (response == "OK")
  {
    Serial.println("ESP32 BLE Configured");
  }
  else
  {
    Serial.println("ESP32 BLE configured Failed !");
  }
}
void startEps32BleScan()
{
  unsigned long timeout = 5000; // 5 seconds
  String command = "AT+BLESCAN=1";
  String response = sendWifiATCommands(command, "OK", timeout);

  if (response == "OK")
  {
    Serial.println("ESP32 BLE scan started");
  }
  else
  {
    Serial.println("ESP32 BLE scan Failed !");
  }
}

void configureReconnect()
{
  unsigned long timeout = 5000; // 5 seconds
  String command = "AT+CWRECONNCFG=2,5";
  String response = sendWifiATCommands(command, "OK", timeout);

  if (response == "OK")
  {
    Serial.println("WIFI reconnect config Successful");
  }
  else
  {
    Serial.println("WIFI reconnect config Failed");
  }
}
// Function to load Wi-Fi credentials from SD card
bool loadWifiConfig()
{
  File file = SD.open(wifiConfigFile, FILE_READ);
  if (file)
  {
    retrievedWifiSsid = file.readStringUntil('\n');
    retrievedWifiPass = file.readStringUntil('\n');
    retrievedWifiSsid.trim(); // Remove any extra spaces or newlines
    retrievedWifiPass.trim();
    Serial.println("WiFi credentials loaded from SD card:");
    Serial.print("SSID: ");
    Serial.println(retrievedWifiSsid);
    Serial.print("Password: ");
    Serial.println(retrievedWifiPass);

    file.close();
    // Debug prints

    return true;
  }
  else
  {
    Serial.println("Failed to open file for reading.");
    return false;
  }
}

void connectToWiFi()
{
  
  if (loadWifiConfig())
  {
    Serial.println("SD Wifi Data");
  }
  else
  {
    Serial.println("SD Card Failed");
    retrievedWifiSsid =  secondarySSID;
    retrievedWifiPass =  secondaryPassword;
  }
  // retrievedWifiSsid = "Zed";
  // retrievedWifiPass = "Wireless4U!";

  // Attempt to connect to the primary Wi-Fi network
  primarySSID = retrievedWifiSsid;
  primaryPassword = retrievedWifiPass;
  espSerial.print("AT+CWJAP=\"");
  espSerial.print(primarySSID);
  espSerial.print("\",\"");
  espSerial.print(primaryPassword);
  espSerial.println("\"");
  Serial.println(primarySSID);
  Serial.println(primaryPassword);

  unsigned long startTime = millis();
  boolean espWifiConnected = false;
  boolean errorOccurred = false;
  String errorResponse;

  while (millis() - startTime < WifiTimeout)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');

      if (response.indexOf("WIFI CONNECTED") != -1)
      {
        espWifiConnected = true;
        break;
      }
      else if (response.indexOf("+CWJAP:") != -1)
      {
        // An error code is received, parse it
        int errorCode = response.substring(response.indexOf("+CWJAP:") + 7).toInt();
        String errorMessage;

        switch (errorCode)
        {
        case 1:
          errorMessage = "Connection timeout.";
          break;
        case 2:
          errorMessage = "Wrong password.";
          break;
        case 3:
          errorMessage = "No Access point.";
          break;
        case 4:
          errorMessage = "Connection failed.";
          break;
        default:
          errorMessage = "Unknown error occurred.";
        }

        errorResponse = "Failed to connect to primary WIFI : " + errorMessage;
        errorOccurred = true;
        DisplayInfo(errorMessage, primarySSID, codeVersion);
        delay(3000);
        break;
      }
    }
  }

  if (espWifiConnected)
  {
    // "WIFI CONNECTED" received, now wait for "WIFI GOT IP" and "OK"
    startTime = millis();
    boolean okReceived = false;
    boolean gotIP = false;

    while (millis() - startTime < WifiTimeout)
    {
      if (espSerial.available())
      {
        String response = espSerial.readStringUntil('\n');

        if (response.indexOf("WIFI GOT IP") != -1)
        {
          gotIP = true;
          Serial.println("WiFi got IP address");
        }
        if (response.indexOf("OK") != -1)
        {
          okReceived = true;
        }
        if (okReceived && gotIP)
        {
          break;
        }
      }
    }

    if (okReceived)
    {
      if (!gotIP)
      {
        Serial.println("Warning: WIFI connected but no IP received, waiting extra...");
        unsigned long ipWaitStart = millis();
        while (millis() - ipWaitStart < 5000)
        {
          if (espSerial.available())
          {
            String response = espSerial.readStringUntil('\n');
            if (response.indexOf("WIFI GOT IP") != -1)
            {
              gotIP = true;
              Serial.println("WiFi got IP address (delayed)");
              break;
            }
          }
        }
      }
      Serial.println("Connected to Wi-Fi: Successful (Primary)");
      connectedSSID = primarySSID;
       DisplayInfo("WIFI connected", primarySSID, codeVersion);
      Serial.println(useSpeakerSound);
      if (useSpeakerSound)
      {
        fileIndex = 4;
        playSpeaker = true;
        Serial.println("primary wifi connected");
      }
      wifiConnected = true;
    }
  }
  else if (errorOccurred)
  {
    Serial.println(errorResponse);
    DisplayInfo("Reconnecting", "ZIG WIFI", codeVersion);
    // Attempt to connect to the secondary Wi-Fi network
    Serial.println("Reconnceting to secondary WIFI ");
    espSerial.print("AT+CWJAP=\"");
    espSerial.print(secondarySSID);
    espSerial.print("\",\"");
    espSerial.print(secondaryPassword);
    espSerial.println("\"");

    startTime = millis();
    espWifiConnected = false;
    errorOccurred = false;

    while (millis() - startTime < WifiTimeout)
    {
      if (espSerial.available())
      {
        String response = espSerial.readStringUntil('\n');

        if (response.indexOf("WIFI CONNECTED") != -1)
        {
          primarySSID = secondarySSID;
          primaryPassword = secondaryPassword;
          espWifiConnected = true;
          break;
        }
        else if (response.indexOf("+CWJAP:") != -1)
        {
          // An error code is received, parse it
          int errorCode = response.substring(response.indexOf("+CWJAP:") + 7).toInt();
          String errorMessage;

          switch (errorCode)
          {
          case 1:
            errorMessage = "Connection timeout.";
            break;
          case 2:
            errorMessage = "Wrong password...!";
            break;
          case 3:
            errorMessage = "No Access point.";
            break;
          case 4:
            errorMessage = "Connection failed.";
            break;
          default:
            errorMessage = "Unknown error occurred.";
          }

          errorResponse = "Failed to connect to secondary WIFI " + errorMessage;
          errorOccurred = true;
          DisplayInfo(errorMessage, secondarySSID, codeVersion);
          delay(3000);
          break;
        }
      }
    }

    if (espWifiConnected)
    {
      // "WIFI CONNECTED" received, now wait for "WIFI GOT IP" and "OK"
      startTime = millis();
      boolean okReceived = false;
      boolean gotIP = false;

      while (millis() - startTime < WifiTimeout)
      {
        if (espSerial.available())
        {
          String response = espSerial.readStringUntil('\n');

          if (response.indexOf("WIFI GOT IP") != -1)
          {
            gotIP = true;
            Serial.println("WiFi got IP address (secondary)");
          }
          if (response.indexOf("OK") != -1)
          {
            okReceived = true;
          }
          if (okReceived && gotIP)
          {
            break;
          }
        }
      }

      if (okReceived)
      {
        if (!gotIP)
        {
          Serial.println("Warning: Secondary WIFI connected but no IP, waiting extra...");
          unsigned long ipWaitStart = millis();
          while (millis() - ipWaitStart < 5000)
          {
            if (espSerial.available())
            {
              String response = espSerial.readStringUntil('\n');
              if (response.indexOf("WIFI GOT IP") != -1)
              {
                gotIP = true;
                Serial.println("WiFi got IP address (secondary delayed)");
                break;
              }
            }
          }
        }
        Serial.println("Connected to Wi-Fi: Successful (secondary)");
         DisplayInfo("WIFI connected", secondarySSID, codeVersion);
        if (useSpeakerSound)
        {
          fileIndex = 4;
          Serial.println("wifi connected");
          playSpeaker = true;
        }
        connectedSSID = secondarySSID;
        primarySSID = secondarySSID;
        primaryPassword = secondaryPassword;
        wifiConnected = true;
      }
    }
    else if (errorOccurred)
    {
      Serial.println(errorResponse);
    }
  }
}
void enableAutoConnect()
{
  String command = "AT+CWAUTOCONN=1";
  unsigned long timeout = 5000; // 5 seconds

  String response = sendWifiATCommands(command, "OK", timeout);

  if (response == "OK")
  {
    Serial.println("Autoconnect enabled: Successful");
  }
  else
  {
    Serial.println("Autoconnect enabling: Failed");
  }
}

bool sendATCommand2(const String &command, const String &expectedResponse, unsigned long timeout)
{
  espSerial.println(command);
  unsigned long startTime = millis();

  while (millis() - startTime < timeout)
  {
    wdt.feed();
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      // Serial.println("Send AT command2 response :" + response);
      if (response.indexOf(expectedResponse) != -1)
      {
        return true;
      }
      else if (response.indexOf("ERROR") != -1)
      {
        return false;
      }
      else if (response.indexOf("WIFI DISCONNECT") != -1)
      {
        Serial.println("Wi-Fi Disconnected (during AT command2)!");
        wifiConnected = false;
        mqttConnected = false;
        return false;
      }
    }
  }

  return false;
}



void sendHTTPCGET(const char *url)
{
  String command = "AT+HTTPCGET=\"" + String(url) + "\"";
  espSerial.println(command);
  Serial.println(command);
}

void subscribeToTopic(const String &topic)
{
  unsigned long timeout = 5000;
  String command = "AT+MQTTSUB=0,\"" + topic + "\",1";
  bool success = sendATCommand2(command, "OK", timeout);
  if (success)
  {
    Serial.print("Subscribed to topic: ");
    mqttConnected = true;
    Serial.println(topic);
  }
  else
  {
    Serial.print("Failed to subscribe to topic: ");
    Serial.println(topic);
  }
}

String generateRandomString(size_t length)
{
  String randomString = "";
  char charset[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  size_t charsetSize = sizeof(charset) - 1;
  for (size_t i = 0; i < length; i++)
  {
    uint8_t randomIndex = random(charsetSize);
    randomString += charset[randomIndex];
  }

  return randomString;
}

void connectToMQTT()
{
  sendATCommand2("AT+MQTTDISCONN=0", "OK", 2000); // <--- added line
  

  String revicevedMqttClientId = generateRandomString(4);
  String clientId = deviceTeensyMacAddress + "-" + "WIFI" + "-" + revicevedMqttClientId + teensySN();
  Serial.println("Wifi Mqtt client id : " + clientId);

  unsigned long timeout = 10000; // Timeout set to 5 seconds

  if (sendATCommand2("AT+MQTTUSERCFG=0,1,\"" + clientId + "\",\"Validator\",\"Validator@4U!\",0,0,\"\"", "OK", timeout) &&
      sendATCommand2("AT+MQTTCONNCFG=0,30,0,\"\",\"\",0,0", "OK", timeout) &&
      sendATCommand2("AT+MQTTCONN=0,\"" + String(mqttServer) + "\"," + String(mqttPort) + ",0", "OK", timeout))
  {
    if (wifiApiDataState)
    {
      delay(200);
      subscribeToTopic(deviceTopic);
      delay(1000);
      subscribeToTopic(deviceReactTopic);
      Serial.println("MQTT connected successfully!");
      mqttConnected = true;
      enableWifiGsm = true;
    }
    if (!firstTimeDeviceLog)
    {
      displayLine1 = connectedSSID;
      displayLine2 = ssid;
      deviceBusyStatus = 1;
      DisplayHomepage(displayLine1, displayLine2);
    }
  }
  else
  {
    Serial.println("MQTT Connection Failed!");
    // mqttConnected = false;
  }
}
void cleanMQTTSession()
{
  unsigned long timeout = 5000; // Timeout set to 5 seconds

  if (sendATCommand2("AT+MQTTCLEAN=0", "OK", timeout))
  {
    Serial.println("MQTT session cleaned successfully!");
  }
  else
  {
    Serial.println("MQTT session cleaning failed!");
  }
}

void checkGPSLocation()
{
  String command = "AT+QGPSLOC=2";          // AT command to check GPS location
  String expectedError = "+CME ERROR: 516"; // Expected error response
  unsigned long timeout = 6000;             // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command

  if (response == command)
  { // Command echo received, now expect the GPS data
    response = sendATCommand3("", "+QGPSLOC:", timeout);
  }

  if (response.startsWith("+CME ERROR:"))
  {
    // Serial.println("GPS is not enabled yet.");
  }
  else if (response.startsWith("+QGPSLOC:"))
  {
    // Extract the GPS data
    gpsData = response.substring(response.indexOf(":") + 1);
    gpsData.trim(); // Remove leading and trailing whitespaces
    // Serial.println("GPS data: " + gpsData);
  }
  else
  {
    // Serial.println("Unexpected response in GPS");
  }
}
void retrieveConnectionDetails()
{
  espSerial.println("AT+CWJAP?");

  unsigned long delayTime = 1000; // Delay set to 1 second
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;

  while (elapsedTime < 5000)
  { // Wait for 5 seconds to receive the response
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      // Serial.println(response);
      if (response.startsWith("+CWJAP:"))
      {
        // Extract SSID and MAC address from the response
        int firstComma = response.indexOf(',');
        int secondComma = response.indexOf(',', firstComma + 1);
        String ssid = response.substring(firstComma + 2, secondComma - 1);
        String macAddress = response.substring(secondComma + 2, response.lastIndexOf('"'));

        // Print SSID and MAC address
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("MAC Address: ");
        Serial.println(macAddress);

        return; // Exit the function after retrieving the details
      }
    }

    delay(delayTime);
    elapsedTime = millis() - startTime;
  }

  Serial.println("Timeout: Failed to retrieve connection details!");
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void teensyDoReboot()
{
  SCB_AIRCR = 0x05FA0004;
}

void ticketvalidatedbrightess()
{
 displaybrightcotrolcenter = false;
 if(displayBrightnessCondition == true)
 {
  displaybrightesscontrol(100);
 }
 else {
  displaybrightesscontrol(displayBrightnessLevel);
     }
        
}




void ProcessDisplayData(String str)
{
  deviceBusyStatus = 1;
  deviceHomePageStatus = true;
  deviceInfoPageStatus = true;
  str.trim();

  // Queue raw ticket payload for /ack forwarding
  if (enableWifiGsm)
    addToWifiQueue(deviceAckTopic, str, QUEUE_PRIORITY_HIGH);
  else
    addToGsmAckQueue(str);

  // if (str == sosStatus)
  // {
  //   displayCommand("page 8");

  //   if (useSpeakerSound)
  //   {
  //     fileIndex = 5;
  //     playSpeaker = true;
  //   }
  //   else
  //   {
  //     playSOSSound();
  //   }
  //   // DisplayHomepage(displayLine1, displayLine2);
  //   return;
  // }
  // else if (str == stopStatus)
  // {
  //   displayCommand("page 9");

  //   if (useSpeakerSound)
  //   {

  //     fileIndex = 3;
  //     playSpeaker = true;
  //     playTicketSoundAndLight(4);
  //   }
  //   else
  //   {
  //     playTicketSoundAndLight(4);
  //   }
  //   DisplayHomepage(displayLine1, displayLine2);
  //   return;
  // }

  String status = str.substring(0, 3);
  int tickets = str.substring(3, 6).toInt();
  String TicketUserName = getValue(str, '#', 1);
  int TicketType = getValue(str, '#', 2).toInt();
  String TicketValidationId = getValue(str, '#', 3);
  String activationDate = getValue(str, '#', 4);
  String expiryDate = getValue(str, '#', 5);
  // Check if activationDate starts with a letter
  bool activationDateStartsWithLetter = (activationDate.length() > 0) && isAlpha(activationDate.charAt(0));

  // New Serial print statement with structured output
  Serial.print("Name: " + TicketUserName + ", Number of Tickets: " + String(tickets) + ", Ticket ID: " + TicketValidationId + ", Ticket Type: " + String(TicketType));
  // Serial.print(", Activation Date: " + activationDate);

  if (activationDateStartsWithLetter)
  {
    // Serial.print(" (Starts with Letter)");
    if (status == validTicketStatus)
    {
      if (tickets >= 1)
      {
        peopleHold = +tickets;
        if (TicketType == 1)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "57250", "0", activationDate, "55106"); // Gold
           ticketvalidatedbrightess();
        }
        else if (TicketType == 2)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "7909", "0", activationDate, "9892"); // Light green
          ticketvalidatedbrightess();
        }
        else if (TicketType == 3)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "8790", "65535", activationDate, "15094"); // blue
          ticketvalidatedbrightess();
        }
        else if (TicketType == 4)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "33046", "65535", activationDate, "37366"); // violet
          ticketvalidatedbrightess();
        
        }
        else if (TicketType == 5)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "12060", "0", activationDate, "9947"); // light blue
          ticketvalidatedbrightess();
        }
        else if (TicketType == 6)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "65535", "0", activationDate, "63390"); // white with name
           ticketvalidatedbrightess();
        }
        else
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "65535", "0", "", "63390");
           ticketvalidatedbrightess();

        
        }
      }
    }
    if (enableMultipleLights)
    {
      for (int i = 0; i < tickets; i++)
      {
        playTicketSoundAndLight(1);
        delay(multipleTicketDelay);
      }
    }
    else
    {
      if (useSpeakerSound)
      {
        fileIndex = 1;
        playSpeaker = true;
      }
      else
      {
        playTicketSoundAndLight(1);
      }
    }
    if (!displayLastTicket)
    {
      delay(validTiceketDelay);
      DisplayHomepage(displayLine1, displayLine2);
      
    }
    return;
  }

  Serial.println(", Expiry Date: " + expiryDate);
  if (status == validTicketStatus)
  {
    if (TofModes == 4 && personInsideThreshold)
    {
      unsigned long elapsedTime = millis() - benchmarkEntryStartTime; // Calculate elapsed time
      Serial.print("Time elapsed in milliseconds: ");
      Serial.println(elapsedTime);
      String elapsedTimeString;
      if (elapsedTime > 1000)
      {
        float elapsedTimeInSeconds = elapsedTime / 1000.0;
        elapsedTimeString = "Time taken: " + String(elapsedTimeInSeconds, 2) + " s";
      }
      else
      {
        elapsedTimeString = "Time taken: " + String(elapsedTime) + " ms";
      }
      DisplayInfo(TicketUserName, elapsedTimeString, TicketValidationId);
      playTicketSoundAndLight(1);
      String mqttBenchmarkPayload = "201001#" + TicketUserName +
                                    "#0#" + TicketValidationId + "#val012#val345#val" + String(elapsedTime);
                              
      // WiFi: queue for single-thread serial access. GSM: direct send.
      if (enableWifiGsm)
      {
        addToWifiQueue(deviceBenchMarkTopic, mqttBenchmarkPayload, QUEUE_PRIORITY_HIGH);
      }
      else
      {
        delay(100);
        publishRawToTopicEG25(deviceBenchMarkTopic, mqttBenchmarkPayload);
      }
      return;
    }
    if (tickets >= 1)
    {
      peopleHold = +tickets;
      int lightValue = (TicketType == 1) ? 3 : 1;
      if (TicketType == specialTicketType)
      {
        // Serial.println("Special Ticket");
        DiplayScreenTicketID(TicketUserName, 0, tickets, "57250", "0", "VIP", "55106"); 
        ticketvalidatedbrightess();
      } 
      
      /*  else if (TicketType == 2)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "7909", "0", activationDate, "9892"); // Light green
          ticketvalidatedbrightess();
        }
        else if (TicketType == 3)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "8790", "65535", activationDate, "15094"); // blue
          ticketvalidatedbrightess();
        }
        else if (TicketType == 4)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "33046", "65535", activationDate, "37366"); // violet
          ticketvalidatedbrightess();
        
        }
        else if (TicketType == 5)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "12060", "0", activationDate, "9947"); // light blue
          ticketvalidatedbrightess();
        }
        else if (TicketType == 6)
        {
          DiplayScreenTicketID(TicketUserName, 0, tickets, "65535", "0", activationDate, "63390"); // white with name
           ticketvalidatedbrightess();
        }*/
      else
      {
        // Serial.println("Valid Ticket");
        DiplayScreenTicketID(TicketUserName, 0, tickets, "65535", "0", "", "63390");
        ticketvalidatedbrightess();
      }
      if (enableMultipleLights)
      {
        for (int i = 0; i < tickets; i++)
        {
          playTicketSoundAndLight(lightValue);
          delay(multipleTicketDelay);
        }
      }
      else
      {
        if (useSpeakerSound)
        {
          fileIndex = 1;
          playSpeaker = true;
          Serial.print("valid"); 
        }
        else
        {
          playTicketSoundAndLight(1);
        }
      }
    }
   if (!displayLastTicket)
    {

     delay(validTiceketDelay);
      DisplayHomepage(displayLine1, displayLine2);
       
    }
  }
  else if (status == invalidTicketStatus)
  {
    Serial.println("Invalid Ticket");
   
    displayCommand("page 3");
    ticketvalidatedbrightess();   
    if (useSpeakerSound)
    {
      fileIndex = 2;
      playSpeaker = true;
      Serial.println("invalid");
    
    }
    
    playTicketSoundAndLight(0);
    DisplayHomepage(displayLine1, displayLine2);
    
    
  }
}

void publishRawToTopicEG25(const String &topic, const String &payload)
{
  // If another instance of this function is already running, skip this execution
  if (isPublishing)
  {
    Serial.println("Publish skipped: another instance is running.");
    return;
  }

  isPublishing = true; // Set the flag to indicate the function is busy

  while (true)
  {
    String command = "AT+QMTPUBEX=3,0,0,0,\"" + topic + "\"," + String(payload.length());
    
    // Send the initial command and check for the ">" response
    String response = sendATCommand3(command, ">", 3000);

    if (response == ">")
    {
      // ">" received, now send the payload
      sendATCommand3(payload, "+QMTPUB: 3,0,0", 5000);
      Serial.println("4G - Device log sent");
      deviceLogStatus = 1;
      break; // Exit the loop if data is published successfully
    }
    else if (response == "ERROR")
    {
      Serial.println("Could not send the data.");
      restartEg25MqttFlow();
    }
    else
    {
      Serial.println("EG25 publish topic Unexpected response: " + response);
    }

    // Add a delay here if you want to wait before retrying
    delay(1000); // Wait for 1 second before retrying, for example
  }

  isPublishing = false; // Clear the flag after the function completes
}

void publishLogFireAndForgetEG25(const String &topic, const String &payload)
{
  // Two-step but blind send - no waiting for ">" response
  String command = "AT+QMTPUBEX=3,0,0,0,\"" + topic + "\"," + String(payload.length());
  eg25.println(command);
  delay(150); // Brief delay for module to respond with ">"
  eg25.println(payload);
  Serial.println("4G - Device log sent (fire-and-forget).");
  deviceLogStatus = 1;
}

void publishNFCDataRawToTopicEG25(const String &topic, const String &payload)
{
  // If another instance of this function is already running, skip this execution
  if (isPublishing)
  {
    Serial.println("Publish skipped: another instance is running.");
    return;
  }

  isPublishing = true; // Set the flag to indicate the function is busy

  while (true)
  {
    String command = "AT+QMTPUBEX=3,0,0,0,\"" + topic + "\"," + String(payload.length());

    // Send the initial command and check for the ">" response
    String response = sendATCommand3(command, ">", 500);

    if (response == ">")
    {
      // ">" received, now send the payload
      sendATCommand3(payload, "+QMTPUB: 3,0,0", 500);
      Serial.println("4G -  NFC Data sent");
      deviceLogStatus = 1;
      break; // Exit the loop if data is published successfully
    }
    else if (response == "ERROR")
    {
      Serial.println("Could not send the data.");
      restartEg25MqttFlow();
    }
    else
    {
      Serial.println("EG25 publish topic Unexpected response: " + response);
    }

    // Add a delay here if you want to wait before retrying
    delay(1000); // Wait for 1 second before retrying, for example
  }

  isPublishing = false; // Clear the flag after the function completes
}
void publishNFCDataToTopic(const String &topic, const String &payload, int size)
{
  if (isPublishing)
  {
    Serial.println("Publish skipped: another instance is running.");
    return;
  }

  isPublishing = true;
  espSerialBusy = true;
  drainingDelay(50); // Brief pause for listener thread to yield, drains serial

  String command = "AT+MQTTPUBRAW=0,\"" + topic + "\"," + String(size) + ",1,0";

  String cmdResp = sendWifiATCommands(command, ">", 3000);
  if (cmdResp == "WIFI_DISCONNECT")
  {
    Serial.println("WIFI NFC publish failed - WiFi lost");
    isPublishing = false;
    espSerialBusy = false;
    goOffline();
    return;
  }
  if (cmdResp == "TIMEOUT")
  {
    Serial.println("WIFI NFC publish failed - MQTT no prompt");
    isPublishing = false;
    espSerialBusy = false;
    goMqttOffline();
    return;
  }

  drainingDelay(100); // Wait before payload, drains serial

  String pubResp = sendWifiATCommands(payload, "+MQTTPUB:OK", 3000);

  if (pubResp.indexOf("+MQTTPUB:OK") != -1)
  {
    Serial.println("WIFI - NFC Data sent.");
    deviceLogStatus = 1;
  }
  else if (pubResp == "WIFI_DISCONNECT")
  {
    Serial.println("WIFI - Disconnected during NFC publish");
    isPublishing = false;
    espSerialBusy = false;
    goOffline();
    return;
  }
  else
  {
    Serial.println("WIFI - NFC Data failed (MQTT timeout)");
    isPublishing = false;
    espSerialBusy = false;
    goMqttOffline();
    return;
  }

  isPublishing = false;
  espSerialBusy = false;
  processBufferedMqttMessages();
}
void publishRawToTopic(const String &topic, const String &payload, int size)
{
  // If another instance of this function is already running, skip this execution
  if (isPublishing)
  {
    Serial.println("Publish skipped: another instance is running.");
    return;
  }

  isPublishing = true;
  espSerialBusy = true;
  drainingDelay(50); // Brief pause for listener thread to yield

  String command = "AT+MQTTPUBRAW=0,\"" + topic + "\"," + String(size) + ",1,0";

  String cmdResp = sendWifiATCommands(command, ">", 3000);
  if (cmdResp == "WIFI_DISCONNECT")
  {
    Serial.println("WIFI publish failed - WiFi lost");
    isPublishing = false;
    espSerialBusy = false;
    goOffline();
    return;
  }
  if (cmdResp == "TIMEOUT")
  {
    Serial.println("WIFI publish failed - MQTT no prompt (no internet?)");
    isPublishing = false;
    espSerialBusy = false;
    goMqttOffline();
    return;
  }

  drainingDelay(100); // Wait before payload

  // Fire and forget - send payload, don't wait for +MQTTPUB:OK
  // This frees the serial immediately so listener can process incoming ticket responses
  // GPS logs are disposable (next one comes in 10s), no need to confirm delivery
  espSerial.println(payload);
  Serial.println("WIFI - Device log fired (no ACK wait).");
  deviceLogStatus = 1;

  isPublishing = false;
  espSerialBusy = false;
  processBufferedMqttMessages();
}

// Check and publish crash report if device had crashed
void checkAndPublishCrashReport()
{
  // Check if there's a crash to report
  if (!CrashReport) {
    Serial.println("No crash data to report - System healthy");
    return;
  }

  Serial.println("*** CRASH DETECTED - Sending Report ***");

  // Print crash report to Serial for debugging
  Serial.print(CrashReport);

  // Create JSON crash report
  DynamicJsonDocument crashJson(2048);
  crashJson["macAddress"] = deviceMacAddress;
  crashJson["timestamp"] = millis();
  crashJson["firmwareVersion"] = codeVersion;
  crashJson["deviceSerial"] = teensySN();

  // Add crash report info
  crashJson["crashReport"]["type"] = "SYSTEM_CRASH";
  crashJson["crashReport"]["details"] = "Device reset due to crash - see Serial output for full report";
  crashJson["crashReport"]["detected"] = true;

  // Add device status at time of crash detection
  crashJson["deviceStatus"]["wifiConnected"] = wifiConnected;
  crashJson["deviceStatus"]["mqttConnected"] = mqttConnected;
  crashJson["deviceStatus"]["gsmMqttConnected"] = gsmMqttConnected;
  crashJson["deviceStatus"]["qrScannerConnected"] = qrScannerConnected;
  crashJson["deviceStatus"]["temperature"] = String(InternalTemperature.readTemperatureC(), 2);

  // Serialize to string
  String crashPayload;
  serializeJson(crashJson, crashPayload);

  Serial.println("Crash Payload: ");
  Serial.println(crashPayload);

  // Publish crash report to error topic
  delay(500);
  if (enableWifiGsm && mqttConnected)
  {
    publishRawToTopic(deviceErrorTopic, crashPayload, crashPayload.length());
    Serial.println("Crash report sent via WiFi");
  }
  else if (gsmMqttConnected)
  {
    publishRawToTopicEG25(deviceErrorTopic, crashPayload);
    Serial.println("Crash report sent via 4G");
  }
  else
  {
    Serial.println("Cannot send crash report - No network connection");
  }

  // Clear crash report after sending
  CrashReport.clear();
  delay(1000);
}

// Process any MQTT messages that were buffered during AT command waits
void processBufferedMqttMessages()
{
  for (int i = 0; i < mqttRecvBufferCount; i++)
  {
    String response = mqttRecvBuffer[i];
    Serial.println("Processing buffered MQTT: " + response);

    lastWifiLogPublishTime = millis();
    int topicIndex = response.substring(14, response.indexOf(',')).toInt();
    String topic = response.substring(response.indexOf(',') + 2, response.lastIndexOf('\"'));
    int payloadLength = response.substring(response.lastIndexOf('\"') + 2, response.lastIndexOf(',')).toInt();
    String payload = response.substring(response.lastIndexOf(',') + 1, response.lastIndexOf(',') + 2 + payloadLength);

    Serial.println("Buffered topic: " + topic);
    Serial.println("Buffered payload:" + payload.trim());
    payload.trim();

    if (topic == deviceTopic)
    {
      ProcessDisplayData(payload);
    }
    else if (topic == deviceReactTopic)
    {
      if (payload == pushApiDataMqttStatus)
      {
        Serial.println("device will push mqtt data here (from buffer)");
        delay(3000);
        publishDeviceLogData();
      }
      else if (payload == reloadApiDataMqttStatus)
      {
        Serial.println("device will recall api data here (from buffer)");
        teensyDoReboot();
      }
    }

    mqttRecvBuffer[i] = "";
  }
  mqttRecvBufferCount = 0;
}

void processBufferedGsmMqttMessages()
{
  for (int i = 0; i < gsmMqttRecvBufferCount; i++)
  {
    String receivedString = gsmMqttRecvBuffer[i];
    Serial.println("Processing buffered GSM MQTT: " + receivedString);

    int lastCommaIndex = receivedString.lastIndexOf(',');
    lastGsmLogPublishTime = millis();
    String mqttTopic = receivedString.substring(receivedString.indexOf('"', 10) + 1, receivedString.indexOf('"', 15));
    String mqttData = receivedString.substring(lastCommaIndex + 1);
    mqttData.replace("\"", "");

    if (mqttTopic.equals(deviceTopic))
    {
      Serial.println("Buffered GSM validation topic: " + mqttTopic);
      Serial.println("Buffered GSM data: " + mqttData);
      ProcessDisplayData(mqttData);
    }
    else if (mqttTopic.equals(deviceReactTopic))
    {
      Serial.println("Buffered GSM API topic: " + mqttTopic);
      mqttData.trim();
      if (mqttData == pushApiDataMqttStatus)
      {
        Serial.println("device will push mqtt data here (from GSM buffer)");
        delay(3000);
        publishDeviceLogData();
      }
      else if (mqttData == reloadApiDataMqttStatus)
      {
        Serial.println("device restart here (from GSM buffer)");
        teensyDoReboot();
      }
    }

    gsmMqttRecvBuffer[i] = "";
  }
  gsmMqttRecvBufferCount = 0;
}

void publishDeviceLogData()
{
  Serial.println("++++++++Device LOG started++++++++");

  String deviceLogPayload;
  int deviceLogPayloadSize;
  if (sendOnlyGpsLog && !firstTimeDeviceLog)
  {
    // Build no-quote, no-braces, pipe-separated payload for AT+MQTTPUB
    String gpsPayload = "latitude:";
    if (gps.location.isValid())
    {
      gpsPayload += String(gps.location.lat(), 8);
      gpsPayload += "|longitude:" + String(gps.location.lng(), 8);
      gpsPayload += "|speedInMph:" + String(gps.speed.mph(), 2);
      gpsPayload += "|altitude:" + String(gps.altitude.value());
      gpsPayload += "|noOfSatilite:" + String(gps.satellites.value());
    }
    else
    {
      gpsPayload += "0|longitude:0|speedInMph:0|altitude:0|noOfSatilite:0";
    }
    gpsPayload += "|logStatus:" + String(deviceLogStatus);
    gpsPayload += "|temperatureCelsius:" + String(InternalTemperature.readTemperatureC(), 2);
    gpsPayload += "|tofCount:" + String(peopleCount);
    // Network signal: W=WiFi RSSI, G=GSM RSSI
    if (enableWifiGsm)
    {
      gpsPayload += "|network:W" + String(wifiRSSI);
    }
    else
    {
      gpsPayload += "|network:G" + String(getGsmRssiData());
    }

    if (enableWifiGsm)
    {
      publishLogFireAndForget(deviceLogTopic, gpsPayload);
    }
    else
    {
      // GSM fire-and-forget GPS log - same pipe-separated format as WiFi
      publishLogFireAndForgetEG25(deviceLogTopic, gpsPayload);
      processBufferedGsmMqttMessages();
    }
  }
  else
  {
    DynamicJsonDocument deviceLogJson(2048); // Adjust size according to your needs
    delay(100);
    deviceLogJson.clear();
    deviceLogJson["device"]["id"] = teensyMAC();
    deviceLogJson["device"]["firmwareVersion"] = codeVersion;
    deviceLogJson["device"]["validversion"] = String(validStoredVersion,1);
    deviceLogJson["device"]["invalidversion"] = String(invalidStoredVersion,1);
    deviceLogJson["device"]["serialNumber"] = teensySN();
    deviceLogJson["device"]["logStatus"] = deviceLogStatus;
    deviceLogJson["device"]["eepromStatus"] = eepromErrorStatus;
    deviceLogJson["device"]["temperatureCelsius"] = String(InternalTemperature.readTemperatureC(), 2);
    Serial.println("The value of bleAdStatus is " + String(bleAdStatus));
    if (bleScanMode)
    {
      deviceLogJson["ble"]["scanRSSI"] = requiredBleScanRssi;
    }
    if (bleAdStatus)
    {
      deviceLogJson["ble"]["version"] = bleVersion;
      deviceLogJson["ble"]["macAdress"] = bleMacAddress;
      deviceLogJson["ble"]["bleTxPower"] = bleTxPower;
      if (bleIbeaconMode)
      {
        deviceLogJson["ble"]["mode"] = "Ibeacon";
        deviceLogJson["ble"]["name"] = String(blename);
        deviceLogJson["ble"]["major"] = ibeaconMajor;
        deviceLogJson["ble"]["minor"] = ibeaconMinor;
        deviceLogJson["ble"]["bleScanMode"] = bleScanMode;
        deviceLogJson["ble"]["Rssi_1m"] = rssiat1m;
      }
      else
      {
        deviceLogJson["ble"]["mode"] = "Classic ble";
        deviceLogJson["ble"]["name"] = blename;
      }
      deviceLogJson["ble"]["connnectMode"] = bleConnectMode;
      deviceLogJson["ble"]["dfuMode"] = bleDfuMode;
    }
    else
    {
      deviceLogJson["ble"]["error"] = "Not enabled";
    }
    deviceLogJson["connected"]["devices"] = detectedDeviceInfo;
    deviceLogJson["connected"]["SDcard"] = isSdCardPresent;
    deviceLogJson["connected"]["pn532Connected"] = pn532Connected;
    if (pn532Connected)
    {
      deviceLogJson["connected"]["pn532Chip"] = pn532ChipInfo;
      deviceLogJson["connected"]["pn532Firmware"] = pn532FirmwareVersion;
    }
    if (simNumber != "")
    {
      deviceLogJson["LTE"]["sumber"] = simNumber;
    }
    deviceLogJson["tof"]["mode"] = TofModes;
    deviceLogJson["mqtt"]["mqttAddress"] = mqttServer;
    deviceLogJson["mqtt"]["validationTopic"] = deviceTopic;
    deviceLogJson["leds"]["validTicketDelay"] = validTiceketDelay;
    deviceLogJson["leds"]["invalidTicketDelay"] = invalidTicketDelay;
    deviceLogJson["leds"]["ticketGroupValidation"] = enableMultipleLights;
    delay(1000);
    deviceLogJson["gps"]["deviceMode"] = deviceTicketTelematricHybridMode;
    deviceLogJson["api"]["deviceApiData"] = apiError;
    deviceLogJson["display"]["displaystatus"] = displaybrigtnesserror;
     deviceLogJson["display"]["Ticketdisplaystatus"] = TicketDisplaybrightnesserror ;
     deviceLogJson["sound"]["validSound"]=  audioUpdateErrorValid;
     deviceLogJson["sound"]["invalidSound"] = audioUpdateErrorInvalid;

    // Add flash memory information
    extern unsigned long _flashimagelen;
    extern unsigned long _stext;
    delay(100);
    unsigned long totalFlash = 8 * 1024 * 1024;  // Teensy 4.1 has 8MB flash
    unsigned long usedFlash = (unsigned long)&_flashimagelen;
    unsigned long availableFlash = totalFlash - usedFlash;
    delay(100);
    deviceLogJson["flash"]["totalBytes"] = totalFlash;
    deviceLogJson["flash"]["availableBytes"] = availableFlash;
    deviceLogJson["flash"]["usedBytes"] = usedFlash;

    Serial.println(deviceLogPayload);
    delay(100);

    if (isEthernetConnected)
    {
      // deviceLogJson["network"]["Ethernet"] = true;
      // delay(2200);
      // serializeJson(deviceLogJson, deviceLogPayload);
      // if (mqttClient.publish(deviceLogTopic.c_str(), deviceLogPayload.c_str()))
      // {
      //   Serial.println("Ethernet Published1");
      // }
      // else
      // {
      //   Serial.println("Failed to publish JSON data via Ethernet1.");
      // }
    }
    else
    {
      if (enableWifiGsm)
      {
        delay(100);
        deviceLogJson["network"]["Imei"] = getIMEI();
        deviceLogJson["network"]["wifiSSID"] = primarySSID;
        deviceLogJson["network"]["wifiPassword"] = primaryPassword;
        deviceLogJson["network"]["macAddress"] = getESP32MAC();
        deviceLogJson["network"]["wifiBSSID"] = getWifiBSSID();
        deviceLogJson["network"]["rssi"] = getWifiRSSI();
        delay(200);
        serializeJson(deviceLogJson, deviceLogPayload);
        delay(100);
        int payloadSize = deviceLogPayload.length();
        delay(100);
        publishRawToTopic(deviceLogTopic, deviceLogPayload, payloadSize);
      }
      else
      {
        delay(100);
        deviceLogJson["network"]["Imei"] = getIMEI();
        delay(200);
        deviceLogJson["network"]["rssi"] = getGsmRssiData();
        delay(200);
        deviceLogJson["network"]["gsm"] = "4G Connected";
        deviceLogJson["network"]["wifiMacAddress"] = getESP32MAC();
        deviceLogJson["network"]["connectedNetwork"] = GsmOperatorName;
        deviceLogJson["network"]["networkType"] = GsmNetworkTypeString;
        delay(100);
        serializeJson(deviceLogJson, deviceLogPayload);
        publishRawToTopicEG25(deviceLogTopic, deviceLogPayload);
      }
    }
  }
  firstTimeDeviceLog = false;
}

void gsmEsp32bleScanThread()
{
  while (true)
  {
    // Check if there is any data available to read
    if (nrf52.available() > 0)
    {
      String data = ""; // Initialize a String to hold the incoming data

      // Read the available data from the serial buffer
      while (nrf52.available() > 0)
      {
        char incomingByte = nrf52.read(); // Read a byte from Serial7
        data += incomingByte;
        // Append the byte to the String
      }
      // Serial.println("BLE DATA : " + data);

      // Check if the data starts with "BLESCAN"
      if (data.startsWith("BLESCAN"))
      {
        // Parse the BLE card number and RSSI
        int firstHash = data.indexOf('#');
        int secondHash = data.indexOf('#', firstHash + 1);

        if (firstHash != -1 && secondHash != -1)
        {
          String bleCardNo = data.substring(firstHash + 1, secondHash);
          String rssiStr = data.substring(secondHash + 1);
          int rssi = rssiStr.toInt(); // Convert RSSI to integer

          // Convert required RSSI to positive and compare
          int positiveRequiredRssi = -requiredBleScanRssi;

          // Print the card number only if the card RSSI is less than the required RSSI
          // and the RSSI is not a single digit, and it's a different card number
          if (rssi < positiveRequiredRssi && abs(rssi) >= 10 && bleCardNo != previousBleCardNo)
          {
            Serial.print("Received BLESCAN data: ");
            Serial.print("BLE Card No: ");
            Serial.print(bleCardNo);
            Serial.print(", RSSI: ");
            Serial.println(rssi);
            nfcType = 3;
            deviceBusyStatus = 2;
            DisplayHomepage(displayLine1, displayLine2);
            SOSflash(200);
            publishNFCData("BLE-Card", bleCardNo);
            // Update the previous card number
            previousBleCardNo = bleCardNo;
          }
        }
        else
        {
          Serial.println("Invalid data format received.");
        }
      }

      // Check if the data starts with "Received data: "
      String prefix = "Received data: ";
      if (data.startsWith(prefix))
      {
        // Extract the data after "Received data: "
        String extractedData = data.substring(prefix.length());
        // Serial.print("Extracted data: ");
        // Serial.println(extractedData);
        ProcessDisplayData(extractedData);
      }
    }

    // Add a small delay to prevent overwhelming the CPU
    delay(10);
  }
}


void getDataFromSubscribedTopic()
{
  String mergedData;
  bool isInHTTPCGET = false;
  

  while (true)
  {
    // Exit thread if device switched to 4G — frees stack/thread resources
    // so audio playback and other operations don't conflict
    if (!enableWifiGsm)
    {
      Serial.println("WiFi listener thread exiting — device on 4G");
      return;
    }

    // Pause listener while loop() is doing WiFi/MQTT reconnection
    // to avoid both threads reading from espSerial simultaneously
    if (espSerialBusy)
    {
      delay(100);
      continue;
    }

    // ---------- ESP Serial listener ----------
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');

      if (response.startsWith("+HTTPCGET"))
      {
        isInHTTPCGET = true;
        int dataSizeIndex = response.indexOf(':') + 1;
        int dataSize = response.substring(dataSizeIndex, response.indexOf(',')).toInt();

        if (dataSize > 0)
        {
          int dataStartIndex = response.indexOf(',') + 1;
          int dataEndIndex = dataStartIndex + dataSize;
          if (dataStartIndex != -1 && dataEndIndex <= response.length())
          {
            String data = response.substring(dataStartIndex, dataEndIndex);
            Serial.println(data);
          }
        }
      }
      else if (isInHTTPCGET && response.startsWith("+"))
      {
        isInHTTPCGET = false;
      }
      else if (response.startsWith("+MQTTSUBRECV:"))
      {
        lastWifiLogPublishTime = millis();
        int topicIndex = response.substring(14, response.indexOf(',')).toInt();
        String topic = response.substring(response.indexOf(',') + 2, response.lastIndexOf('\"'));
        int payloadLength = response.substring(response.lastIndexOf('\"') + 2, response.lastIndexOf(',')).toInt();
        String payload = response.substring(response.lastIndexOf(',') + 1, response.lastIndexOf(',') + 2 + payloadLength);

        Serial.println("Received data from topic: " + topic);
        Serial.println("Payload:" + payload.trim());
        payload.trim();

        if (topic == deviceTopic)
        {
          ProcessDisplayData(payload);
        }
        else if (topic == deviceReactTopic)
        {
          if (payload == pushApiDataMqttStatus)
          {
            Serial.println("device will push mqtt data here");
            delay(3000);
            publishDeviceLogData();
          }
          else if (payload == reloadApiDataMqttStatus)
          {
            Serial.println("device will recall api data here");
            teensyDoReboot();
          }
          else if (payload.startsWith("TOF#"))
          {
            String tofData = payload.substring(4);
            int firstHashPos = tofData.indexOf('#');
            int secondHashPos = tofData.indexOf('#', firstHashPos + 1);
            int thirdHashPos = tofData.indexOf('#', secondHashPos + 1);

            bleTxPower = tofData.substring(0, firstHashPos).toInt();
            tofSensorThreshold = tofData.substring(firstHashPos + 1, secondHashPos).toInt();
            advertiseInterval = tofData.substring(secondHashPos + 1, thirdHashPos).toInt();
            TofModes = tofData.substring(thirdHashPos + 1).toInt();

            Serial.println("TOF Data:");
            Serial.println("bleTxPower: " + String(bleTxPower));
            Serial.println("tofSensorThreshold: " + String(tofSensorThreshold));
            Serial.println("advertiseInterval: " + String(advertiseInterval));
            Serial.println("tofMode: " + String(TofModes));
            DisplayInfo("BLE : " + String(bleTxPower), "Feet : " + String(tofSensorThreshold), "TOF : " + String(advertiseInterval));
            delay(2000);
            DisplayHomepage(displayLine1, displayLine2);
            handleTofModes(TofModes);
          }
          else if (payload.startsWith("WIFIRSSI"))
          {
            deviceBusyStatus = 0;
            DisplayHomepage(displayLine1, displayLine2);
            String wifiNetworksJSON = getWifiNetworksJSON(10000);
            Serial.println("Wi-Fi Networks in JSON format:");
            Serial.println(wifiNetworksJSON);
            publishRawToTopic(deviceNetworkTopic, wifiNetworksJSON, wifiNetworksJSON.length());
            deviceBusyStatus = 1;
            DisplayHomepage(displayLine1, displayLine2);
          }
          else if (payload.startsWith("GAIN#"))
          {
            gainValue = payload.substring(5).toInt();
            if (gainValue >= 0 && gainValue <= 100)
            {
              Serial.println("Setting gain to: " + String(gainValue));
              mixer1.gain(0, gainValue);
              mixer2.gain(0, gainValue);
            }
            else
            {
              Serial.println("Invalid gain value in GAIN payload");
            }
          }
          else
          {
            Serial.println("Unknown data for device react topic");
          }
        }
      }
      else if (response.startsWith("+MQTTDISCONNECTED:0"))
      {
        Serial.println("MQTT Disconnected!");
        mqttConnected = false;
        lightUpLED(1, 2, 2, 0);
        deviceBusyStatus = 0;
        DisplayHomepage(displayLine1, displayLine2);
        // Don't call connectToMQTT() here - let loop() handle it
        // to avoid serial port contention between threads
      }
      else if (response.indexOf("WIFI DISCONNECT") != -1)
      {
        Serial.println("Wi-Fi Disconnected!");
        wifiConnected = false;
        mqttConnected = false;
        lightUpLED(1, 2, 2, 0);
        deviceBusyStatus = 0;
        DisplayHomepage(displayLine1, displayLine2);
      }
      else if (response.indexOf("WIFI GOT IP") != -1)
      {
        Serial.println("Wi-Fi got reconnected");
        wifiConnected = true;
        // Don't call connectToMQTT() here - let loop() handle it
        // to avoid serial port contention between threads
      }
    }

    // ---------- Periodic MQTT Health Check (detect internet loss in idle mode) ----------
    if (mqttConnected && !espSerialBusy)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - lastMqttHealthCheck >= mqttHealthCheckInterval)
      {
        lastMqttHealthCheck = currentMillis;
        espSerial.println("AT+MQTTCONN?");
        unsigned long startTime = millis();
        bool mqttStillAlive = false;
        while (millis() - startTime < 3000)
        {
          if (espSerial.available())
          {
            String hcResp = espSerial.readStringUntil('\n');
            hcResp.trim();
            if (hcResp.startsWith("+MQTTCONN:"))
            {
              int commaIdx = hcResp.indexOf(',');
              int secondComma = hcResp.indexOf(',', commaIdx + 1);
              int state = hcResp.substring(commaIdx + 1, secondComma).toInt();
              mqttStillAlive = (state >= 3);
              break;
            }
            else if (hcResp.startsWith("+MQTTSUBRECV:"))
            {
              bufferMqttRecv(hcResp);
            }
            else if (hcResp.indexOf("WIFI DISCONNECT") != -1)
            {
              wifiConnected = false;
              mqttConnected = false;
              deviceBusyStatus = 0;
              lightUpLED(1, 2, 2, 0);
              DisplayHomepage(displayLine1, displayLine2);
              Serial.println("WiFi lost during health check!");
              break;
            }
          }
        }
        if (!mqttStillAlive && mqttConnected)
        {
          Serial.println("MQTT health check: connection lost!");
          mqttConnected = false;
          deviceBusyStatus = 0;
          lightUpLED(1, 2, 2, 0);
          DisplayHomepage(displayLine1, displayLine2);
        }
        processBufferedMqttMessages(); // Process any messages caught during health check
      }
    }

    // ---------- Process WiFi MQTT Queue (tickets first, then logs) ----------
    if (mqttConnected && !espSerialBusy)
    {
      int nextItem = getNextWifiQueueItem();
      if (nextItem != -1)
      {
        String qTopic = wifiQueue[nextItem].topic;
        String qPayload = wifiQueue[nextItem].payload;
        int qPriority = wifiQueue[nextItem].priority;
        wifiQueue[nextItem].occupied = false; // Free slot before sending

        Serial.println("WiFi queue sending [" + String(qPriority == QUEUE_PRIORITY_HIGH ? "TICKET" : "LOG") + "]: " + qTopic);
        publishNFCDataToTopic(qTopic, qPayload, qPayload.length());
      }
    }

    // ---------- Periodic Log Publishing (skip if ticket is queued) ----------
    if (mqttConnected && getNextWifiQueueItem() == -1)
    {
      if (totalDeviceLog && deviceTicketTelematriceMode)
      {
        unsigned long currentMillis = millis();
        if (currentMillis - lastWifiLogPublishTime >= wifiLogPublishInterval)
        {
          lastWifiLogPublishTime = currentMillis;
          publishDeviceLogData();
        }
      }
      else if (deviceTicketTelematricHybridMode == 1)
      {
        unsigned long currentMillis = millis();
        if (currentMillis - lastWifiLogPublishTime >= wifiLogPublishInterval)
        {
          lastWifiLogPublishTime = currentMillis;
          publishDeviceLogData();
        }
      }
    }

  }

  espSerial.flush();
}






bool sendATCommand(const String &command, const String &expectedResponse)
{
  delay(2000);
  espSerial.print(command); // Send the AT command
  espSerial.print("\r\n");  // Send newline and carriage return
  delay(100);               // Wait for 100 milliseconds

  unsigned long startMillis = millis();
  while (millis() - startMillis < 1000)
  { // Wait for response for up to 3 seconds
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      // Serial.print("Received response: ");
      // Serial.println(response);

      if (response.indexOf(expectedResponse) != -1)
      {
        return true; // Received the expected response
      }
      else if (response.indexOf("WIFI DISCONNECT") != -1)
      {
        Serial.println("Wi-Fi Disconnected (during sendATCommand)!");
        wifiConnected = false;
        mqttConnected = false;
        return false;
      }
    }
  }

  return false; // Timeout: Did not receive the expected response
}

void checkWiFiStatus()
{
  while (true)
  {
    delay(3000);
    espSerial.println("AT+CWSTATE?"); // Send AT command to check WiFi status
    if (espSerial.find("+CWSTATE:"))
    {
      int state = espSerial.parseInt();
      switch (state)
      {
      case 0:
        Serial.println("WiFi state: Not connected to any network");
        break;
      case 1:
        Serial.println("WiFi state: Connected to AP, acquiring IP address");
        break;
      case 2:
        Serial.println("WiFi state: Connected to AP, IP address acquired");
        break;
      case 3:
        Serial.println("WiFi state: Connecting or reconnecting");
        break;
      case 4:
        Serial.println("WiFi state: Disconnected");
        break;
      default:
        Serial.println("Unknown WiFi state");
        break;
      }
    }
  }
}

void connectToWiFi(String ssid, String password)
{
  espSerial.println("AT+CWJAP=\"" + String(ssid) + "\",\"" + String(password) + "\"");
  unsigned long delayTime = 1000; // Delay set to 1 second

  for (int i = 0; i < 10; i++)
  {
    if (espSerial.available())
    {
      String response = espSerial.readStringUntil('\n');
      // Serial.println(response);
      if (response.indexOf("WIFI CONNECTED") != -1)
      {
        if (firstTimeDeviceLog)
        {
          lightUpLED(1, 2, 2, 0);
          delay(500);
          DisplayInfo("WIFI found", ssid, codeVersion);
        }
        Serial.println("WiFi Connected!");
        delay(500);
        wifiConnected = true;
        if (!firstTimeDeviceLog)
        {
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
        break;
      }
      else if (response.indexOf("ERROR") != -1)
      {
        Serial.println("WiFi Connection Error!");
        break;
      }
    }

    delay(delayTime);
  }

  if (!wifiConnected)
  {
    Serial.println("Timeout: WiFi Connection Failed!");
  }
}

void restartEsp32Module()
{
  espSerial.println("AT+RST");
  // espSerial.println("AT+CWQAP");
  if (espSerial.find("OK"))
  {
    Serial.println("Esp32 Module restarted ");
    lightUpLED(1, 1, 2, 0);
  }
}
void enableEG25GPSModule()
{
  String command = "AT+QGPS=1";   // AT command to check GPS module status
  String expectedResponse = "OK"; // Expected response
  unsigned long timeout = 6000;   // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command

  if (response == command)
  { // Command echo received, now expect +QGPS: 1
    response = sendATCommand3("", expectedResponse, timeout);
  }
  // Serial.println("The response is : " + response);
  if (response == expectedResponse)
  {
    Serial.println("GPS module is found");
    gpsEnabled = true;
  }
  else
  {
    // Serial.println("GPS module is not enabled");
    gpsEnabled = false;
  }
}

void mqttLogPusher()
{
  while (1)
  {
    publishDeviceLogData();
    delay(mqttLogDelay);
    if (gsmMqttConnected)
    {
      playTicketSoundAndLight(5);
    }
  }
}
void hexPortLisener()
{
  while (true)
  {
    static int dataSizeLeft = 0;
    static bool isInHTTPCGET = false;
    static String buffer = ""; // Buffer to hold incoming characters

    while (espSerial.available())
    {
      char receivedChar = espSerial.read();

      // Check if receivedChar is a line break or the buffer is full
      if (receivedChar == '\n' || buffer.length() > 500)
      {
        String response = buffer;
        buffer = "";     // Clear the buffer
        response.trim(); // Remove any leading/trailing white spaces or new lines

        if (response.startsWith("+HTTPCGET"))
        {
          isInHTTPCGET = true;

          // Extract the data size from the line
          int dataSizeIndex = response.indexOf(':') + 1;
          dataSizeLeft = response.substring(dataSizeIndex, response.indexOf(',')).toInt();

          // Extract any data that might be on this line
          int dataStartIndex = response.indexOf(',') + 1;
          if (dataStartIndex != -1 && dataStartIndex < response.length())
          {
            String data = response.substring(dataStartIndex);
            dataSizeLeft -= data.length();

            // print in a new line including ':'
            for (int i = 0; i < data.length(); i++)
            {
              if (data[i] == ':')
              {
                Serial.println();
              }
              Serial.print(data[i]);
            }
          }
        }
        else if (isInHTTPCGET)
        {
          // Print the data directly
          dataSizeLeft -= response.length();

          // print in a new line including ':'
          for (int i = 0; i < response.length(); i++)
          {
            if (response[i] == ':')
            {
              Serial.println();
            }
            Serial.print(response[i]);
          }

          if (response.endsWith(":00000001FF"))
          {
            isInHTTPCGET = false;                // Exit from +HTTPCGET part
            Serial.println();                    // New line after the completion of data transfer
            Serial.println("Response is ended"); // Indicate the end of the response
          }
        }
      }
      else
      {
        // Append received characters to buffer
        buffer += receivedChar;
      }
    }
  }
}
void bleNrf52Init(String command)
{
  const int timeout_ms = 5000; // 5 seconds timeout
  String response;

  // Send command to NRF52
  nrf52.println(command);

  // Wait for a response with timeout
  long startTime = millis();
  while ((millis() - startTime) < timeout_ms)
  {
    if (nrf52.available())
    {
      response += (char)nrf52.read();
    }
  }

  // If there's no response, print a message and return
  if (response.length() == 0)
  {
    Serial.println("No response from NRF52");
    bleAdStatus = false;
    return;
  }

  // Extract names
  int commandStartIndex = command.indexOf('#') + 1;
  int commandEndIndex = command.indexOf('#', commandStartIndex);
  String commandName = command.substring(commandStartIndex, commandEndIndex).trim();
  // Serial.println("The command name " + commandName);

  String identifier = "Starting BLE advertising with name: ";
  int responseStartIndex = response.indexOf(identifier) + identifier.length();
  String responseName = response.substring(responseStartIndex).trim();
  // Serial.println("The response name " + responseName);

  // Compare names and print result
  if (commandName.equals(responseName))
  {
    Serial.println("BLE verified");
    bleAdStatus = true;
  }
  else
  {
    Serial.println("BLE not verified");
    // bleAdStatus = false;
  }
}

bool waitForResponse(const String &expectedResponse, unsigned long timeout)
{
  unsigned long startTime = millis();

  while (millis() - startTime < timeout)
  {
    if (eg25.available())
    {
      String response = eg25.readStringUntil('\n');

      if (response.indexOf(expectedResponse) != -1)
      {
        return true;
      }
    }
  }

  return false;
}
int getWifiRSSI()
{
  String response = sendWifiATCommands("AT+CWJAP?", "+CWJAP:", 5000);

  if (response == "TIMEOUT")
  {
    // Handle the timeout case
    return -1; // Indicate an error or timeout with -1
  }
  else if (response == "ERROR")
  {
    // Handle the error case
    return -1; // Indicate an error with -1
  }
  else
  {
    // Parse the RSSI value from the response
    int pos1 = response.indexOf("-");           // Find the first "-"
    int pos2 = response.indexOf(",", pos1 + 1); // Find the following ","
    if (pos1 != -1 && pos2 != -1)
    {
      String rssiStr = response.substring(pos1, pos2); // Extract the RSSI value
      wifiRSSI = rssiStr.toInt();                      // Store the RSSI value as an integer
      return wifiRSSI;
    }

    // Return -1 to indicate an error if parsing fails
    return -1;
  }
}

// Get BSSID (router MAC address) from connected WiFi
// Response format: +CWJAP:<ssid>,<bssid>,<channel>,<rssi>
String getWifiBSSID()
{
  String response = sendWifiATCommands("AT+CWJAP?", "+CWJAP:", 5000);

  if (response == "TIMEOUT" || response == "ERROR")
  {
    return "";
  }

  // Find BSSID between first and second comma after SSID
  // +CWJAP:"SSID","aa:bb:cc:dd:ee:ff",channel,rssi
  int firstQuoteEnd = response.indexOf("\",\"");
  if (firstQuoteEnd != -1)
  {
    int bssidStart = firstQuoteEnd + 3; // skip ","
    int bssidEnd = response.indexOf("\"", bssidStart);
    if (bssidEnd != -1)
    {
      return response.substring(bssidStart, bssidEnd);
    }
  }
  return "";
}
int getGsmRssiData()
{
  String response = sendATCommand3("AT+CSQ", "+CSQ:", 5000);

  if (response == "TIMEOUT")
  {
    // Handle the timeout case
    return -1; // Indicate an error or timeout with -1
  }
  else if (response == "ERROR")
  {
    // Handle the error case
    return -1; // Indicate an error with -1
  }
  else
  {
    // Parse the RSSI value from the response
    int pos1 = response.indexOf(":");
    int pos2 = response.indexOf(",");
    if (pos1 != -1 && pos2 != -1)
    {
      String rssiStr = response.substring(pos1 + 1, pos2);
      gsmRSSI = rssiStr.toInt(); // Store the RSSI value as an integer
      return gsmRSSI;
    }

    // Return -1 to indicate an error if parsing fails
    return -1;
  }
}
String getEG25DeviceModel()
{
  String command = "AT+GMR";
  String expectedResponsePrefix = "EG";
  unsigned long timeout = 5000; // 5 seconds timeout

  // Send the AT command to get device model
  String response = sendATCommand3(command, expectedResponsePrefix, timeout);

  // Check if the response starts with the expected prefix
  if (response.startsWith(expectedResponsePrefix))
  {
    // Extract the device model from the response
    response.trim(); // Remove leading/trailing spaces
    return response;
  }
  else
  {
    // Handle unexpected response
    return "Unknown"; // Return an "Unknown" model in case of unexpected response
  }
}

void checkSimCardStatus(bool networkMode)
{
  const String simStatusCommand = "AT+QSIMSTAT?";
  const String simStatusPrefix = "+QSIMSTAT:";
  const String iccidCommand = "AT+QCCID";
  const String iccidPrefix = "+QCCID:";
  const unsigned long timeout = 5000; // 5 seconds timeout
  const String simInsertedValue = "1";

  // Send the AT command to check SIM card status
  String response = sendATCommand3(simStatusCommand, simStatusPrefix, timeout);

  // Check if the response starts with the expected prefix
  if (response.startsWith(simStatusPrefix))
  {
    // Extract the enable and inserted status values from the response
    response.remove(0, simStatusPrefix.length()); // Remove the prefix
    response.trim();                              // Remove leading/trailing spaces

    int commaIndex = response.indexOf(',');
    if (commaIndex != -1)
    {
      String enableStatus = response.substring(0, commaIndex);
      String insertedStatus = response.substring(commaIndex + 1);

      // Check if the SIM card is present (insertedStatus == simInsertedValue)
      if (insertedStatus == simInsertedValue)
      {
        // Optionally display information if networkMode is true
        if (networkMode)
        {
          DisplayInfo("LTE", "SIM found", codeVersion);
          
        }
        Serial.println("SIM card is present");
        gsmSimFound = true;
        delay(2000);

        // Get the ICCID from the SIM card
        response = sendATCommand3(iccidCommand, iccidPrefix, timeout);

        if (response.startsWith(iccidPrefix))
        {
          // Extract the ICCID from the response
          simNumber = response.substring(iccidPrefix.length());
          simNumber.trim(); // Remove any leading/trailing spaces
          Serial.println("SIM card ICCID: " + simNumber);
        }
        else
        {
          Serial.println("Failed to retrieve ICCID.");
          gsmSimFound = false;
        }
      }
      else
      {
        Serial.println("Please insert the SIM card");
        if (networkMode)
        {
          displayCommand(insertSimDisplayPage);
           gsmSimFound = false;
        }
      }
    }
  }
  else
  {
    // Handle unexpected response
    Serial.println("Unexpected response: " + response);
    if (networkMode)
    {
      displayCommand(insertSimDisplayPage);
     gsmSimFound = false;
    }
  }
}

void activatePDP()
{
  String command = "AT+QIACT=1";  // AT command to deactivate PDP context
  String expectedResponse = "OK"; // Expected response
  unsigned long timeout = 6000;   // 6 seconds timeout

  String response = sendATCommand3(command, expectedResponse, timeout);

  if (response == expectedResponse)
  {
    Serial.println("PDP context activated successfully");
  }
  else if (response == "ERROR")
  {
    Serial.println("Failed to deactivate PDP context");
  }
  else
  {
    Serial.println("Failed to send AT command");
  }
}

// Function to get the network the SIM card is connected to
void getConnectedNetwork()
{
  String command = "AT+COPS?";
  String expectedResponsePrefix = "+COPS:";
  unsigned long timeout = 5000; // 5 seconds timeout

  // Send the AT command to get connected network information
  String response = sendATCommand3(command, expectedResponsePrefix, timeout);

  // Check if the response starts with the expected prefix
  if (response.startsWith(expectedResponsePrefix))
  {
    // Extract the operator name and network type from the response
    response.remove(0, expectedResponsePrefix.length()); // Remove the prefix
    response.trim();                                     // Remove leading/trailing spaces

    // Find the first quote character
    int firstQuoteIndex = response.indexOf('\"');
    if (firstQuoteIndex != -1)
    {
      // Find the second quote character
      int secondQuoteIndex = response.indexOf('\"', firstQuoteIndex + 1);
      if (secondQuoteIndex != -1)
      {
        // Extract the operator name between the quotes
        GsmOperatorName = response.substring(firstQuoteIndex + 1, secondQuoteIndex);

        // Find the last number in the response
        int lastIndex = response.lastIndexOf(',');
        if (lastIndex != -1)
        {
          // Extract the network type as the last number
          String networkTypeStr = response.substring(lastIndex + 1);
          int networkType = networkTypeStr.toInt(); // Get the network type as an integer

          // Interpret the network type based on your requirements
          switch (networkType)
          {
          case 0:
            GsmNetworkTypeString = "GSM";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 2:
            GsmNetworkTypeString = "UTRAN";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 3:
            GsmNetworkTypeString = "GSM";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 4:
            GsmNetworkTypeString = "UTRAN";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 5:
            GsmNetworkTypeString = "LUTRAN";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 6:
            GsmNetworkTypeString = "HSUPA";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 7:
            GsmNetworkTypeString = "EUTRAN";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          case 100:
            GsmNetworkTypeString = "CDMA";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          default:
            GsmNetworkTypeString = "N/A";
            Serial.println("Sim internet connection is " + GsmNetworkTypeString);
            break;
          }
          return;
        }
      }
    }
  }

  // Handle unexpected response
  GsmOperatorName = "Unknown";
  GsmNetworkTypeString = "Unknown";
  Serial.println("Could not retrieve connected network information");
}
void resetMQTT()
{
  String command = "AT+QMTDISC=0"; // AT command to reset MQTT
  String expectedResponse = "OK";  // Expected response
  unsigned long timeout = 6000;    // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command

  if (response == command)
  { // Command echo received, now expect OK
    response = sendATCommand3("", expectedResponse, timeout);
  }

  if (response == expectedResponse)
  {
    Serial.println("MQTT reset successfully");
  }
  else
  {
    Serial.println("Failed to reset MQTT");
  }
}

void setMQTTMode()
{
  String command = "AT+QMTCFG=\"recv/mode\",0,0,1"; // AT command to set MQTT mode
  String expectedResponse = "OK";                   // Expected response
  unsigned long timeout = 6000;                     // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command

  if (response == command)
  { // Command echo received, now expect OK
    response = sendATCommand3("", expectedResponse, timeout);
  }

  if (response == expectedResponse)
  {
    Serial.println("MQTT mode set successfully");
  }
  else
  {
    Serial.println("Failed to set MQTT mode");
  }
}

void openMQTT()
{
  String command = "AT+QMTOPEN=3,\"" + String(mqttServer) + "\",1883"; // AT command to open MQTT connection
  String expectedResponse = "+QMTOPEN: 3,";                  // Expected response prefix
  unsigned long timeout = 6000;                              // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command

  if (response == command)
  { // Command echo received, now expect +QMTOPEN: 3,
    response = sendATCommand3("", expectedResponse, timeout);
  }

  if (response.startsWith(expectedResponse))
  {
    // If the command is successfully acknowledged, read the MQTT connection status
    char mqttStatus = response.charAt(response.length() - 1); // Last character is MQTT status

    if (mqttStatus == '0')
    {
      Serial.println("MQTT connected successfully");
    }
    else
    {
      Serial.println("MQTT connection failed or no network");
    }
  }
  else
  {
    Serial.println("Failed to send AT command");
  }
}

void connectMQTT(bool reconnectUntilSuccessful)
{
  while (true)
  {
    wdt.feed();
    String revicevedMqttClientId = generateRandomString(4);
    String clientId = deviceTeensyMacAddress + "-" + "GSM" + "-" + revicevedMqttClientId + teensySN();
    Serial.println("GSM Mqtt client id : " + clientId);

    String command = "AT+QMTCONN=3,\"" + clientId + "\",\"Validator\",\"Validator@4U!\""; // AT command to connect to MQTT
    String expectedResponse = "+QMTCONN: 3,0,0";                    // Expected response
    unsigned long timeout = 6000;                                   // 6 seconds timeout

    String response = sendATCommand3(command, command, timeout); // Expect echo of the command

    if (response == command)
    { // Command echo received, now expect +QMTCONN: 3,0,0
      response = sendATCommand3("", expectedResponse, timeout);
    }

    if (response == expectedResponse)
    {
      Serial.println("MQTT connected to specific client ID successfully");
      // if (!reconnectUntilSuccessful)
      // {
      //   displayCommand(zigNetworkFoundDisplayPage);
      //   lightUpLED(2, 2, 2, 1);
      //   delay(3000);
      //   lightUpLED(0, 0, 0, 0);
      // }
      delay(1000);
      return; // Exit the function on successful connection
    }
    else
    {
      Serial.println("Failed to connect to MQTT with specific client ID");

      if (!reconnectUntilSuccessful)
      {
        // Non-looping mode: return on failure so caller can handle it
        // (e.g. failover can revert to WiFi offline mode)
        Serial.println("MQTT connection failed — returning to caller.");
        return;
      }
      // DisplayInfo("LTE", "ZIG reconnecting", codeVersion);
      delay(3000); // Add a 3-second delay before retrying
    }
  }
}

void subscribeMQTT(const String &topic)
{
  String command = "AT+QMTSUB=3,1,\"" + topic + "\",0"; // AT command to subscribe to MQTT topic
  String expectedResponse = "+QMTSUB: 3,1,0,0";         // Expected response
  unsigned long timeout = 6000;                         // 6 seconds timeout

  String response = sendATCommand3(command, command, timeout); // Expect echo of the command
  

  if (response == command)
  { // Command echo received, now expect +QMTSUB: 3,1,0,0
    response = sendATCommand3("", expectedResponse, timeout);
  }

  if (response == expectedResponse)
  {
    Serial.println("Successfully subscribed to topic " + topic);
    gsmMqttConnected = true;
    enableWifiGsm = false;
    displayLine1 = GsmNetworkTypeString;
    displayLine2 = GsmOperatorName;
  }
  else
  {
    if (firstTimeDeviceLog)
    {
      Serial.println("Failed to subscribe to topic " + topic);
      displayCommand(invaildDataDisplayPage);
      while (true)
      {
        // Halt the program
      }
    }
  }
}
// Function to set Internet registration status to "1" with retries
void setInternetRegistration()
{
  String command = "AT+CREG=1";
  String expectedResponse = "OK";
  unsigned long timeout = 5000; // 5 seconds timeout
  int maxRetries = 3;           // Maximum number of retries

  for (int retryCount = 0; retryCount < maxRetries; retryCount++)
  {
    String response = sendATCommand3(command, expectedResponse, timeout);

    if (response == expectedResponse)
    {
      // Setting Internet registration successful
      Serial.println("Sim network success");
      return; // Exit the function on success
    }
    else
    {
      // Handle command failure or timeout
      Serial.println("Failed to set sim network");
    }
  }

  // If setting Internet registration fails after maxRetries attempts, you can handle it here.
  Serial.println("Could not set sim network to '1'");
  // Optionally add a delay before retrying
  // delay(1000); // 1-second delay before retry
}
// Function to activate the internet with retry and timeout
void activateInternet()
{
  String internetCommand = "AT+CREG?";
  String expectedResponse1 = "+CREG: 1,1"; // Registered, home network
  String expectedResponse2 = "+CREG: 1,5"; // Registered, roaming
  unsigned long timeout = 5000;            // 5 seconds timeout
  int maxRetries = 10;                     // Maximum number of retries

  // Check network registration status
  for (int retryCount = 0; retryCount < maxRetries; retryCount++)
  {
    wdt.feed();
    String response = sendATCommand3(internetCommand, "+CREG:", timeout);

    if (response.startsWith(expectedResponse1))
    {
      // Registered, home network
      Serial.println("Registered, home network");
      DisplayInfo("LTE", "Home network", codeVersion);
      gsmInternetStatus = true;
      return;
    }
    else if (response.startsWith(expectedResponse2))
    {
      // Registered, roaming
      Serial.println("Registered, roaming");
      DisplayInfo("LTE", "Roaming network", codeVersion);
      gsmInternetStatus = true;
      return;
    }
    else if (response == "TIMEOUT")
    {
      // Handle timeout, optionally add a delay between retries if needed
      // delay(1000); // 1-second delay between retries
    }
    else
    {
      // Handle unexpected response or error
      Serial.println("Could not get the sim network");
    }
  }

  // Internet activation failed after maxRetries attempts
  Serial.println("Could not activate the internet");
  displayCommand(zigNetworkFailedDisplayPage);
  gsmInternetStatus = false;
  return;
  // Legacy halt removed — caller checks gsmInternetStatus
}
void gsmMqttConnectFlow(bool reconnectUntilSuccessful)
{
  // activatePDP();
  if (reconnectUntilSuccessful)
  {
    deviceBusyStatus = 0;
    DisplayHomepage(displayLine1, displayLine2);
  }
  delay(500);
  wdt.feed();
  getConnectedNetwork();
  delay(500);
  wdt.feed();
  disconnectPreviousMqtt();
  delay(1500);
  wdt.feed();
  resetMQTT();
  delay(500);
  wdt.feed();
  setMQTTMode();
  delay(500);
  wdt.feed();
  openMQTT();
  delay(500);
  wdt.feed();
  connectMQTT(reconnectUntilSuccessful);
  wdt.feed();
  delay(1500);
  wdt.feed();
  subscribeMQTT(deviceTopic);
  wdt.feed();
  delay(1500);
  wdt.feed();
  subscribeMQTT(deviceReactTopic);
  if (!firstTimeDeviceLog)
  {
    displayLine1 = GsmNetworkTypeString;
    displayLine2 = GsmOperatorName;
    deviceBusyStatus = 1;
    DisplayHomepage(displayLine1, displayLine2);
  }
}

// Single-attempt MQTT reconnect for 4G (non-blocking, no infinite loops)
void gsmMqttConnectFlowSingleAttempt()
{
  delay(500);
  getConnectedNetwork();
  delay(500);
  disconnectPreviousMqtt();
  delay(1500);
  resetMQTT();
  delay(500);
  setMQTTMode();
  delay(500);
  openMQTT();
  delay(500);

  // Single attempt to connect MQTT
  String revicevedMqttClientId = generateRandomString(4);
  String clientId = deviceTeensyMacAddress + "-" + "GSM" + "-" + revicevedMqttClientId + teensySN();
  Serial.println("GSM Mqtt client id : " + clientId);
  String command = "AT+QMTCONN=3,\"" + clientId + "\",\"Validator\",\"Validator@4U!\"";
  String expectedResponse = "+QMTCONN: 3,0,0";
  unsigned long timeout = 6000;
  String response = sendATCommand3(command, command, timeout);
  if (response == command)
  {
    response = sendATCommand3("", expectedResponse, timeout);
  }
  if (response != expectedResponse)
  {
    Serial.println("4G MQTT single connect attempt failed");
    return; // gsmMqttConnected stays false
  }
  Serial.println("4G MQTT connected, subscribing...");
  delay(1500);
  subscribeMQTT(deviceTopic);
  delay(1500);
  subscribeMQTT(deviceReactTopic);
  // gsmMqttConnected is set to true inside subscribeMQTT() on success
}

// Phase 1: Verify 4G MQTT is reachable while device is running on WiFi.
// Non-halting — returns false on any failure. Does NOT switch to 4G.
// Sets eg25ReadyForFailover = true if 4G MQTT connects and disconnects cleanly.
bool check4GAvailability()
{
  Serial.println("___________4G Availability Check (Phase 1)___________");

  // Step 1: Non-halting APN + PDP context activation
  bool apnOk = false;
  for (int i = 0; i < 5; i++)
  {
    wdt.feed();
    String r = sendATCommand3("AT+CGDCONT=1,\"IP\",\"iot.kore.com\"", "OK", 5000);
    if (r == "OK")
    {
      String r2 = sendATCommand3("AT+CGACT=1,1", "OK", 5000);
      if (r2 == "OK")
      {
        apnOk = true;
        Serial.println("[4G Check] APN/PDP OK");
        break;
      }
    }
    delay(500);
  }
  if (!apnOk)
  {
    Serial.println("[4G Check] APN failed — 4G not available");
    return false;
  }

  // Step 2: Network registration enable (returns normally, no halt)
  setInternetRegistration();

  // Step 3: Non-halting network registration check
  bool netOk = false;
  for (int i = 0; i < 5; i++)
  {
    wdt.feed();
    String r = sendATCommand3("AT+CREG?", "+CREG:", 5000);
    if (r.startsWith("+CREG: 1,1") || r.startsWith("+CREG: 1,5"))
    {
      netOk = true;
      Serial.println("[4G Check] Network registered OK");
      break;
    }
    delay(1000);
  }
  if (!netOk)
  {
    Serial.println("[4G Check] Network not registered — 4G not available");
    return false;
  }

  // Step 4: Open MQTT connection (check only — no subscribe)
  wdt.feed();
  disconnectPreviousMqtt();
  delay(500);
  wdt.feed();
  resetMQTT();
  delay(500);
  wdt.feed();
  setMQTTMode();
  delay(500);
  wdt.feed();
  openMQTT();
  delay(500);
  wdt.feed();

  // Step 5: Single MQTT connect attempt
  String clientId = deviceTeensyMacAddress + "-CHECK-" + generateRandomString(4) + teensySN();
  String command = "AT+QMTCONN=3,\"" + clientId + "\",\"Validator\",\"Validator@4U!\"";
  String response = sendATCommand3(command, command, 6000);
  wdt.feed();
  if (response == command)
  {
    response = sendATCommand3("", "+QMTCONN: 3,0,0", 6000);
  }
  wdt.feed();

  if (response != "+QMTCONN: 3,0,0")
  {
    Serial.println("[4G Check] MQTT connect failed — 4G not ready for failover");
    disconnectPreviousMqtt();
    return false;
  }

  // Step 6: Immediately disconnect — we verified it works, but we don't use it yet
  delay(200);
  disconnectPreviousMqtt();
  delay(500);

  Serial.println("[4G Check] 4G MQTT verified OK — ready for failover");
  eg25ReadyForFailover = true;
  eg25moduleconnected = true;
  return true;
}

void restartEg25MqttFlow()
{
  DisplayInfo("LTE", "ZIG Reconnecting", codeVersion);
  delay(2000);
  setInternetRegistration();
  delay(1000);
  activateInternet();
  delay(500);
  gsmMqttConnectFlow(true);
}

void bleAdvertisebegin()
{
  if (!wifiApiDataState)
  {
    bleConnectMode = true;
    // unsigned int eepromAddress = 100;
    // EEPROM.get(eepromAddress, retrievedBLEMajor);
    // eepromAddress += sizeof(retrievedBLEMajor);
    // EEPROM.get(eepromAddress, retrievedBLEMinor);
    // eepromAddress += sizeof(retrievedBLEMinor);
    // EEPROM.get(eepromAddress, retrievedBLETxPower);

    // Print the retrieved values
    Serial.println("Retrieved BLE Data from STATIC DATA:");
    Serial.print("Retrieved BLE Major: ");
    Serial.println(retrievedBLEMajor);
    ibeaconMajor = retrievedBLEMajor;
    Serial.print("Retrieved BLE Minor: ");
    Serial.println(retrievedBLEMinor);
    ibeaconMinor = retrievedBLEMinor;
    Serial.print("Retrieved BLE TX Power: ");
    Serial.println(retrievedBLETxPower);
    bleTxPower = retrievedBLETxPower;
    Serial.print("Retrieved Rssiat1m: ");
    Serial.println(retrivedrssi);
    rssiat1m = retrivedrssi;
    bleNrf52Command = bleStartCommand + "#" + blename + "#" + String(bleTxPower) + "#" +
                      (bleDfuMode ? "true" : "false") + "#" + String(ibeaconMajor) + "#" +
                      String(ibeaconMinor) + "#" + (bleIbeaconMode ? "true" : "false") + "#" + (bleScanMode ? "true" : "false") + "#" +
                      (bleConnectMode ? "true" : "false")+ "#"+String(rssiat1m);;
  }
  if (bleStartOnlyOneTime)
  {
    Serial.println("+++++++++++++BLE process begin++++++++++++++");
    bleNrf52Init(bleNrf52Command);
    // threads.addThread(nrf52SerialPortListener);
    bleStartOnlyOneTime = false;
  }
}


void gsmPortLisener()
{


  while (true)
  {
    // ---------- Existing EG25 listener ----------
    if (eg25.available())
    {
      String receivedString = eg25.readStringUntil('\n'); // Read the entire line
      if (receivedString.startsWith("+QMTRECV"))
      {
        int lastCommaIndex = receivedString.lastIndexOf(',');
        lastGsmLogPublishTime = millis();
        String mqttTopic = receivedString.substring(receivedString.indexOf('"', 10) + 1, receivedString.indexOf('"', 15));
        String mqttData = receivedString.substring(lastCommaIndex + 1);
        mqttData.replace("\"", "");
        if (mqttTopic.equals(deviceTopic))
        {
          Serial.println("Received MQTT validation topic: " + mqttTopic);
          Serial.println("Received MQTT data: " + mqttData);
          ProcessDisplayData(mqttData);
        }
        else if (mqttTopic.equals(deviceReactTopic))
        {
          Serial.println("Received MQTT API topic: " + mqttTopic);
          Serial.println("Received MQTT API data: " + mqttData);
          mqttData.trim();
          if (mqttData == pushApiDataMqttStatus)
          {
            Serial.println("device will push mqtt data here");
            publishDeviceLogData();
          }
          else if (mqttData == reloadApiDataMqttStatus)
          {
            Serial.println("device restart here");
            teensyDoReboot();
          }
          else if (mqttData.startsWith("TOF"))
          {
            String tofData = mqttData.substring(4);
            int firstHashPos = tofData.indexOf('#');
            int secondHashPos = tofData.indexOf('#', firstHashPos + 1);
            int thirdHashPos = tofData.indexOf('#', secondHashPos + 1);
            bleTxPower = tofData.substring(0, firstHashPos).toInt();
            tofSensorThreshold = tofData.substring(firstHashPos + 1, secondHashPos).toInt();
            advertiseInterval = tofData.substring(secondHashPos + 1, thirdHashPos).toInt();
            TofModes = tofData.substring(thirdHashPos + 1).toInt();
            Serial.println("TOF Data:");
            Serial.println("bleTxPower: " + String(bleTxPower));
            Serial.println("tofSensorThreshold: " + String(tofSensorThreshold));
            Serial.println("advertiseInterval: " + String(advertiseInterval));
            Serial.println("tofMode: " + String(TofModes));
            DisplayInfo("BLE : " + String(bleTxPower), "Feet : " + String(tofSensorThreshold), "TOF : " + String(advertiseInterval));
            delay(2000);
            DisplayHomepage(displayLine1, displayLine2);
            handleTofModes(TofModes);
          }
          else if (mqttData.startsWith("WIFIRSSI"))
          {
            deviceBusyStatus = 0;
            DisplayHomepage(displayLine1, displayLine2);
            String wifiNetworksJSON = getWifiNetworksJSON(10000);
            Serial.println("Wi-Fi Networks in JSON format:");
            Serial.println(wifiNetworksJSON);
            publishRawToTopicEG25(deviceNetworkTopic, wifiNetworksJSON);
            deviceBusyStatus = 1;
            DisplayHomepage(displayLine1, displayLine2);
          }
          else if (mqttData.startsWith("GAIN#"))
          {
            gainValue = mqttData.substring(5).toInt();
            if (gainValue >= 0 && gainValue <= 100)
            {
              Serial.println("Setting gain to: " + String(gainValue));
              mixer1.gain(0, gainValue);
              mixer2.gain(0, gainValue);
            }
            else
            {
              Serial.println("Invalid gain value in GAIN payload");
            }
          }
        }
        else
        {
          Serial.println("Invalid mqtt topic and data");
        }
        gsmMqttConnected = true;
      }
      else if (receivedString.startsWith("+QIURC: \"pdpdeact\",1"))
      {
        Serial.println("PDP context has been deactivated");
        goGsmMqttOffline();
      }
      else if (receivedString.startsWith("+QMTPING:"))
      {
        Serial.print("__Data(PING)__-->"); Serial.println(receivedString);
        Serial.println("MQTT PING error detected");
        goGsmMqttOffline();
      }
      else if (receivedString.startsWith("+QMTSTAT"))
      {
        Serial.print("__Data(STAT)__-->"); Serial.println(receivedString);
        Serial.println("MQTT STAT error detected");
        goGsmMqttOffline();
      }
    }

    // ---------- 4G MQTT Reconnection with 10-min watchdog ----------
    if (!gsmMqttConnected && !enableWifiGsm)
    {
      unsigned long currentMillis = millis();

      // Check 10-minute watchdog → reboot
      if (gsmBusySinceTime > 0 && (currentMillis - gsmBusySinceTime >= gsmMqttDownRebootTimeout))
      {
        Serial.println("*** 4G MQTT down for 10 minutes - REBOOTING ***");
        teensyDoReboot();
      }

      // Try reconnection every 30 seconds
      if (currentMillis - lastMqttReconnectAttempt >= 30000)
      {
        lastMqttReconnectAttempt = currentMillis;
        Serial.println("4G MQTT reconnection attempt...");
        gsmMqttConnectFlowSingleAttempt();

        if (gsmMqttConnected)
        {
          Serial.println("4G MQTT reconnected successfully!");
          gsmBusySinceTime = 0; // Reset watchdog
          deviceBusyStatus = 1;
          displayLine1 = GsmNetworkTypeString;
          displayLine2 = GsmOperatorName;
          DisplayHomepage(displayLine1, displayLine2);
        }
        else
        {
          unsigned long elapsed = (currentMillis - gsmBusySinceTime) / 1000;
          Serial.println("4G MQTT reconnection failed. Offline for " + String(elapsed) + "s / 600s");
        }
      }
    }

    // ---------- Log timers (only if MQTT connected) ----------
    if (gsmMqttConnected && totalDeviceLog && deviceTicketTelematriceMode)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - lastGsmLogPublishTime >= gsmLogPublishInterval)
      {
        lastGsmLogPublishTime = currentMillis;
        publishDeviceLogData();
        processBufferedGsmMqttMessages();
      }
    }
    else if (gsmMqttConnected && deviceTicketTelematricHybridMode == 1)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - lastGsmLogPublishTime >= gsmLogPublishInterval)
      {
        lastGsmLogPublishTime = currentMillis;
        publishDeviceLogData();
        processBufferedGsmMqttMessages();
      }
    }

    // Process any GSM buffered messages each loop cycle
    if (gsmMqttRecvBufferCount > 0)
    {
      processBufferedGsmMqttMessages();
    }

    // ---------- Drain GSM ACK queue (one per loop cycle) ----------
    if (gsmMqttConnected)
    {
      int ackSlot = getNextGsmAckQueueItem();
      if (ackSlot != -1)
      {
        String ackPayload = gsmAckQueue[ackSlot].payload;
        gsmAckQueue[ackSlot].occupied = false;
        publishLogFireAndForgetEG25(deviceAckTopic, ackPayload);
      }
    }
  }

  eg25.flush();
}



void activateAPN()
{
  // String apnCommand = "AT+CGDCONT=1,\"IP\",\"iot.com\""; // Airtel
  String apnCommand = "AT+CGDCONT=1,\"IP\",\"iot.kore.com\""; // A&T
  String pdpCommand = "AT+CGACT=1,1";
  String expectedAPNResponse = "OK";
  String expectedPDPResponse = "OK";
  unsigned long timeout = 5000; // 5 seconds timeout
  int maxRetries = 10;          // Maximum number of retries

  // Activate APN
  for (int retryCount = 0; retryCount < maxRetries; retryCount++)
  {
    String apnResponse = sendATCommand3(apnCommand, expectedAPNResponse, timeout);

    if (apnResponse == expectedAPNResponse)
    {
      // APN activation successful
      Serial.println("APN activated successfully");

      // Attempt to activate PDP context
      String pdpResponse = sendATCommand3(pdpCommand, expectedPDPResponse, timeout);

      if (pdpResponse == expectedPDPResponse)
      {
        // PDP context activation successful
        Serial.println("PDP context activated successfully");
        gsmApnStatus = true;
        DisplayInfo("LTE", "APN connected", codeVersion);
        delay(2000);
        return;
      }
      else
      {
        // Handle PDP activation failure
        Serial.println("Can't activate PDP context");
        // Optionally add a delay before retrying PDP activation
        // delay(1000); // 1-second delay before retry
      }
    }
    else if (apnResponse == "TIMEOUT")
    {
      // Handle APN activation timeout, optionally add a delay between retries
      // delay(1000); // 1-second delay between retries
    }
    else if (apnResponse == "ERROR")
    {
      // Handle APN activation error, you can log an error message or take appropriate action
    }

    // If it's not successful, retry APN activation
  }

  // APN and PDP context activation failed after maxRetries attempts
  Serial.println("Can't activate the APN and PDP context");
  displayCommand(zigNetworkFailedDisplayPage);
  gsmApnStatus = false;
  // Legacy halt removed — caller checks gsmApnStatus
}
bool verifyCheckSum(unsigned char data[], unsigned char len)
{
  TOF_check = 0;

  for (int k = 0; k < len - 1; k++)
  {
    TOF_check += data[k];
  }

  if (TOF_check == data[len - 1])
  {
    // Serial.println("TOF data is ok!");
    return true;
  }
  else
  {
    Serial.println("TOF data is error!");
    return false;
  }
}
void publishNFCData(String nfcName, String nfcTicketID)
{
  String nfcDataPayLoad;
  DynamicJsonDocument nfcLogJson(512);

  // Clear and populate the JSON document
  nfcLogJson.clear();
  nfcLogJson["id"] = teensyMAC();
  nfcLogJson["nfcType"] = nfcType; // Assuming a default type or remove this if not needed
  nfcLogJson["nfcName"] = nfcName;
  nfcLogJson["nfcUserId"] = nfcTicketID;

  // Serialize the JSON document into a string payload
  serializeJson(nfcLogJson, nfcDataPayLoad);

  // WiFi: queue for single-thread serial access. GSM: direct send.
  if (enableWifiGsm)
  {
    addToWifiQueue(deviceBleCardTopic, nfcDataPayLoad, QUEUE_PRIORITY_HIGH);
  }
  else
  {
    delay(100);
    publishNFCDataRawToTopicEG25(deviceBleCardTopic, nfcDataPayLoad);
  }
}

void decodeNfcData(String nfcData)
{
  // Trim whitespace from the input data
  nfcData.trim();
  // Reset global variables
  nfcTicketID = "";
  nfcName = "";

  // Only process non-empty data
  if (nfcData.length() > 0)
  {
    // Check if the data is an NDEF record (commonly starts with "D1")
    if (nfcData.startsWith("D1"))
    {
      // Extract the payload length (next two hex characters after the header)
      int payloadLength = strtol(nfcData.substring(4, 6).c_str(), NULL, 16);

      // Extract the actual NDEF payload
      String payloadHex = nfcData.substring(10, 10 + payloadLength * 2);

      // Convert hex payload to a string
      String decodedText = "";
      for (int i = 0; i < payloadHex.length(); i += 2)
      {
        String hexByte = payloadHex.substring(i, i + 2);
        char decodedChar = (char)strtol(hexByte.c_str(), NULL, 16);
        decodedText += decodedChar;
      }

      // Extract nfcTicketID and nfcName
      int separatorIndex = decodedText.indexOf('#');
      if (separatorIndex != -1)
      {
        nfcTicketID = decodedText.substring(2, separatorIndex); // Extract nfcTicketID after language code
        nfcName = decodedText.substring(separatorIndex + 1);    // Extract nfcName
      }
      nfcType = 1;
    }
    else if (nfcData.indexOf('|') != -1)
    {
      // Handle NFC wallet data or other structured formats
      // Split the string using the "|" delimiter to extract different parts
      int part1End = nfcData.indexOf('|');
      int part2End = nfcData.indexOf('|', part1End + 1);
      int part3End = nfcData.indexOf('|', part2End + 1);

      nfcTicketID = nfcData.substring(part2End + 1, part3End); // This is the nfcTicketID
      nfcName = nfcData.substring(part3End + 1);
      nfcType = 2; // This is the nfcName
    }

    // Print nfcTicketID and nfcName if they were detected
    if (nfcTicketID != "" && nfcName != "")
    {
      Serial.println("Ticket ID: " + nfcTicketID);
      Serial.println("Name: " + nfcName);
      DisplayInfo("Hai " + nfcName, "Please wait....!", codeVersion);
      SOSflash(200);
      publishNFCData(nfcName, nfcTicketID);
    }
    else
    {
      Serial.println("No valid NFC data detected.");
    }
  }
}
void tofSerialPortLisener()
{
  while (true)
  {
    if (enableTofSensor)
    {
      if (Serial4.available() >= 32)
      {
        for (int i = 0; i < 32; i++)
        {
          TOF_data[i] = Serial4.read();
        }

        for (int j = 0; j < 16; j++)
        {
          if ((TOF_data[j] == TOF_header[0] && TOF_data[j + 1] == TOF_header[1] && TOF_data[j + 2] == TOF_header[2]) && (verifyCheckSum(&TOF_data[j], TOF_length)))
          {
            if (((TOF_data[j + 12]) | (TOF_data[j + 13] << 8)) == 0)
            {
              Serial.println("Out of range!");
            }
            else
            {
              delay(50);
              TOF_distance = (TOF_data[j + 8]) | (TOF_data[j + 9] << 8) | (TOF_data[j + 10] << 16);
              float distanceInFeet = TOF_distance * 0.00328084;
              // Serial.print("TOF distance is: ");
              // Serial.println(distanceInFeet, 2);
              if (TofModes == 6)
              {
                String formattedDistance = tofDisplayPrefix + distanceInFeet + " ft";
                displayCommand("t2.txt=\"" + formattedDistance + "\"");
              }

              if (TofModes == 2 || TofModes == 5)
              {
                // Serial.print("The Entries is : ");
                // Serial.println(peopleCount);
              }
              if (distanceInFeet > 0.00)
              {
                if (distanceInFeet < tofSensorThreshold)
                {
                  if (!personInsideThreshold)
                  {
                    if (TofModes == 2)
                    {
                      peopleCount++;
                    }
                    else if (TofModes == 3)
                    {
                      if (peopleHold > 0)
                      {
                        Serial.println("Valid person");
                        peopleHold--;
                      }
                      else
                      {
                        Serial.println("illegal entry");
                        peopleCount++;
                        ProcessDisplayData("301");
                      }
                    }
                    else if (TofModes == 4 && !benchmarkTimerStarted)
                    {
                      benchmarkEntryStartTime = millis(); // Start the timer
                      benchmarkTimerStarted = true;
                    }
                    else if (TofModes == 5)
                    {
                      peopleCount++;
                    }
                    personInsideThreshold = true;
                  }
                  if (TofModes == 1 || TofModes == 2)
                  {
                    // Serial.println("BLE Advertise begin");
                    bleAdvertisebegin(); // Begin Ble
                  }
                  delay(100);
                  lastAdvertiseTime = millis();
                  bleAdvertiseCalled = true;
                }
                else if (distanceInFeet >= tofSensorThreshold && personInsideThreshold)
                {
                  // If the person moves out of the threshold, reset the flag
                  if (TofModes == 4)
                  {
                    benchmarkTimerStarted = false;
                  }
                  personInsideThreshold = false;
                }
              }
              else
              {
                personInsideThreshold = false;
              }
            }
            break;
          }
        }
      }

      if (bleAdvertiseCalled && millis() - lastAdvertiseTime >= advertiseInterval)
      {
        if (TofModes == 1 || TofModes == 2)
        {
          Serial.println("_____________________Ble STOP_____________________");
          nrf52.println("BLEADVSTOP"); // Stop Ble
          bleAdStatus = false;
        }
        delay(100);
        // DisplayHomepage(displayLine1, displayLine2);
        bleAdvertiseCalled = false;
        bleStartOnlyOneTime = true;
      }
    }
  }
}
void gpsSerialPortLisener()
{
  while (true)
  {
    while (gpsSerial.available() > 0)
      if (gps.encode(gpsSerial.read()))
        getGpsInfo();
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println("No GPS detected");
      while (true)
        ;
    }
  }
}

void nrf52SerialPortListener()
{
  int bleCardJsonPayloadSize;
  while (true)
  {
    if (nrf52.available())
    {
      // Read the incoming data until a newline character is encountered
      String incomingData = nrf52.readStringUntil('\n');
      Serial.println(incomingData);
      // Check if any data was read
      if (incomingData.length() > 0)
      {
        // Search for the pattern "Last 6 digits of UUID: " in the received data
        int startPos = incomingData.indexOf("Last 6 digits of UUID: ");
        // if (bleScanMode)
        // {
        //   if (startPos != -1)
        //   {
        //     DynamicJsonDocument bleCardJson(256);
        //     // Extract the user ID part after the pattern
        //     String userId = incomingData.substring(startPos + 23);

        //     // Check if the new user ID is different from the last one
        //     if (userId != lastUserId)
        //     {
        //       // Update the last user ID
        //       lastUserId = userId;
        //       userId.trim();
        //       DisplayInfo("Please wait", "Card detected", codeVersion);

        //       // Clear the JSON document
        //       bleCardJson.clear();

        //       // Populate the JSON document with the "userId" field
        //       bleCardJson["deviceId"] = teensyMAC();
        //       bleCardJson["deviceTopic"] = deviceTopic;
        //       bleCardJson["userId"] = userId;

        //       // Serialize the JSON document to a string
        //       String jsonString;
        //       serializeJson(bleCardJson, jsonString);

        //       // Print the JSON string
        //       Serial.println(jsonString);
        //       bleCardJsonPayloadSize = jsonString.length();
        //       if (enableWifiGsm)
        //       {
        //         delay(100);
        //         publishRawToTopic(deviceReactTopic, jsonString, bleCardJsonPayloadSize);
        //       }
        //       else
        //       {
        //         delay(100);
        //         publishRawToTopicEG25(deviceReactTopic, jsonString);
        //       }
        //     }
        //   }
        // }
        int receivedDataPos = incomingData.indexOf("Received data: ");
        if (receivedDataPos != -1)
        {
          // Extract the string after "Received data: "
          String receivedDataString = incomingData.substring(receivedDataPos + 15);
          Serial.println("Received data: " + receivedDataString);
          ProcessDisplayData(receivedDataString);
        }
      }
    }
  }
}
void disconnectPreviousMqtt()
{
  String response = sendATCommand3("AT+QMTDISC=3", "+QMTDISC: 3,0", 5000); // Adjust the timeout as needed

  if (response == "ERROR")
  {
    // Handle the "ERROR" response here, e.g., print an error message or take appropriate action.
    Serial.println("AT+QMTDISC=3 returned ERROR");
  }
  else if (response == "+QMTDISC: 3,0")
  {
    // Handle the "+QMTDISC: 3,0" response here, e.g., print a success message or take appropriate action.
    Serial.println("AT+QMTDISC=3 returned +QMTDISC: 3,0");
  }
}

void beginBleScan()
{
  threads.addThread(gsmEsp32bleScanThread);
}




void getApiResponse(const String &apiUrl)
{
  const int maxRetries = 3;

  for (int attempt = 1; attempt <= maxRetries; attempt++)
  {
    Serial.print("API call attempt ");
    Serial.print(attempt);
    Serial.print(" of ");
    Serial.println(maxRetries);

    // Flush any stale data from ESP32 serial buffer before sending command
    while (espSerial.available())
    {
      espSerial.read();
    }
    delay(100);

    // Send the AT command to make the HTTP GET request
    espSerial.print("AT+HTTPCGET=\"");
    espSerial.print(apiUrl);
    espSerial.println("\"");

    unsigned long startTime = millis();
    unsigned long timeout = 20000;

    // Read the response from espSerial
    String response = "";

    while (millis() - startTime < timeout)
    {
      wdt.feed();
      if (espSerial.available())
      {
        char c = espSerial.read();
        response += c;
      }
    }
    response.replace("\r", "");
    response.replace("\n", "");
    response.trim();

    // Extract and print the JSON data, removing +HTTPCGET:<size> and numbers/commas next to it
    int jsonStartIndex = response.indexOf("{");
    int jsonEndIndex = response.lastIndexOf("}");

    if (jsonStartIndex != -1 && jsonEndIndex != -1)
    {
      String jsonData = response.substring(jsonStartIndex, jsonEndIndex + 1);
      int httpcgetIndex = jsonData.indexOf("+HTTPCGET:");
      while (httpcgetIndex != -1)
      {
        int commaIndex = jsonData.indexOf(',', httpcgetIndex);
        if (commaIndex != -1)
        {
          jsonData.remove(httpcgetIndex, commaIndex - httpcgetIndex + 1);
        }
        httpcgetIndex = jsonData.indexOf("+HTTPCGET:");
      }
      Serial.print("WIFI API Response:  ");
      Serial.println(jsonData);
      parseApiResponse(jsonData);
      return; // Success - exit the retry loop
    }
    else
    {
      Serial.println("JSON data not found in the response (attempt " + String(attempt) + ")");
      Serial.print("Raw response length: ");
      Serial.println(response.length());
      if (response.length() > 0) {
        Serial.print("Raw response: ");
        Serial.println(response.substring(0, min((int)response.length(), 200)));
      }

      if (attempt < maxRetries)
      {
        Serial.println("Retrying API call in 3 seconds...");
        delay(3000);
      }
    }
  }

  // All retries exhausted
  Serial.println("API call failed after all retries.");
  DisplayInfo("ZIG", "Failed", codeVersion);
}


// QR Scanner Data Listener Thread
void qrScannerListener()
{
  while (true)
  {
    wdt.feed(); // Feed watchdog in thread
    myusb.Task();

    // Check hdc1 for new input
    if (hdc1.hasNewInput())
    {
      String qrCode = hdc1.getLastInput();
      hdc1.clearNewInputFlag();

      qrCode.trim();
      for (int j = qrCode.length() - 1; j >= 0; j--)
      {
        if (qrCode[j] < 32 || qrCode[j] > 126)
          qrCode.remove(j, 1);
      }

      if (qrCode.length() > 0 && qrCode.length() <= 15 && qrCode != lastQRCode)
      {
        if (qrCode.length() > 15)
          qrCode = qrCode.substring(0, 15);

        Serial.println("Scanned QR Code: " + qrCode);
        lastQRCode = qrCode;

        // Show "Please wait" screen
        DisplayInfo("QR Code Scanned", "Please wait...", qrCode);
        delay(100);

        DynamicJsonDocument qrJson(512);
        nfcType = 6;
        qrJson["id"] = teensyMAC();
        qrJson["nfcType"] = nfcType;
        qrJson["nfcUserId"] = qrCode;

        String payload;
        serializeJson(qrJson, payload);

        deviceBusyStatus = 2;

        if (enableWifiGsm)
          addToWifiQueue(deviceBleCardTopic, payload, QUEUE_PRIORITY_HIGH);
        else
          publishNFCDataRawToTopicEG25(deviceBleCardTopic, payload);

        // Wait for MQTT response with timeout (10 seconds)
        unsigned long waitStartTime = millis();
        unsigned long responseTimeout = 10000; // 10 second timeout
        bool responseReceived = false;

        while ((millis() - waitStartTime) < responseTimeout)
        {
          // Check if ProcessDisplayData was called (deviceBusyStatus will be set to 1 inside it)
          if (deviceBusyStatus == 1)
          {
            responseReceived = true;
            Serial.println("QR validation response received");
            break;
          }
          wdt.feed(); // Feed watchdog while waiting
          delay(50);  // Small delay to prevent tight loop
        }

        if (!responseReceived)
        {
          Serial.println("QR validation response timeout - no response within 10 seconds");
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
      }
    }

    // Check hdc2
    if (hdc2.hasNewInput())
    {
      String qrCode = hdc2.getLastInput();
      hdc2.clearNewInputFlag();
      qrCode.trim();
      for (int j = qrCode.length() - 1; j >= 0; j--)
        if (qrCode[j] < 32 || qrCode[j] > 126) qrCode.remove(j, 1);

      if (qrCode.length() > 0 && qrCode != lastQRCode)
      {
        if (qrCode.length() > 15) qrCode = qrCode.substring(0, 15);
        Serial.println("Scanned QR Code: " + qrCode);
        lastQRCode = qrCode;

        DisplayInfo("QR Code Scanned", "Please wait...", qrCode);
        delay(100);

        DynamicJsonDocument qrJson(512);
        nfcType = 6;
        qrJson["id"] = teensyMAC();
        qrJson["nfcType"] = nfcType;
        qrJson["nfcUserId"] = qrCode;
        String payload;
        serializeJson(qrJson, payload);

        deviceBusyStatus = 2;
        if (enableWifiGsm)
          addToWifiQueue(deviceBleCardTopic, payload, QUEUE_PRIORITY_HIGH);
        else
          publishNFCDataRawToTopicEG25(deviceBleCardTopic, payload);

        // Wait for MQTT response with timeout
        unsigned long waitStartTime = millis();
        unsigned long responseTimeout = 10000;
        bool responseReceived = false;

        while ((millis() - waitStartTime) < responseTimeout)
        {
          if (deviceBusyStatus == 1)
          {
            responseReceived = true;
            Serial.println("QR validation response received");
            break;
          }
          wdt.feed();
          delay(50);
        }

        if (!responseReceived)
        {
          Serial.println("QR validation response timeout - no response within 10 seconds");
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
      }
    }

    // Check hdc3
    if (hdc3.hasNewInput())
    {
      String qrCode = hdc3.getLastInput();
      hdc3.clearNewInputFlag();
      qrCode.trim();
      for (int j = qrCode.length() - 1; j >= 0; j--)
        if (qrCode[j] < 32 || qrCode[j] > 126) qrCode.remove(j, 1);

      if (qrCode.length() > 0 && qrCode != lastQRCode)
      {
        if (qrCode.length() > 15) qrCode = qrCode.substring(0, 15);
        Serial.println("Scanned QR Code: " + qrCode);
        lastQRCode = qrCode;

        DisplayInfo("QR Code Scanned", "Please wait...", qrCode);
        delay(100);

        DynamicJsonDocument qrJson(512);
        nfcType = 6;
        qrJson["id"] = teensyMAC();
        qrJson["nfcType"] = nfcType;
        qrJson["nfcUserId"] = qrCode;
        String payload;
        serializeJson(qrJson, payload);

        deviceBusyStatus = 2;
        if (enableWifiGsm)
          addToWifiQueue(deviceBleCardTopic, payload, QUEUE_PRIORITY_HIGH);
        else
          publishNFCDataRawToTopicEG25(deviceBleCardTopic, payload);

        // Wait for MQTT response with timeout
        unsigned long waitStartTime = millis();
        unsigned long responseTimeout = 10000;
        bool responseReceived = false;

        while ((millis() - waitStartTime) < responseTimeout)
        {
          if (deviceBusyStatus == 1)
          {
            responseReceived = true;
            Serial.println("QR validation response received");
            break;
          }
          wdt.feed();
          delay(50);
        }

        if (!responseReceived)
        {
          Serial.println("QR validation response timeout - no response within 10 seconds");
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
      }
    }

    // Check hdc4
    if (hdc4.hasNewInput())
    {
      String qrCode = hdc4.getLastInput();
      hdc4.clearNewInputFlag();
      qrCode.trim();
      for (int j = qrCode.length() - 1; j >= 0; j--)
        if (qrCode[j] < 32 || qrCode[j] > 126) qrCode.remove(j, 1);

      if (qrCode.length() > 0 && qrCode != lastQRCode)
      {
        if (qrCode.length() > 15) qrCode = qrCode.substring(0, 15);
        Serial.println("Scanned QR Code: " + qrCode);
        lastQRCode = qrCode;

        DisplayInfo("QR Code Scanned", "Please wait...", qrCode);
        delay(100);

        DynamicJsonDocument qrJson(512);
        nfcType = 6;
        qrJson["id"] = teensyMAC();
        qrJson["nfcType"] = nfcType;
        qrJson["nfcUserId"] = qrCode;
        String payload;
        serializeJson(qrJson, payload);

        deviceBusyStatus = 2;
        if (enableWifiGsm)
          addToWifiQueue(deviceBleCardTopic, payload, QUEUE_PRIORITY_HIGH);
        else
          publishNFCDataRawToTopicEG25(deviceBleCardTopic, payload);

        // Wait for MQTT response with timeout
        unsigned long waitStartTime = millis();
        unsigned long responseTimeout = 10000;
        bool responseReceived = false;

        while ((millis() - waitStartTime) < responseTimeout)
        {
          if (deviceBusyStatus == 1)
          {
            responseReceived = true;
            Serial.println("QR validation response received");
            break;
          }
          wdt.feed();
          delay(50);
        }

        if (!responseReceived)
        {
          Serial.println("QR validation response timeout - no response within 10 seconds");
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
      }
    }

    // Check hdc5
    if (hdc5.hasNewInput())
    {
      String qrCode = hdc5.getLastInput();
      hdc5.clearNewInputFlag();
      qrCode.trim();
      for (int j = qrCode.length() - 1; j >= 0; j--)
        if (qrCode[j] < 32 || qrCode[j] > 126) qrCode.remove(j, 1);

      if (qrCode.length() > 0 && qrCode != lastQRCode)
      {
        if (qrCode.length() > 15) qrCode = qrCode.substring(0, 15);
        Serial.println("Scanned QR Code: " + qrCode);
        lastQRCode = qrCode;

        DisplayInfo("QR Code Scanned", "Please wait...", qrCode);
        delay(100);

        DynamicJsonDocument qrJson(512);
        nfcType = 6;
        qrJson["id"] = teensyMAC();
        qrJson["nfcType"] = nfcType;
        qrJson["nfcUserId"] = qrCode;
        String payload;
        serializeJson(qrJson, payload);

        deviceBusyStatus = 2;
        if (enableWifiGsm)
          addToWifiQueue(deviceBleCardTopic, payload, QUEUE_PRIORITY_HIGH);
        else
          publishNFCDataRawToTopicEG25(deviceBleCardTopic, payload);

        // Wait for MQTT response with timeout
        unsigned long waitStartTime = millis();
        unsigned long responseTimeout = 10000;
        bool responseReceived = false;

        while ((millis() - waitStartTime) < responseTimeout)
        {
          if (deviceBusyStatus == 1)
          {
            responseReceived = true;
            Serial.println("QR validation response received");
            break;
          }
          wdt.feed();
          delay(50);
        }

        if (!responseReceived)
        {
          Serial.println("QR validation response timeout - no response within 10 seconds");
          deviceBusyStatus = 1;
          DisplayHomepage(displayLine1, displayLine2);
        }
      }
    }

    delay(10); // Small delay to prevent CPU overload
  }
}

// Detect Newland QR Scanner
void detectQRScanner()
{
  myusb.Task(); // Task to detect devices
  delay(500);   // Give devices time to enumerate

  qrScannerConnected = false;

  // Check all HID drivers for Newland devices
  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++)
  {
    if (*hiddrivers[i])
    {
      const uint8_t *manufacturer = hiddrivers[i]->manufacturer();
      const uint8_t *product = hiddrivers[i]->product();
      const uint8_t *serialNum = hiddrivers[i]->serialNumber();

      bool isNewland = false;

      // Check manufacturer string
      if (manufacturer && *manufacturer)
      {
        String mfgStr = String((char *)manufacturer);
        Serial.println("Manufacturer: " + mfgStr);
        if (mfgStr.indexOf("Newland") != -1 || mfgStr.indexOf("NEWLAND") != -1)
        {
          isNewland = true;
          qrScannerManufacturer = mfgStr;
        }
      }

      // Check product string
      if (product && *product)
      {
        String prodStr = String((char *)product);
        Serial.println("Product: " + prodStr);
        if (prodStr.indexOf("Newland") != -1 || prodStr.indexOf("NEWLAND") != -1)
        {
          isNewland = true;
          qrScannerProduct = prodStr;
        }
      }

      // Get serial number if available
      if (serialNum && *serialNum)
      {
        qrScannerSerial = String((char *)serialNum);
        Serial.println("Serial: " + qrScannerSerial);
      }

      if (isNewland)
      {
        qrScannerConnected = true;

        // Build device info string
        detectedDeviceInfo = "Manufacturer: " + qrScannerManufacturer +
                             ", Product: " + qrScannerProduct +
                             ", Serial: " + qrScannerSerial;

        Serial.println("*** Newland QR Scanner Connected ***");
        Serial.println(detectedDeviceInfo);

        DisplayInfo("QR Scanner detected!", qrScannerProduct, codeVersion);
        delay(2000);
    if(startDeviceMode == 2 || startDeviceMode == 3)
    {
        // Start QR scanner listener thread
        threads.addThread(qrScannerListener);

        Serial.println("QR Scanner thread started");
        return;
      }
       else{
    Serial.println("qr scanner will not read");
      }

    }
    }
  }
  if (!qrScannerConnected)
  {
    Serial.println("No Newland QR Scanner connected.");
    detectedDeviceInfo = "QR Scanner not connected";
  }



}
void threadNFC()
{
    uint32_t nextScanAllowed = 0;
    const uint32_t scanDelay = 800;  // Prevent double-reads
    uint32_t lastCardSeenTime = 0;
    const uint32_t cardRemovalTimeout = 2000;  // Reset after 2 seconds of no card
    uint32_t lastReaderReset = 0;
    const uint32_t readerResetInterval = 3000;  // Reset reader every 3 seconds if idle
    uint32_t lastSuccessfulScan = millis();
    const uint32_t hardResetInterval = 30000;  // Hard reset if no successful scan for 30 seconds
    uint32_t loopIterations = 0;

    while (true)
    {
        wdt.feed(); // Feed watchdog in thread
        loopIterations++;

        // Hard reset if reader appears stuck (no successful scans for 30 seconds)
        if (millis() - lastSuccessfulScan > hardResetInterval) {
           // Serial.println("NFC: No activity for 30s - performing hard reset");
            Wire.end();
            delay(100);
            Wire.begin();
            delay(100);
            nfc.begin();
            delay(100);
            nfc.SAMConfig();
            lastSuccessfulScan = millis();
            lastReaderReset = millis();
         //   Serial.println("NFC: Hard reset complete");
        }

        // Debug heartbeat every 1000 iterations
        if (loopIterations % 1000 == 0) {
           // Serial.println("NFC thread alive - iteration: " + String(loopIterations));
        }

        // Cooldown
        if (millis() < nextScanAllowed) {
            delay(5);
            continue;
        }

        uint8_t uid[7];
        uint8_t uidLength;

        // Non-blocking scan with timeout protection
        unsigned long scanStartTime = millis();
        bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);  // 100ms timeout
        unsigned long scanDuration = millis() - scanStartTime;

        // If scan took too long, something is wrong
        if (scanDuration > 500) {
            Serial.println("NFC: Scan timeout detected (" + String(scanDuration) + "ms) - resetting reader");
            nfc.SAMConfig();
            lastReaderReset = millis();
            delay(50);
            continue;
        }

        if (!success) {
            // Reset tracking variables if no card detected for timeout period
            if (lastCardSeenTime > 0 && (millis() - lastCardSeenTime) > cardRemovalTimeout) {
                lastNFC = "";
                lastTicketTID = "";
                lastCardSeenTime = 0;
            }

            // Periodically reset reader to prevent stuck state
            if (millis() - lastReaderReset > readerResetInterval) {
                nfc.SAMConfig();
                lastReaderReset = millis();
            }

            delay(10);
            continue;
        }

        // Card detected - update last seen time and successful scan tracker
        lastCardSeenTime = millis();
        lastReaderReset = millis();
        lastSuccessfulScan = millis();  // Track successful activity
        nextScanAllowed = millis() + scanDelay;

        // Check if it's a Mifare Classic 1K card (UID length = 4)
        if (uidLength == 4) {
            // Mifare Classic 1K detected
            Serial.println("==============================");
            Serial.println("MIFARE CLASSIC CARD DETECTED");

            // Build Serial Number (UID) as string
            String serialID = "";
            for (uint8_t i = 0; i < uidLength; i++) {
                if (uid[i] < 0x10) serialID += "0";
                serialID += String(uid[i], HEX);
                if (i < uidLength - 1) serialID += ":";
            }
            serialID.toUpperCase();

            Serial.print("Serial Number: ");
            Serial.println(serialID);

            // Check for duplicate
            if (serialID == lastTicketTID) {
                Serial.println("Duplicate card - ignoring");
                Serial.println("==============================\n");
                continue;
            }

            // Try to read Block 1 (optional - for additional info)
            uint8_t data[16];
            uint8_t defaultKey[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

            unsigned long authStartTime = millis();
            bool authSuccess = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 1, 0, defaultKey);
            unsigned long authDuration = millis() - authStartTime;

            if (authDuration > 300) {
                Serial.println("Mifare: Authentication timeout - skipping block read");
            } else if (authSuccess) {
                if (nfc.mifareclassic_ReadDataBlock(1, data)) {
                    Serial.print("Block 1 Data: ");
                    for (int i = 0; i < 16; i++) {
                        if (data[i] < 0x10) Serial.print("0");
                        Serial.print(data[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                } else {
                    Serial.println("Block 1: Read failed");
                }
            } else {
                Serial.println("Block 1: Authentication failed");
            }

            // Always reset reader after Mifare authentication to prevent stuck state
            nfc.SAMConfig();
            lastReaderReset = millis();

            Serial.println("==============================\n");

            // Set static username and userID from serial
            String username = "NFC-MF";
            String userID = serialID;

            // Store new ticket
            lastTicketTID = userID;
            lastNFC = serialID;  // Update lastNFC to prevent duplicates

            DisplayInfo("Card Detected", "please wait", username);
            Serial.println("UserID: " + userID);
            Serial.println("Username: " + username);
            nfcType = 5;
            publishNFCData(username, userID);

            continue;
        }

        // TAG2 card handling (UID length typically 7)
        uint8_t data[4];
        String raw = "";

        // FAST read for NDEF area with timeout protection
        unsigned long readStartTime = millis();
        int failedReads = 0;
        int successfulReads = 0;

        for (int p = 4; p <= 39; p++) {  // Increased from 15 to 39 to read more pages
            if (nfc.ntag2xx_ReadPage(p, data)) {
                for (int i = 0; i < 4; i++) {
                    char c = data[i];
                    if (c >= 32 && c <= 126)
                        raw += c;
                }
                failedReads = 0;  // Reset fail counter on success
                successfulReads++;
            } else {
                failedReads++;
                // If too many consecutive failures and we have some data, stop gracefully
                if (failedReads > 5 && successfulReads > 0) {
                    Serial.println("TAG2: Multiple read failures but got " + String(successfulReads) + " pages - proceeding");
                    break;
                }
                // If failures at start with no data, abort completely
                if (failedReads > 3 && successfulReads == 0) {
                    Serial.println("TAG2: Read failures at start - aborting");
                    nfc.SAMConfig();
                    lastReaderReset = millis();
                    break;
                }
            }
            delay(2);

            // Timeout protection - don't spend more than 500ms reading (increased from 200ms)
            if (millis() - readStartTime > 500) {
                Serial.println("TAG2: Read timeout after " + String(successfulReads) + " pages - proceeding with available data");
                break;
            }
        }

        Serial.println("TAG2: Read complete. Total raw data length: " + String(raw.length()) + " chars");

        // Parse zignfc first before checking duplicates
        int idx = raw.indexOf("zignfc");
        if (idx == -1) {
            // Invalid/empty TAG2 card - ignore and don't update lastNFC
            Serial.println("TAG2 card detected but no valid 'zignfc' data found - ignoring");
            Serial.println("Raw data: [" + raw + "]");
            continue;
        }

        // DEBUG: Show parsed data
        Serial.println("==============================");
        Serial.println("NFC TAG2 CARD PARSING DEBUG");
        Serial.println("Found 'zignfc' at index: " + String(idx));
        String after = raw.substring(idx + 6);
        Serial.println("After 'zignfc': [" + after + "]");

        int h1 = after.indexOf('#');
        int h2 = after.indexOf('#', h1 + 1);

        Serial.println("Delimiter positions: h1=" + String(h1) + ", h2=" + String(h2));

        if (h1 == -1 || h2 == -1) {
            // Invalid format - don't block future reads
            Serial.println("TAG2 card has 'zignfc' but invalid format - ignoring");
            Serial.println("Expected format: zignfc#<userID>#<username>");
            Serial.println("Raw data: [" + raw + "]");
            Serial.println("==============================\n");
            lastNFC = "";  // Reset to allow next card
            continue;
        }

        String userID = after.substring(h1 + 1, h2);
        String username = after.substring(h2 + 1);

        Serial.println("Before trim - userID: [" + userID + "], username: [" + username + "]");

        userID.trim();
        username.trim();

        Serial.println("After trim - userID: [" + userID + "], username: [" + username + "]");

        // Validate fields are not empty
        if (userID.length() == 0 || username.length() == 0) {
            Serial.println("TAG2 card has empty userID or username - ignoring");
            Serial.println("userID length: " + String(userID.length()) + ", username length: " + String(username.length()));
            Serial.println("==============================\n");
            lastNFC = "";  // Reset to allow next card
            continue;
        }

        Serial.println("==============================\n");

        // If raw is same as last raw → ignore (duplicate tap)
        if (raw == lastNFC)
            continue;

        lastNFC = raw;

        if (userID == lastTicketTID) {
            continue;
        }

        // Store new ticket
        lastTicketTID = userID;
DisplayInfo("Card Detected","please wait",username);
        Serial.println("UserID: " + userID);
        Serial.println("Username: " + username);
        nfcType = 5;
        publishNFCData(username, userID);

      //  delay(20);
    }
 }
void detectPN532()
{
  Serial.println("Initializing PN532 NFC Reader...");

  // Initialize I2C
  Wire.begin();
  delay(100);

  // Initialize PN532
  nfc.begin();
  delay(100);

  // Get firmware version
  uint32_t versiondata = nfc.getFirmwareVersion();

  if (!versiondata)
  {
    Serial.println("ERROR: Didn't find PN53x board");
    Serial.println("Check your wiring!");
    pn532Connected = false;
    pn532ChipInfo = "PN532 not found";
    pn532FirmwareVersion = "N/A";
    return;
  }

  // PN532 found - extract chip and firmware info
  pn532Connected = true;

  uint8_t chipVersion = (versiondata >> 24) & 0xFF;
  uint8_t firmwareMajor = (versiondata >> 16) & 0xFF;
  uint8_t firmwareMinor = (versiondata >> 8) & 0xFF;

  // Build chip info string
  pn532ChipInfo = "Found chip PN5" + String(chipVersion, HEX);
  pn532FirmwareVersion = "Firmware ver. " + String(firmwareMajor) + "." + String(firmwareMinor);

  Serial.print("Found chip PN5");
  Serial.println(chipVersion, HEX);
  Serial.print("Firmware ver. ");
  Serial.print(firmwareMajor, DEC);
  Serial.print('.');
  Serial.println(firmwareMinor, DEC);

  // Add to detected device info
  if (detectedDeviceInfo.length() > 0 && detectedDeviceInfo != "QR Scanner not connected")
  {
    detectedDeviceInfo += " | " + pn532ChipInfo + " (" + pn532FirmwareVersion + ")";
  }
  else
  {
    detectedDeviceInfo = pn532ChipInfo + " (" + pn532FirmwareVersion + ")";
  }

  Serial.println("*** PN532 NFC Reader Connected ***");
  Serial.println(detectedDeviceInfo);
  if(startDeviceMode == 1  ||startDeviceMode == 3)
  {
   threads.addThread(threadNFC);
  }
  else{
    Serial.println("Nfc will not work");
  }
}

String getEG25ResetResponse()
{
  String command = "AT+CFUN=1,1";  
  String expectedResponse = "OK";  
  unsigned long timeout = 20000;    

  // Step 1: Send reset command
  String response = sendATCommand3(command, expectedResponse, timeout);
  response.trim();

  // Step 2: Check for OK
  if (response == expectedResponse)
  {
    Serial.println("Reset command accepted. Waiting for RDY...");

    // Step 3: Wait for "RDY" after reboot
    unsigned long startTime = millis();
    String bootMsg = "";
    while (millis() - startTime < 20000)  // Wait up to 10 seconds for RDY
    {
       wdt.feed();
      while (Serial1.available()) {
        char c = Serial1.read();
        bootMsg += c;

        if (bootMsg.indexOf("RDY") != -1) {
          Serial.println("Modem RDY received.");
          return "Device reseted successfully";
        }
      }
      delay(1);
    }

    Serial.println("Timeout waiting for RDY.");
    return "Timeout";
  }
  else
  {
    Serial.println("Reset command failed.");
    return "Unknown";
  }
}

  void loadbrightessvalue() {
  File file = SD.open(displaybrightessFile, FILE_READ);
  if (file) {
    String brightnessStr = "";
    while (file.available()) {
      brightnessStr += (char)file.read();
    }
    file.close();

    // Convert the string to an integer
    int displayBrightnessValue = brightnessStr.toInt();
    Serial.println(displayBrightnessValue);

    // Call the brightness control function
    displaybrightesscontrol(displayBrightnessValue);
  } else {
    Serial.println("Failed to open file for reading.");
  }
}

void writeWavHeader(File &file, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels, uint32_t dataSize) {
  file.seek(0);
  file.write("RIFF", 4);
  uint32_t chunkSize = 36 + dataSize;
  file.write((uint8_t*)&chunkSize, 4);
  file.write("WAVE", 4);

  file.write("fmt ", 4);
  uint32_t subchunk1Size = 16;
  file.write((uint8_t*)&subchunk1Size, 4);
  uint16_t audioFormat = 1;
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&channels, 2);
  file.write((uint8_t*)&sampleRate, 4);

  uint32_t byteRate = sampleRate * channels * bitsPerSample / 8;
  file.write((uint8_t*)&byteRate, 4);
  uint16_t blockAlign = channels * bitsPerSample / 8;
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);

  file.write("data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

void finalizeWav(File &file, uint32_t dataSize) {
  file.seek(4);
  uint32_t chunkSize = 36 + dataSize;
  file.write((uint8_t*)&chunkSize, 4);

  file.seek(40);
  file.write((uint8_t*)&dataSize, 4);
}

void setAudioUpdateError(const String &msg) {
    if (isvalid) 
    {
      audioUpdateErrorValid = msg;
      isvalid = false;
    }
    if (isinvalid) 
    {
      audioUpdateErrorInvalid = msg;
      isinvalid = false;
    }
}

void deletewavfile()
{

if(SD.exists(INPUT_FILE) || SD.exists(OUTPUT_FILE))
{
  if(SD.remove(INPUT_FILE) || SD.remove(OUTPUT_FILE))
  {
    Serial.println("Corrupted wav file removed");

  }
  else{
    Serial.println("Corupted file is available");

  }
}
else{
  Serial.println("File.wav does not exist");
}
}


void appendAudioversion(String filename, float versionToStore) {
  File wavFile = SD.open(filename.c_str(), FILE_WRITE);  
  if (wavFile) {
    wavFile.seek(wavFile.size());
    wavFile.write((uint8_t*)&versionToStore, sizeof(float)); 
    wavFile.close();

    Serial.print("Appended version marker (float): ");
    Serial.println(versionToStore, 1);
  
  } else {
    Serial.print("Failed to open file for appending: ");
    Serial.println(filename);
    setAudioUpdateError("Fail to write version");
  }
  
}



void processwafile() {
bool fileremoved;


  if(fileSize  == totalbyteswritten)
{
  File inputfile = SD.open(INPUT_FILE, FILE_READ);
  if (!inputfile) {
    Serial.println(F("Failed to open input.txt!"));
    setAudioUpdateError("open failed for read");
    return;
  }
  

  File wavFile = SD.open(OUTPUT_FILE, FILE_WRITE);
  if (!wavFile) {
    Serial.println(F("Failed to open output.wav!"));
    inputfile.close();
    setAudioUpdateError("open failed.wav");
    return;
  }

  Serial.println(F("Writing WAV header…"));
  writeWavHeader(wavFile, 44100, 16, 1, 0); // Sample rate, bits, channels, dummy size

  const size_t BUFFER_SIZE = 512;
  uint8_t buffer[BUFFER_SIZE];
  size_t bufIndex = 0;
  dataSize = 0;

  char highNibble = 0;
  char lowNibble = 0;
  bool halfByte = false;

  while (inputfile.available()) {
    char c = inputfile.read();

    if (!isxdigit(c)) continue; // skip if not a valid hex digit

    if (!halfByte) {
      highNibble = c;
      halfByte = true;
    } else {
      lowNibble = c;
      char hexStr[3] = {highNibble, lowNibble, '\0'};
      uint8_t value = (uint8_t)strtoul(hexStr, NULL, 16);
      buffer[bufIndex++] = value;
      dataSize++;
      halfByte = false;

      if (bufIndex == BUFFER_SIZE) {
        wavFile.write(buffer, BUFFER_SIZE);
        bufIndex = 0;
      }
    }
  }

  if (bufIndex > 0) {
    wavFile.write(buffer, bufIndex);
  }

  Serial.print(F("Total audio data bytes: "));
  Serial.println(dataSize);
 
  if (Sizedataforbytes == dataSize ) {
    finalizeWav(wavFile, dataSize);
    Serial.println(F("WAV file written and finalized."));

    inputfile.close();
    wavFile.close();

    if (SD.exists(filename.c_str())) {
      SD.remove(filename.c_str());
      if(!SD.exists(filename.c_str()))
      {
         fileremoved = true;
      }
     else{
      Serial.println("old file is not removed");
      fileremoved = false;
      setAudioUpdateError("old file not removed");
       
     }       
    }
    else
    {
      Serial.println("file is not found");
      fileremoved = true;
    }
  
if(fileremoved)
{
  
    SD.rename(OUTPUT_FILE, filename.c_str());
    Serial.println(F("Saved final WAV as: ") + filename);
    SD.remove(INPUT_FILE);
    DisplayInfo("Audio Update Done", "Please wait", codeVersion);
     
   
      if (isvalid) {
        appendAudioversion(validFilename, validaudioupdate);
        setAudioUpdateError("updated");    
        validStoredVersion = validaudioupdate;
    }
    if (isinvalid) {
        appendAudioversion(invalidFilename, invalidaudioupdate);
        setAudioUpdateError("updated");    
        invalidStoredVersion = invalidaudioupdate;
    }
       
}
  } else {
    Serial.println(F("Size mismatch. Deleting corrupt output."));
    inputfile.close();
    wavFile.close();
    SD.remove(INPUT_FILE);
    SD.remove(OUTPUT_FILE);
    setAudioUpdateError("Audio size mismatched");
  }
}
else{
   Serial.println("Bytes Written mismatched");
   SD.remove(INPUT_FILE);
  SD.remove(OUTPUT_FILE);
  setAudioUpdateError("Bytes written mismatched");
}

}


void receiveWavChunks()
{
  
DisplayInfo("Audio updating...", "Please wait", codeVersion);
  int txSize = 1024;
int rxSize = 4096;
int timeout4 = 60000;
String get = String("AT+HTTPCGET=\"") + urldata.c_str() + "\"," + String(txSize) + "," + String(rxSize) + "," + String(timeout4);
// Send the command
sendAT(get.c_str());


Serial.println("Receiving HTTPCLIENT data stream...");

 File inputfile = SD.open(INPUT_FILE, FILE_WRITE);
if (!inputfile) {
    Serial.println("Failed to open update.txt for writing!");
    setAudioUpdateError("failed to open txt file");
    return;
}


String header = "";
bool readingHeader = true;
int bytesToRead = 0;
int bytesWritten = 0;
 totalbyteswritten = 0;

unsigned long lastDataTime = millis();
const unsigned long timeout_ms = 60000;

while (millis() - lastDataTime < timeout_ms) {
    if (espSerial.available()) {
        char c = espSerial.read();
        lastDataTime = millis();

        if (readingHeader) {
            // Only append printable characters
            if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
                header += c;

                if (c == ',') {
                    // full header line should now exist
                    int colonIndex = header.indexOf(':');
                    int commaIndex = header.lastIndexOf(',');
                    if (colonIndex != -1 && commaIndex != -1) {
                        String numStr = header.substring(colonIndex + 1, commaIndex);
                        numStr.trim();
                        bytesToRead = numStr.toInt();
                        if (bytesToRead > 0 && bytesToRead < 2000000) {
                            Serial.print("Valid header. Expected data bytes: ");
                            Serial.println(bytesToRead);
                            readingHeader = false;
                            bytesWritten = 0;
                            header = ""; // clear header for next chunk
                        } else {
                            Serial.println("Invalid byte count in header: " + numStr);
                            header = "";
                        }
                    } else {
                        Serial.println("Malformed header: " + header);
                        header = "";
                    }
                }

                if (header.length() > 100) {
                    Serial.println("Header too long. Resetting.");
                    header = "";
                }
            }
        } else {
            // Writing binary data
            inputfile.write((uint8_t)c);
            bytesWritten++;
            totalbyteswritten++;

            if (bytesWritten >= bytesToRead) {
                Serial.println("Chunk written.");
                readingHeader = true;
                header = "";
            }
            
        }
    } else {
        delay(1);
    }
}

inputfile.close();
Serial.println("All chunks saved.");
Serial.print("Bytes written in last chunk: ");
Serial.println(bytesWritten);
Serial.print("Total bytes written: ");
Serial.println(totalbyteswritten);
processwafile();
}



long getHttpFileSize(const char *url) {
  String cmd = String("AT+HTTPGETSIZE=\"") + url + "\"";
  espSerial.println(cmd);
  Serial.print(F("> "));
  Serial.println(cmd);

  unsigned long start = millis();
  String line;
  while (millis() - start < 30000) {
    while (espSerial.available()) {
      char c = espSerial.read();
      Serial.write(c);  // echo to Serial
      if (c == '\n' || c == '\r') {
        if (line.startsWith("+HTTPGETSIZE:")) {
          String numStr = line.substring(13);
          numStr.trim();
          long size = numStr.toInt();
          Serial.print(F("\nFile Size: "));
          Serial.println(size);
          return size;
        }
        line = "";
      } else {
        line += c;
      }
    }
  }
  Serial.println(F("Timeout waiting for HTTPGETSIZE response"));
  return -1;
}





void Httpcget()
{



 fileSize = getHttpFileSize(urldata.c_str());
if (fileSize > 0) {

  Serial.print(F("Server reports file size: "));
  Serial.println(fileSize);
 DisplayInfo("Audio update found", "Please wait", codeVersion);
  receiveWavChunks();

} else {
  Serial.println(F("Failed to get file size!"));
  setAudioUpdateError("AT+HTTPSIZE failed");
}
}


 
void setup()
{
CORE_PIN2_CONFIG  = 0;
CORE_PIN3_CONFIG  = 0;
CORE_PIN4_CONFIG  = 0;
CORE_PIN5_CONFIG  = 0;
CORE_PIN33_CONFIG = 0;
   Serial.begin(115200);
displaySerial.begin(displayBaud);
initializeSDCard();
loadbrightessvalue();
CORE_PIN14_CONFIG = 0;
CORE_PIN15_CONFIG = 0;
Serial3.addMemoryForRead(serial3Buffer_hw, sizeof(serial3Buffer_hw));
  gpsSerial.begin(GPSBaud); // Initialize Serial Monitor
  delay(1000);
CORE_PIN14_CONFIG = 2;
 CORE_PIN15_CONFIG = 2;
Serial2.addMemoryForRead(serial2Buffer_hw, sizeof(serial2Buffer_hw));
  espSerial.begin(allSerialBraudrate); // Initialize Serial2 for ESP32
  nrf52.begin(allSerialBraudrate);
 Serial1.addMemoryForRead(serial1Buffer_hw, sizeof(serial1Buffer_hw));
  eg25.begin(allSerialBraudrate);
  delay(500);
  teensyMAC(mac);
  teensySN(serial);
  delay(3000);
  DisplayInfo(deviceTeensyMacAddress, "Please wait", codeVersion);
  randomSeed(millis());
  Serial.print("V ");
  Serial.println(codeVersion);
  Serial.println(deviceApiUrl);
  Serial.println();

  // Check for crash report
  Serial.println("=== Crash Report Check ===");
  Serial.print(CrashReport);
  Serial.println("=== End Crash Report ===");

  // Initialize Watchdog Timer (8 seconds timeout)
  watchdogConfig.window = 8000;      // 8 second window
  watchdogConfig.timeout = 10000;    // 10 second timeout
  watchdogConfig.trigger = 5;        // Reset after 5 triggers
  wdt.begin(watchdogConfig);
  Serial.println("Watchdog Timer initialized - 8s timeout");
  pinMode(6,OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(U9_RED, OUTPUT);
  pinMode(U9_GREEN, OUTPUT);
  pinMode(U8_RED, OUTPUT);
  pinMode(U8_GREEN, OUTPUT);
  pinMode(U8_BLUE, OUTPUT);
  pinMode(U5_RED, OUTPUT);
  pinMode(U5_GREEN, OUTPUT);
  pinMode(U5_BLUE, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  AudioMemory(40);
  delay(200);
  mixer1.gain(0, 4);
  mixer2.gain(0, 4);
  //lightUpLED(1, 1, 1, 0);
  SOSflash(100);
  delay(500);

   
  unsigned long dhcpTimeout = 10000;    // 10-second total timeout for DHCP
  unsigned long responseTimeout = 2000; // 2-second timeout per response

  if (isEthernetConnected)
  {
    DisplayInfo(deviceTeensyMacAddress, "ETHERNET FOUND", codeVersion);
    
  
  }
  else
  {


     wdt.feed();
    displayCommand(switchingTo4GDisplayPage);
    deviceInfoPageStatus = true;
    delay(3000);
    wdt.feed();
      String resetResponse = getEG25ResetResponse();
     Serial.println("Reset Response: " + resetResponse);
    wdt.feed();
    String deviceModel = getEG25DeviceModel();
    Serial.println("Device Model: " + deviceModel);
    checkSimCardStatus(true); // halt program
    wdt.feed();
    if (gsmSimFound)
    {
      delay(2000);
      wdt.feed();
      activateAPN(); // halt program
      delay(500);
      wdt.feed();
      if (gsmApnStatus)
      {
        delay(100);
        setInternetRegistration();
        delay(1000);
        wdt.feed();
        activateInternet(); // halt program
        delay(100);
       deleteHexFileIfExists();
       deletewavfile();
       eg25moduleconnected = true;
        wdt.feed();
        get4GApiReponse(); // halt program
        wdt.feed();
        if (gsmInternetStatus && wifiApiDataState)
        {
          gsmMqttConnectFlow(false);
          wdt.feed();
          delay(1000);
          // enableEG25GPSModule();
        }
      }
    }
    else if(!gsmSimFound)
    {
      wdt.feed();
    delay(200);
    deleteHexFileIfExists();
    deletewavfile();
    delay(1000);
    bleNrf52Init();
    InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);
   delay(2000);
  resetAndDisconnect();
  delay(2000);
  configureReconnect();
  delay(2000);
  connectToWiFi();
  delay(2000);
  enableAutoConnect();
  delay(2000);
     delay(2000);
     wdt.feed();
    getApiResponse(deviceApiUrl);
     wdt.feed();
delay(4000);
    cleanMQTTSession();
    delay(3000);
    connectToMQTT();
    delay(500);
    }
    delay(2000);
    wdt.feed();
    if (gsmMqttConnected)
    {
      myusb.begin();
      delay(2000);
      detectQRScanner();
      delay(1000);
      detectPN532();
      delay(2000);
      wdt.feed();
      publishDeviceLogData();
      delay(1000);
      checkAndPublishCrashReport(); // Check and send crash report after device log
      delay(2000);
      wdt.feed();
      Serial.println("___________GSM Thread started___________");
      threads.addThread(gsmPortLisener);
      enableWifiGsm = false;
      delay(8000);
      wdt.feed();
      if (enableTofSensor && !TofModes == 0)
      {
        delay(100);
        publishRawToTopicEG25(deviceTofTopic, TofData);
      }
    }
  }

    Serial.println(bleNrf52Command);
    threads.addThread(gpsSerialPortLisener);
    delay(5000);
    if (enableTofSensor)
    {
      tofSerial.begin(allSerialBraudrate);
      Serial.println("___________TOF SENSOR ENABLED___________");
      threads.addThread(tofSerialPortLisener);
    }
    if (bleScanMode)
    {
      Serial.println("___________BLE CARD SCAN ENABLED___________");
      beginBleScan();
    }
      displayAllConnectedPage();
  }



  //   delay(200);
  //   deleteHexFileIfExists();
  //   deletewavfile();
  //   delay(1000);
  //   bleNrf52Init();
  //   InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);
  //  delay(2000);
  // resetAndDisconnect();
  // delay(2000);
  // configureReconnect();
  // delay(2000);
  // connectToWiFi();
  // delay(2000);
  // enableAutoConnect();
  // delay(2000);
  // if (wifiConnected)
  // {
  //   // Wait for ESP32 WiFi stack to stabilize (DNS, DHCP) before API call
  //   Serial.println("Waiting for WiFi stack to stabilize...");
  //   delay(2000);
  //   wdt.feed();
  //   getApiResponse(deviceApiUrl);
  //   wdt.feed();

  //   if (wifiApiDataState)
  //   {
  //     // API succeeded — continue WiFi path
  //     DisplayInfo("ZIG", "Connecting", codeVersion);
  //     delay(4000);
  //     wdt.feed();
  //     cleanMQTTSession();
  //     delay(3000);
  //     wdt.feed();
  //     connectToMQTT();
  //     delay(500);
  //     wdt.feed();
  //     checkSimCardStatus(false);
  //     if (gsmSimFound)
  //     {
  //       check4GAvailability(); // Phase 1: verify 4G MQTT reachable, sets eg25ReadyForFailover
  //       if (!eg25ReadyForFailover)
  //       {
  //         Serial.println("*** Phase 1: 4G not available — device will stay on WiFi if disconnected ***");
  //       }
  //       else
  //       {
  //         Serial.println("*** Phase 1: 4G verified — device will auto-switch to 4G after 2 min WiFi loss ***");
  //       }
  //     }
  //     else
  //     {
  //       Serial.println("*** Phase 1: No SIM detected — WiFi-only mode, no 4G failover ***");
  //     }
  //     if (mqttConnected)
  //     {
  //        myusb.begin();
  //       delay(2000);
  //       detectQRScanner();
  //       delay(1000);
  //       detectPN532();
  //       delay(2000);
  //       wdt.feed();
  //       publishDeviceLogData();
  //       delay(1000);
  //       checkAndPublishCrashReport(); // Check and send crash report after device log
  //       delay(2000);
  //       wdt.feed();
  //      Serial.println("___________Wifi Thread started___________");
  //       threads.addThread(getDataFromSubscribedTopic);
  //       enableWifiGsm = true;
  //       delay(8000);
  //       wdt.feed();
  //       if (enableTofSensor && !TofModes == 0)
  //       {
  //         delay(100);
  //         publishRawToTopic(deviceTofTopic, TofData, TofData.length());
  //       }
  //     }
  //   }
  //   else
  //   {
  //     // WiFi connected but API failed (no internet) — fall through to 4G path
  //     Serial.println("*** WiFi connected but API failed — switching to 4G ***");
  //   }
  // }

  // // 4G fallback: no WiFi, or WiFi had no internet (API failed), or MQTT failed
  // if (!wifiApiDataState || !mqttConnected)
  // {
  //   wdt.feed();
  //   displayCommand(switchingTo4GDisplayPage);
  //   deviceInfoPageStatus = true;
  //   delay(3000);
  //   wdt.feed();
  //     String resetResponse = getEG25ResetResponse();
  //    Serial.println("Reset Response: " + resetResponse);
  //   wdt.feed();
  //   String deviceModel = getEG25DeviceModel();
  //   Serial.println("Device Model: " + deviceModel);
  //   checkSimCardStatus(true); // halt program
  //   wdt.feed();
  //   if (gsmSimFound)
  //   {
  //     delay(2000);
  //     wdt.feed();
  //     activateAPN(); // halt program
  //     delay(500);
  //     wdt.feed();
  //     if (gsmApnStatus)
  //     {
  //       delay(100);
  //       setInternetRegistration();
  //       delay(1000);
  //       wdt.feed();
  //       activateInternet(); // halt program
  //       delay(100);
  //      deleteHexFileIfExists();
  //      deletewavfile();
  //      eg25moduleconnected = true;
  //       wdt.feed();
  //       get4GApiReponse(); // halt program
  //       wdt.feed();
  //       if (gsmInternetStatus && wifiApiDataState)
  //       {
  //         gsmMqttConnectFlow(false);
  //         wdt.feed();
  //         delay(1000);
  //         // enableEG25GPSModule();
  //       }
  //     }
  //   }
  //   delay(2000);
  //   wdt.feed();
  //   if (gsmMqttConnected)
  //   {
  //     myusb.begin();
  //     delay(2000);
  //     detectQRScanner();
  //     delay(1000);
  //     detectPN532();
  //     delay(2000);
  //     wdt.feed();
  //     publishDeviceLogData();
  //     delay(1000);
  //     checkAndPublishCrashReport(); // Check and send crash report after device log
  //     delay(2000);
  //     wdt.feed();
  //     Serial.println("___________GSM Thread started___________");
  //     threads.addThread(gsmPortLisener);
  //     enableWifiGsm = false;
  //     delay(8000);
  //     wdt.feed();
  //     if (enableTofSensor && !TofModes == 0)
  //     {
  //       delay(100);
  //       publishRawToTopicEG25(deviceTofTopic, TofData);
  //     }
  //   }
  // }

  //   Serial.println(bleNrf52Command);
  //   threads.addThread(gpsSerialPortLisener);
  //   delay(5000);
  //   if (enableTofSensor)
  //   {
  //     tofSerial.begin(allSerialBraudrate);
  //     Serial.println("___________TOF SENSOR ENABLED___________");
  //     threads.addThread(tofSerialPortLisener);
  //   }
  //   if (bleScanMode)
  //   {
  //     Serial.println("___________BLE CARD SCAN ENABLED___________");
  //     beginBleScan();
  //   }
  // }

  // displayAllConnectedPage();





 

 


// Helper: revert from a failed 4G switch attempt back to WiFi offline mode.
// On the first failed attempt the retry window expands from 2 min → 10 min.
void revertToWifiOfflineMode(const String &reason)
{
  Serial.println("[4G Switch] FAILED — " + reason + " — reverting to WiFi offline mode");
  enableWifiGsm = true;
  eg25moduleconnected = false;
  eg25ReadyForFailover = false;
  espSerialBusy = false;

  // After first failed attempt, increase retry window to 10 minutes
  if (!firstFailoverAttempted)
  {
    firstFailoverAttempted = true;
    wifiFailoverTimeout = 600000; // 10 min for all subsequent attempts
    Serial.println("[4G Switch] Retry window extended to 10 min for subsequent attempts");
  }
  // Reset the failover timer so we count the full new window from now
  wifiDownSince = millis();

  // Return display to home page — clears the "Switching to 4G" screen
  wifiConnected = false;
  deviceBusyStatus = 0;
  deviceHomePageStatus = true;
  DisplayHomepage(displayLine1, displayLine2);
}

// Phase 2: Switch from WiFi to 4G after WiFi has been down for wifiFailoverTimeout.
// Three gates must ALL pass; any failure reverts to WiFi offline mode.
//   Gate 1 — SIM present and ready (AT+CPIN?)
//   Gate 2 — 4G MQTT connects successfully
//   Gate 3 — Device log published
// Only after all three does the device commit to 4G mode.
void switchToEG25Mode()
{
  Serial.println("___________Switching to 4G Failover Mode___________");

  // Pause WiFi listener for the duration of this function
  espSerialBusy = true;
  delay(300);
  wdt.feed();

  displayCommand(switchingTo4GDisplayPage);
  deviceInfoPageStatus = true;
  delay(3000);
  wdt.feed();

  // ── Gate 1: SIM check ────────────────────────────────────────────────
  Serial.println("[4G Switch] Gate 1: Checking SIM...");
  wdt.feed();
  String simResp = sendATCommand3("AT+CPIN?", "+CPIN:", 5000);
  if (simResp.indexOf("READY") < 0)
  {
    revertToWifiOfflineMode("SIM not ready (" + simResp + ")");
    return;
  }
  Serial.println("[4G Switch] Gate 1 PASSED — SIM ready");

  // Re-verify network registration
  setInternetRegistration();
  delay(1000);

  bool netOk = false;
  for (int i = 0; i < 10; i++)
  {
    wdt.feed();
    String r = sendATCommand3("AT+CREG?", "+CREG:", 5000);
    if (r.startsWith("+CREG: 1,1") || r.startsWith("+CREG: 1,5"))
    {
      netOk = true;
      gsmInternetStatus = true;
      Serial.println("[4G Switch] Network registered OK");
      break;
    }
    delay(1000);
  }

  if (!netOk)
  {
    revertToWifiOfflineMode("Network not registered");
    return;
  }

  // ── Gate 2: 4G MQTT must connect ─────────────────────────────────────
  Serial.println("[4G Switch] Gate 2: Connecting 4G MQTT...");
  wdt.feed();
  enableWifiGsm = false;
  eg25moduleconnected = true;
  gsmMqttConnectFlow(false);
  wdt.feed();

  if (!gsmMqttConnected)
  {
    revertToWifiOfflineMode("4G MQTT connect failed");
    return;
  }
  Serial.println("[4G Switch] Gate 2 PASSED — 4G MQTT connected");

 // ── Gate 3: Send device log ───────────────────────────────────────────
  Serial.println("[4G Switch] Gate 3: Sending device log...");
  wdt.feed();
  publishDeviceLogData();
  delay(1000);
  Serial.println("[4G Switch] Gate 3 PASSED — device log sent");

  // NEW FIX: Stop the ESP32 from frantically searching for WiFi.
  // This saves ~400mA of power and prevents brownouts!
  espSerial.println("AT+CWQAP"); // Tell ESP32 to drop connection
  delay(100);
  espSerial.println("AT+SLEEP=2"); // Put ESP32 radio to sleep
  
  // NEW FIX: Increase delay from 12 to 20 seconds. 
  // LTE networks (like AT&T/Kore) keep the modem in high-power mode for up to 15-20 seconds.
  Serial.println("[4G Switch] Waiting for modem to settle...");
  for (int i = 0; i < 20; i++) 
  {
    wdt.feed();
    delay(1000);
  }

  espSerialBusy = false;
  delay(500);
  wdt.feed();

  displayCommand(zigNetworkFoundDisplayPage);
  if (useSpeakerSound)
  {
    // NEW FIX: Temporarily suspend NFC thread to prevent Pin 2/3 Hardware collision
    threads.suspend(threads.id()); // Suspend active conflicts if possible, or just delay
    
    // Lower the audio gain slightly just for the failover sound to reduce peak current
    float oldGain = gainValue;
    mixer1.gain(0, gainValue * 0.5); 
    mixer2.gain(0, gainValue * 0.5);

    fileIndex = 3;
    playSpeaker = true;
    playWavFile(); // Play synchronously before GSM thread starts

    // Restore volume
    mixer1.gain(0, oldGain);
    mixer2.gain(0, oldGain);
  }
  else
  {
    lightUpLED(2, 2, 2, 1);
    delay(3000);
    wdt.feed();
    lightUpLED(0, 0, 0, 0);
  }
  
  deviceHomePageStatus = true;
  displayLine1 = GsmNetworkTypeString;
  displayLine2 = GsmOperatorName;
  deviceBusyStatus = 1;
  DisplayHomepage(displayLine1, displayLine2);

  // NOW start the GSM thread — sound is done
  Serial.println("___________GSM Failover Thread started___________");
  threads.addThread(gsmPortLisener);
  wdt.feed();
  Serial.println("[4G Switch] Successfully switched to 4G mode");
}

void loop()
{
  // Feed the watchdog timer to prevent reset
  wdt.feed();
  

  // if (!isEthernetConnected)
  // {
    while (!mqttConnected && enableWifiGsm)
    {
      wdt.feed(); // Feed watchdog during reconnection attempts
      Serial.println(mqttConnected);

      // Track how long we've been stuck in busy/offline
      if (busySinceTime == 0)
      {
        busySinceTime = millis();
      }

      // Track when WiFi first went down (Phase 2 failover timer)
      if (!wifiConnected && wifiDownSince == 0)
      {
        wifiDownSince = millis();
        Serial.println("*** WiFi down — failover timer started ***");
      }
      else if (wifiConnected)
      {
        wifiDownSince = 0; // WiFi recovered — reset failover timer
      }

      // Phase 2: If WiFi has been down for 2 minutes and 4G was verified in Phase 1 → switch
      if (!wifiConnected && wifiDownSince != 0 &&
          millis() - wifiDownSince >= wifiFailoverTimeout &&
          eg25ReadyForFailover)
      {
        Serial.println("*** WiFi down for 2 min — switching to 4G ***");
        switchToEG25Mode();
        break; // exit WiFi reconnect loop (enableWifiGsm is now false)
      }

      // Safety watchdog logic:
      // - WiFi connected but no internet/MQTT for 10 min → reboot
      // - WiFi down (ESP32 hung / serial corruption) for 2 min → reboot
      // - No WiFi available at all → don't reboot (nothing to fix)
      if (wifiConnected && millis() - busySinceTime >= mqttDownRebootTimeout)
      {
        Serial.println("*** WATCHDOG: WiFi up but MQTT down for 10 min - staying offline, retrying ***");
        busySinceTime = millis(); // Reset timer, keep retrying MQTT
      }
      else if (!wifiConnected && millis() - busySinceTime >= busyWatchdogTimeout)
      {
        // WiFi down for 2 min — likely ESP32 hung or serial corruption
        // Check if WiFi router is actually reachable by trying one more connect
        espSerialBusy = true;
        delay(200);
        connectToWiFi(primarySSID, primaryPassword);
        espSerialBusy = false;

        if (!wifiConnected)
        {
          // WiFi router truly not available — don't reboot, just keep waiting
          Serial.println("*** WATCHDOG: No WiFi available - waiting (no reboot) ***");
          busySinceTime = millis(); // Reset timer, check again in 2 min
        }
        else
        {
          // WiFi connected now but wasn't before — ESP32 was likely hung, it recovered
          // Try MQTT, if fails the 10 min timer will handle it
          Serial.println("*** WATCHDOG: WiFi recovered after being stuck ***");
        }
      }

      // Pause listener thread so it doesn't steal our AT command responses
      espSerialBusy = true;
      delay(200); // Give listener time to pause

      if (wifiConnected)
      {
        // WiFi is up but MQTT is down - only reconnect MQTT, don't re-send AT+CWJAP
        Serial.println("WiFi OK, attempting MQTT reconnect...");
        cleanMQTTSession();
        delay(3000);
        connectToMQTT();
      }
      else
      {
        // WiFi is down - do full WiFi reconnection
        connectToWiFi(primarySSID, primaryPassword);
        if (wifiConnected)
        {
          // WiFi reconnected, now connect MQTT
          cleanMQTTSession();
          delay(3000);
          connectToMQTT();
        }
      }

      // Resume listener thread
      espSerialBusy = false;

      if (!mqttConnected)
      {
        Serial.println("Retrying MQTT connection...");
        delay(5000);
      }
      else
      {
        busySinceTime = 0;   // Reset watchdog on successful reconnect
        wifiDownSince = 0;   // Reset 4G failover timer — WiFi/MQTT is healthy again
      }
    // }
  }
  // else
  // {
  //   if (!mqttClient.connected())
  //   {
  //     ethernetConnectToMqtt();
  //   }
  //   mqttClient.loop();
  // }
  if (playSpeaker)
  {
    wdt.feed(); // Feed watchdog before audio playback
    playWavFile();

  }

}

void initializeSerial()
{
  if (serialIn == (Stream *)&Serial)
  {
    Serial.begin(115200);
    while (!Serial)
    {
      // Wait for Serial to initialize
    }
  }
  else
  {
    ((HardwareSerial *)serial)->begin(115200);
  }
}

void initializeSDCard()
{
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("SD Initialization failed!");
    return;
  }
  isSdCardPresent = true;
  Serial.println("Initialization done.");
}

void deleteHexFileIfExists()
{
  if (SD.exists(HEX_FILE_NAME))
  {
    if (SD.remove(HEX_FILE_NAME))
    {
      Serial.print("File ");
      Serial.print(HEX_FILE_NAME);
      Serial.println(" deleted successfully");
    }
    else
    {
      Serial.print("Failed to delete file ");
      Serial.println(HEX_FILE_NAME);
    }
  }
  else
  {
    Serial.print("File ");
    Serial.print(HEX_FILE_NAME);
    Serial.println(" does not exist");
  }
}

void hexInitDownload(const char *url, const char *fileName)
{
  Serial.println("--------------Download HEX Started--------------");
  sendHTTPCGET(url);
  File file = SD.open(fileName, FILE_WRITE);
  uint32_t buffer_addr, buffer_size;
  while (true)
  {
    static int dataSizeLeft = 0;
    static bool isInHTTPCGET = false;
    static String buffer = ""; // Buffer to hold incoming characters
    static bool firstLinePrinted = false;

    while (espSerial.available())
    {
      char receivedChar = espSerial.read();

      // Check if receivedChar is a line break or the buffer is full
      if (receivedChar == '\n' || buffer.length() > 500)
      {
        String response = buffer;
        buffer = "";     // Clear the buffer
        response.trim(); // Remove any leading/trailing white spaces or new lines

        if (response.startsWith("+HTTPCGET"))
        {
          isInHTTPCGET = true;

          // Extract the data size from the line
          int dataSizeIndex = response.indexOf(':') + 1;
          dataSizeLeft = response.substring(dataSizeIndex, response.indexOf(',')).toInt();

          // Extract any data that might be on this line
          int dataStartIndex = response.indexOf(',') + 1;
          if (dataStartIndex != -1 && dataStartIndex < response.length())
          {
            String data = response.substring(dataStartIndex);
            dataSizeLeft -= data.length();

            // print in a new line including ':'
            for (int i = 0; i < data.length(); i++)
            {
              if (data[i] == ':')
              {
                if (storedOutputString.length() != 0 || firstLinePrinted)
                {
                  storedOutputString += "\n";
                }
                else
                {
                  firstLinePrinted = true;
                }
              }
              storedOutputString += data[i];
            }
          }
        }
        else if (isInHTTPCGET)
        {
          // Print the data directly
          if (response.endsWith("OK"))
          {
            response = response.substring(0, response.length() - 2);
          }
          dataSizeLeft -= response.length();
          // print in a new line including ':'
          for (int i = 0; i < response.length(); i++)
          {
            if (response[i] == ':')
            {
              if (storedOutputString.length() != 0 || firstLinePrinted)
              {
                storedOutputString += "\n";
              }
              else
              {
                firstLinePrinted = true;
              }
            }
            storedOutputString += response[i];
          }

          // Serial.print(storedOutputString);
          file.print(storedOutputString);
          if (storedOutputString.endsWith(":00000001FF"))
          {
            isInHTTPCGET = false;
            Serial.println("Ended");
            file.close();
            Serial.println("Initial File Download.");
            return;
          }
          storedOutputString = "";
        }
      }

      else
      {
        // Append received characters to buffer
        buffer += receivedChar;
      }
    }
  }
}

void verifyHexData(const char *url, const char *fileName)
{

  Serial.println("--------------Verify HEX Started--------------");
  DisplayInfo("Downloading update ...!", "Please Wait", codeVersion);
  sendHTTPCGET(url);
  File file = SD.open(fileName, FILE_WRITE);
  uint32_t buffer_addr, buffer_size;
  while (true)
  {
    wdt.feed();
    static int dataSizeLeft = 0;
    static bool isInHTTPCGET = false;
    static String buffer = ""; // Buffer to hold incoming characters

    while (espSerial.available())
    {
      char receivedChar = espSerial.read();

      // Check if receivedChar is a line break or the buffer is full
      if (receivedChar == '\n' || buffer.length() > 500)
      {
        String response = buffer;
        buffer = "";     // Clear the buffer
        response.trim(); // Remove any leading/trailing white spaces or new lines

        if (response.startsWith("+HTTPCGET"))
        {
          isInHTTPCGET = true;

          // Extract the data size from the line
          int dataSizeIndex = response.indexOf(':') + 1;
          dataSizeLeft = response.substring(dataSizeIndex, response.indexOf(',')).toInt();

          // Extract any data that might be on this line
          int dataStartIndex = response.indexOf(',') + 1;
          if (dataStartIndex != -1 && dataStartIndex < response.length())
          {
            String data = response.substring(dataStartIndex);
            dataSizeLeft -= data.length();

            // print in a new line including ':'
            for (int i = 0; i < data.length(); i++)
            {
              if (data[i] == ':')
              {
                storedOutputString += "\n";
              }
              storedOutputString += data[i];
            }
          }
        }
        else if (isInHTTPCGET)
        {
          // Print the data directly
          if (response.endsWith("OK"))
          {
            response = response.substring(0, response.length() - 2);
          }
          dataSizeLeft -= response.length();
          // print in a new line including ':'
          for (int i = 0; i < response.length(); i++)
          {
            if (response[i] == ':')
            {
              storedOutputString += "\n";
            }
            storedOutputString += response[i];
          }

          // Serial.print(storedOutputString);
          file.print(storedOutputString);
          if (storedOutputString.endsWith(":00000001FF"))
          {
            isInHTTPCGET = false;
            Serial.print("Ended");
            file.close();
            Serial.println("Data written to file successfully.");
            DisplayInfo("Installing Update ...!", "Please Wait", codeVersion);
            removeFirstLine(HEX_FILE_NAME);
            delay(3000);
            uint32_t buffer_addr, buffer_size;
            if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
            {
              serialIn->printf("unable to create buffer\n");
              apiError ="unable to create buffer";
              displayError(216);
              serialIn->flush();
              return;
              for (;;)

              {
              }
            }
            serialIn->printf("created buffer = %1luK %s (%08lX - %08lX)\n",
                             buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
                             buffer_addr, buffer_addr + buffer_size);
            int user_input = 2;
            char line[32];
            if (!SD.begin(BUILTIN_SDCARD))
            {
              serialIn->println("SD initialization failed");
              apiError="SD initialization failed";
              displayError(214);
              return;
            }
          //  File hexFile;
            serialIn->println("SD initialization OK");
            hexFile = SD.open(fileName, FILE_READ);
            if (!hexFile)
            {
              serialIn->println("SD file open failed");
              apiError ="SD file open failed";
              displayError(213);
              return;
            }
            serialIn->println("SD file open OK");
            update_firmware(&hexFile, serialIn, buffer_addr, buffer_size);
            serialIn->printf("erase FLASH buffer / free RAM buffer...\n");
            serialIn->println("Came Here.......!");
            firmware_buffer_free(buffer_addr, buffer_size);
            serialIn->flush();
            REBOOT;
          }
          storedOutputString = "";
        }
      }
      else
      {
        // Append received characters to buffer
        buffer += receivedChar;
      }
    }
  }
}

void removeFirstLine(const char *fileName)
{

  File originalFile = SD.open(fileName, FILE_READ);
  if (!originalFile)
  {
    Serial.println("Failed to open file for reading");
    apiError ="Failed to open file for reading";
    displayError(212);
    return;
  }

  // const char* tempFileName = "temp.hex";
  File tempFile = SD.open(HEX_FILE_TEMP, FILE_WRITE);
  if (!tempFile)
  {
    Serial.println("Failed to open temp file for writing");
    apiError = "Failed to open temp file for writing";
     displayError(211);
    originalFile.close();
    return;
  }

  bool firstLineSkipped = false;
  while (originalFile.available())
  {
    String line = originalFile.readStringUntil('\n');
    if (firstLineSkipped)
    {
      tempFile.println(line);
    }
    else
    {
      firstLineSkipped = true; // Skip the first line
    }
  }

  originalFile.close();
  tempFile.close();

  // Verify the content of the temp file
  tempFile = SD.open(HEX_FILE_TEMP, FILE_READ);
  if (!tempFile)
  {
    Serial.println("Failed to open temp file for reading");
    apiError ="Failed to open temp file for reading";
    displayError(220);
    return;
  }

  String firstLine = tempFile.readStringUntil('\n');
  tempFile.close();

  if (firstLine.startsWith(":0200000460009A"))
  {
    Serial.println("File verified");

    // Remove the original file and rename the temp file to the original file name
    SD.remove(fileName);
    SD.rename(HEX_FILE_TEMP, fileName);
  }
  else
  {
    Serial.println("File verification failed, deleting temp file");
    apiError ="File verification failed, deleting temp file";
    displayError(208);
    SD.remove(HEX_FILE_TEMP);
    while (1)
    {
    }
  }

}