#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPIFlash.h>
#include <SPI.h>
// #include "LowPower.h"

#include <ArduinoJson.h>
#include <JamAtm-Vyrsa.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

/******************************************************************* debug ********************************************************************************************/
#define RF_RST 27
#define INT_RF 2
#define INT_RTC 14
#define CS_M 22
#define CS_RF 23
#define VREF_IN 24
#define VREF_EXT 29
#define PG_RXD 16
#define PG_TXD 17
#define SIM_PWR 26
#define SIM_AWK 21
#define LED_SETUP 3
#define PCINT_PIN 18
#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))
/******************************************************************* declarations  ************************************************************************************/
const char apn[] = "wlapn.com";
const char user[] = "vyr";
const char pass[] = "vyr";
SPIFlash flash(CS_M);
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);

Jam jam;
char asignacion[4];                    // The 4 output of the oasis
uint8_t data[RH_RF95_MAX_MESSAGE_LEN]; // Don't put this on the stack:
uint8_t buf[50];
bool rf_flag = false;

typedef enum
{ // This enum contains the possible actions
  REQUEST_MANVAL,
  REQUEST_MANUAL,
  REQUEST_TIME,
  REQUEST_ASSIGNED_VALVES,
  REQUEST_STOP_ALL,
  REQUEST_FULL_MESSAGE
} messages_radio;

/******************************************************************* setup section ************************************************************************************/
void setup()
{
  Serial.begin(115200);
  delay(250);
  pinMode(CS_RF, OUTPUT);
  digitalWrite(CS_RF, HIGH);
  pinMode(RF_RST, OUTPUT);
  digitalWrite(RF_RST, HIGH);
  pinMode(SIM_PWR, OUTPUT);
  pinMode(SIM_AWK, OUTPUT);
  pinMode(PCINT_PIN, INPUT);
  pinMode(LED_SETUP, OUTPUT);
  digitalWrite(SIM_PWR, LOW);
  digitalWrite(SIM_AWK, HIGH);
  manager.init();
  manager.setRetries(1);
  manager.setTimeout(175);
  driver.setTxPower(20, false);
  SWire.begin();
  jam.ledBlink(LED_SETUP, 1000);

  connectSIM();
  connectMqtt();
  delay(5);
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{

  // LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
  mqttClient.loop();
  if (!mqttClient.connected())
  {
    Serial.println("Mqtt connection fail");
    connectMqtt();
    delay(5);
  }
}

/*******************************************************************   functions     ************************************************************************************/

/*
this function connects to internet through sim card
*/
void connectSIM()
{

  delay(500);
  Serial.println("Oasis-Com starts");
  pinMode(SIM_PWR, OUTPUT);
  digitalWrite(SIM_PWR, LOW);
  delay(1200);
  Serial.println("Initializing modem...");
  digitalWrite(SIM_PWR, HIGH);
  delay(1200);
  Serial1.begin(57600);
  modem.restart();

  String modemInfo = modem.getModemInfo();
  if (modemInfo.indexOf("SIM800") == -1)
  {
    Serial1.end();
    digitalWrite(SIM_PWR, LOW);
    delay(1200);
    digitalWrite(SIM_PWR, HIGH);
    delay(1200);
    Serial1.begin(57600);
    modem.restart();
    modemInfo = modem.getModemInfo();
  }

  Serial.print("Modem: ");
  Serial.println(modemInfo);
  Serial1.print("AT+IPR=115200");
  modem.init();
  Serial1.begin(115200);
  delay(2000);
  Serial1.print("AT+IPR=115200");
  modem.simUnlock("8724");
  //Serial.print("AT+CSCLK=2");

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    while (true)
      ;
  }
  Serial.println(" succeed");

  Serial.print("Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass))
  {
    Serial.println(" fail");
    while (true)
      ;
  }
  Serial.println(" succeed");
  int signalq = modem.getSignalQuality();
  Serial.println("Signal quality: " + String(signalq));
}
/*
this function connects to jam-idea mqtt broker
*/
void connectMqtt()
{
  String topic;

  String clientId = "Jam-Idea" + modem.getIMEI();
  mqttClient.setServer("mqtt.pre.hydro-plus.es", 1883);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected())
  {
    if (mqttClient.connect(clientId.c_str(), "hydroplus", "vyrsa"))
      mqttClient.subscribe("oasis-gprs");
    else
    {
      Serial.println("Trying to connect to MQTT...");
      delay(1000);
    }
  }
  Serial.println("Successfully connected to MQTT");
  topic = "test";
  mqttClient.subscribe("test");
  mqttClient.subscribe("uuid_prueba_1_10/manvalve/app");
  mqttClient.publish(topic.c_str(), (const uint8_t *)"HOLA", 4, true);
}
/*
this callback function is called everytime a subscription topic message is received
*/
#define MAX_JSON_SIZE 250
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  uint32_t dir;
  uint8_t aa[] = {'A', '1'};

  char json[MAX_JSON_SIZE];
  String sTopic, jsParsed, str, aux;
  uint8_t i, id, len, val, h, mn, prevChildValves[4];
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);

  String topicStr = topic;
  Serial.println("He ido un topic y pone: ");
  // for (int i = 0; i < length; i++)
  // Serial.write(payload[i]);
  Serial.println("");
  if (topicStr == "uuid_prueba_1_10/manvalve/app")
  {
    //I firts of all parse the message
    jsParsed = jam.byteArrayToString(payload, length);
    JsonObject &parsed = jsonBuffer.parseObject(jsParsed);
    Serial.println("I have received:");
    Serial.println(jsParsed.c_str());
    //I obtein the values of the parser info:
    JsonArray &valves = parsed["valves"];
    String valve_time;
    int valve_number, valve_action, valve_min, valve_hours;
    //Start the game:
    valve_number = valves[0]["v"].as<int>();
    valve_action = valves[0]["action"].as<int>();
    if (valve_action == 1) // If I have to open then I have to obtein the time
    {
      valve_time = valves[0]["time"].as<String>();
      Serial.println(valve_number);
      Serial.println(valve_action);
      Serial.println(valve_time);
      // I parse the valve time
      valve_hours = (valve_time.charAt(0) - '0') * 10 + (valve_time.charAt(1) - '0');
      valve_min = (valve_time.charAt(3) - '0') * 10 + (valve_time.charAt(4) - '0');
      Serial.println(valve_hours);
      Serial.println(valve_min);
      Serial.println("I open the valve");
      uint16_t index = 1;
      send_nodo(index, aa, REQUEST_MANUAL, 1, 2, 1, asignacion);

      //Then I execute the stuff
    }
    else
    {
      Serial.println("I close the valve");
      send_nodo(1, aa, REQUEST_MANUAL, 1, 0, 0, asignacion);
    }
  }
}

void send_nodo(uint16_t order, uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
{
  //First write the destination of the message:
  bool f_man_valve = false, f_time = false, f_asigned = false, f_stop = false, f_manual = false, f_full = false;
  switch (msg)
  {
  case REQUEST_MANUAL:
  {
    f_manual = true;
    Serial.println("MANUAL OPEN");
    uint8_t str_manual[] = "##MANVAL#000#00:00#00";
    if (valve > 99)
    {
      str_manual[9] = '1';
      str_manual[10] = ((valve - 100) / 10) + 0x30;
      str_manual[11] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_manual[9] = '0';
      str_manual[10] = (valve / 10) + 0x30;
      str_manual[11] = (valve % 10) + 0x30;
    }
    str_manual[13] = (hour / 10) + 0x30;
    str_manual[14] = (hour % 10) + 0x30;
    str_manual[16] = (minutes / 10) + 0x30;
    str_manual[17] = (minutes % 10) + 0x30;
    str_manual[19] = uuid[0];
    str_manual[20] = uuid[1];
    data[0] = '_';
    for (int i = 0; i < sizeof(str_manual); i++)
    {
      data[i + 1] = str_manual[i];
    }
    // Serial.println() break;
  }
  }
  manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
