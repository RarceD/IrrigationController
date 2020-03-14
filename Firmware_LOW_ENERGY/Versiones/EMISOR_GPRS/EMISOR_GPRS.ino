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

#define CLIENT_ADDRESS 3
#define SERVER_ADDRESS 2

#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)

#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGLN(...) Serial.print(__VA_ARGS__)

#define MAX_NODE_NUMBER 7
#define MAX_MANUAL_TIMERS 120
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000
#define MAX_NUM_MESSAGES 15
#define MAX_JSON_SIZE 250

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
// const char apn[] = "airtelwap.es";


// const char user[] = "vyr";
// const char pass[] = "vyr";
const char user[] = "vyr";
const char pass[] = "vyr";

typedef enum
{ // This enum contains the possible actions
  REQUEST_MANVAL,
  REQUEST_MANUAL,
  REQUEST_TIME,
  REQUEST_ASSIGNED_VALVES,
  REQUEST_STOP_ALL,
  REQUEST_FULL_MESSAGE
} messages_radio;
typedef struct
{
  uint8_t id;
  uint8_t nChild;
  char devUuid[UUID_LEN];
  uint8_t oasisRfId[MAX_CHILD];
  char oasisUuid[MAX_CHILD][UUID_LEN];
  uint8_t childValves[MAX_CHILD][4];
  uint8_t master_id;
  uint8_t UUID[UUID_LENGTH];
  uint8_t nodes_uuid[UUID_LENGTH][MAX_NODE_NUMBER];
} sysVar;
typedef struct
{
  uint8_t interval;
  uint8_t startDay;
  uint8_t wateringDay;
  uint16_t waterPercent;
  uint8_t start[6][2];
  uint16_t irrigTime[128];
} program;
typedef struct
{
  bool timer_active;                        // When there is any timer active this flag is True
  uint8_t index;                            // For movin into the arrays
  uint8_t valve[MAX_MANUAL_TIMERS];         // Save the number of the valves
  uint32_t millix[MAX_MANUAL_TIMERS];       // Save the millis time timers are just for manual close the valves
  uint32_t time_to_stop[MAX_MANUAL_TIMERS]; // Save the millis time timers are just for manual close the valves
} manual;
typedef struct
{                                        // This struct conteis the messages of radio that are going to be sent but the node is sleeping so it has to wait
  bool request_MANUAL[MAX_NUM_MESSAGES]; // the max number of messages are 4
  bool request_TIME[MAX_NUM_MESSAGES];
  bool request_ASSIGNED_VALVES[MAX_NUM_MESSAGES];
  bool request_STOP_ALL[MAX_NUM_MESSAGES];
  bool request_FULL_MESSAGE[MAX_NUM_MESSAGES];
  uint8_t num_message_flags;
  uint8_t valve_info[3][MAX_NUM_MESSAGES];    //This info is for [num oasis,  tº hours , tºminutes]
  uint8_t assigned_info[5][MAX_NUM_MESSAGES]; //This info is for assigned [num oasis, assig0, assig1, assig2, assig3]
} radio_actions;
typedef struct
{
  bool id_node[5];
} msg_received_all;

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);

RV1805 rtc;
Sleep lowPower;
SimpleTimer timerA, timerB, timerC, timerD, timerE, timerF, timerCheck;
radio_actions radio_waitting_msg;
SPIFlash flash(CS_M);
program prog[TOTAL_PROG];
SoftwareSerial softSerial(PG_RXD, PG_TXD);

char asignacion[4];                    // The 4 output of the oasis
uint8_t data[RH_RF95_MAX_MESSAGE_LEN]; // Don't put this on the stack:
uint8_t buf[50];
bool rf_flag = false;

uint8_t UUID_1[] = {'A', '1'}; // THE EMITER MUST CHANGE THIS IN EVERY ONE
Jam jam;
sysVar sys;
manual man;
msg_received_all ack;
// Don't put this on the stack:

String pg;
int timer_A, timer_B, timer_C, timer_D, timer_E, timer_F;
int timer_check;
char pgData[PG_MAX_LEN];
uint8_t i, j, rfId, cmd;
volatile uint8_t oldPort = 0x00;
volatile bool intButton = false, intRtc = false;
uint8_t valveDef[MAX_CHILD], progDef[TOTAL_PROG];

bool oasis_actions;
uint32_t millix;
uint8_t index_prog_A, index_prog_B, index_prog_C, index_prog_D, index_prog_E, index_prog_F;
bool start_programA, start_programB, start_programC, start_programD, start_programE, start_programF;
bool start_programA_ones, start_programB_ones, start_programC_ones, start_programD_ones, start_programE_ones, start_programF_ones;

uint32_t start = 0;
uint8_t counter_syn, counter_time_sms;

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
  oldPort = PCPIN;
  PCMSK |= (1 << PCINT);
  jam.ledBlink(LED_SETUP, 100);
  softSerial.begin(9600);
  flash.powerUp();
  flash.begin();
  /*
  char first_mem[] = "uuid_prueba_1_10";
  for (uint8_t aux = 0; aux < sizeof(first_mem); aux++)
    sys.devUuid[aux] = first_mem[aux];

  char ack[] = "##OK55##";
  for (int i = 0; i < sizeof(ack); i++)
    sys.ack_msg[i] = ack[i];
  flash.eraseSector(SYS_VAR_ADDR);
  flash.writeAnything(SYS_VAR_ADDR, sys);
  */

  flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  manager.init();
  manager.setRetries(8);
  manager.setTimeout(250);
  driver.setTxPower(20, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.setAlarmMode(0);
  rtc.setAlarmMode(6);
  rtc.setAlarm(10, 0, 0, 0, 0);
  //rtc.setAlarm(0, 30, 0, 0, 0);
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  //rtc.setToCompilerTime();

  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  Serial.println(rtc.stringTime());
  Serial.println(rtc.stringDate());
  jam.ledBlink(LED_SETUP, 1000);
  timer_check = timerCheck.setInterval(20000, check_time);
  radio_waitting_msg.num_message_flags = 0;
  for (int i = 0; i < sizeof(data); i++)
    data[i] = 'z';
  for (int msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  {
    radio_waitting_msg.request_MANUAL[msg] = false; // the max n
    radio_waitting_msg.request_TIME[msg] = false;
    radio_waitting_msg.request_ASSIGNED_VALVES[msg] = false;
    radio_waitting_msg.request_STOP_ALL[msg] = false;
    radio_waitting_msg.request_FULL_MESSAGE[msg] = false;
  }

  print_flash();
  connectSIM();
  connectMqtt();
  delay(5);
  millix = millis();
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{

  mqttClient.loop();
  if (!mqttClient.connected())
  {
    Serial.println("Mqtt connection fail");
    connectMqtt();
    delay(5);
  }
  // listening_pg();
  if (millis() - millix >= 20000)
  {
    millix = millis();
    Serial.println("Alive");
    char json[MAX_JSON_SIZE];
    DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
    JsonObject &root = jsonBuffer.createObject();
    root.set("alive", "true");
    int num_random = random(1, 1230);
    char buf[16];
    sprintf(buf, "%d", num_random);
    const char *p = buf;
    root.set("random", p);
    root.printTo(json);
    //I publis the succes msg
    mqttClient.publish(String(sys.devUuid).c_str(), (const uint8_t *)json, strlen(json), false);
  }
}

/*******************************************************************   functions     ************************************************************************************/

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
  //I suscribe to all the topics
  topic = String(sys.devUuid);
  mqttClient.subscribe(topic.c_str());
  mqttClient.subscribe("uuid_prueba_1_10/manvalve/app");
  mqttClient.subscribe("uuid_prueba_1_10/manprg/app");
  mqttClient.subscribe("uuid_prueba_1_10/program/app");
  mqttClient.subscribe("uuid_prueba_1_10/oasis/app");
}
/*
this callback function is called everytime a subscription topic message is received
*/
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String sTopic, jsParsed, str, aux;
  char json[MAX_JSON_SIZE];

  String topicStr = topic;
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);

  //First I create the msg in correct format and the printed to the console
  jsParsed = jam.byteArrayToString(payload, length);
  Serial.println(jsParsed.c_str());
  JsonObject &parsed = jsonBuffer.parseObject(jsParsed);

  // Then I identify the topic related with
  if (topicStr == (String(sys.devUuid) + "/manvalve/app"))
  {
    Serial.println("Publish in /manvalve/app");
    //I firts of all parse the message
    JsonArray &valves = parsed["valves"];
    //I obtein the values of the parser info:
    String valve_time;
    int valve_number, valve_action, valve_min, valve_hours;
    //Start the game:
    valve_number = valves[0]["v"].as<int>();
    valve_action = valves[0]["action"].as<int>();
    bool send_ack = false; // This is for sendding ack to the web feedback

    if (valve_action == 1) // If I have to open then I have to obtein the time
    {
      send_ack = true;
      valve_time = valves[0]["time"].as<String>();
      // I parse the valve time
      valve_hours = (valve_time.charAt(0) - '0') * 10 + (valve_time.charAt(1) - '0');
      valve_min = (valve_time.charAt(3) - '0') * 10 + (valve_time.charAt(4) - '0');
      ///////////////////////////////////////////////////////////////////
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = valve_hours;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = valve_min;
    }
    else if (valve_action == 0)
    {
      send_ack = true;
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = 0;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = 0;
    }

    if (send_ack)
    {
      //Then I create a json object just for send a ack to the web
      JsonObject &root = jsonBuffer.createObject();
      root.set("id", parsed["id"]);
      root.set("success", "true");
      root.printTo(json);
      //I publis the succes msg
      mqttClient.publish((String(sys.devUuid) + "/manvalve").c_str(), (const uint8_t *)json, strlen(json), false);

      //I send via radio to the receiver
      prepare_message();
      manager.sendtoWait(data, 50, CLIENT_ADDRESS);
    }
  }
  else if (topicStr == String(sys.devUuid) + "/manprg/app")
  {
    Serial.println("Publish in  /manprg/app");
    //I obtein the values of the parser info:
    uint8_t activate = parsed["action"];
    String manual_program;
    manual_program = parsed["prog"].as<String>();
    // char a = ;
    if (manual_program.charAt(0) == 'A' && activate == 1)
      Serial.println("Activate A program");
    else
      Serial.println("Desactivate A program");
  }
  /*
  else*/
  if (topicStr == String(sys.devUuid) + "/oasis/app")
  {
    Serial.println("Publish in /oasis/app assignation way");
    JsonArray &oasis = parsed["oasis"];
    Serial.println("the value of the assignations are:");
    uint8_t assigned_id[4];
    uint8_t ide = oasis[0]["id"];

    for (int d = 0; d < sizeof(assigned_id); d++)
      assigned_id[d] = oasis[0]["assign"][d];

    radio_waitting_msg.request_ASSIGNED_VALVES[radio_waitting_msg.num_message_flags] = true;
    radio_waitting_msg.assigned_info[0][radio_waitting_msg.num_message_flags] = ide-1;
    radio_waitting_msg.assigned_info[1][radio_waitting_msg.num_message_flags] = assigned_id[0]-1;
    radio_waitting_msg.assigned_info[2][radio_waitting_msg.num_message_flags] = assigned_id[1]-1;
    radio_waitting_msg.assigned_info[3][radio_waitting_msg.num_message_flags] = assigned_id[2]-1;
    radio_waitting_msg.assigned_info[4][radio_waitting_msg.num_message_flags] = assigned_id[3]-1;
    //I send via radio to the receiver
    JsonObject &root = jsonBuffer.createObject();
    root.set("id", parsed["id"]);
    root.set("success", "true");
    root.printTo(json);
    //I publis the succes msg
    mqttClient.publish((String(sys.devUuid) + "/oasis").c_str(), (const uint8_t *)json, strlen(json), false);

    prepare_message();
    manager.sendtoWait(data, 50, CLIENT_ADDRESS);
  }
}

void send_nodo(uint16_t &order, uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
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
    for (int i = 0; i < sizeof(str_manual); i++)
    {
      data[order] = str_manual[i];
      order++;
    }
    break;
  }
  case REQUEST_MANVAL:
  {
    f_man_valve = true;
    Serial.println("REQUEST MANVALVE");
    uint8_t str_manval[] = "##MANVAL#000#00:00#00X";
    if (valve > 99)
    {
      str_manval[9] = '1';
      str_manval[10] = ((valve - 100) / 10) + 0x30;
      str_manval[11] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_manval[9] = '0';
      str_manval[10] = (valve / 10) + 0x30;
      str_manval[11] = (valve % 10) + 0x30;
    }

    str_manval[13] = (hour / 10) + 0x30;
    str_manval[14] = (hour % 10) + 0x30;
    str_manval[16] = (minutes / 10) + 0x30;
    str_manval[17] = (minutes % 10) + 0x30;
    str_manval[19] = uuid[0];
    str_manval[20] = uuid[1];

    for (int i = 0; i < sizeof(str_manval); i++)
    {
      data[order] = str_manval[i];
      order++;
    }
    f_man_valve = false;
    break;
  }
  case REQUEST_TIME:
  {

    f_time = true;
    Serial.println("REQUEST TIME");
    rtc.updateTime();
    rtc_node((int)rtc.getHours(), (int)rtc.getMinutes(), (int)rtc.getSeconds(), (int)rtc.getDate(), (int)rtc.getMonth(), order);
    f_time = false;
    break;
  }
  case REQUEST_ASSIGNED_VALVES:
  {
    Serial.println("CHANGE ASIGNATION VALVE");
    f_asigned = true;
    //VALVE es el ID que va del 1 al 250
    uint8_t str_assigned[] = "##ASIGNED#000#000:000:000:000#00X";
    if (valve > 99)
    {
      str_assigned[10] = '1';
      str_assigned[11] = ((valve - 100) / 10) + 0x30;
      str_assigned[12] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_assigned[10] = '0';
      str_assigned[11] = (valve / 10) + 0x30;
      str_assigned[12] = (valve % 10) + 0x30;
    }
    uint8_t index_assigned = 14;

    for (uint8_t out = 0; out < 4; out++, index_assigned += 4)
    {
      if (assigned[out] > 99)
      {
        str_assigned[index_assigned] = '1';
        str_assigned[index_assigned + 1] = ((assigned[out] - 100) / 10) + 0x30;
        str_assigned[index_assigned + 2] = ((assigned[out] - 100) % 10) + 0x30;
      }
      else
      {
        str_assigned[index_assigned] = '0';
        str_assigned[index_assigned + 1] = (assigned[out] / 10) + 0x30;
        str_assigned[index_assigned + 2] = (assigned[out] % 10) + 0x30;
      }
    }
    str_assigned[30] = uuid[0];
    str_assigned[31] = uuid[1];

    for (int j = 0; j < sizeof(str_assigned); j++)
    {
      data[order] = str_assigned[j];
      Serial.write(data[order]);
      order++;
    }
    f_asigned = false;
    break;
  }
  case REQUEST_STOP_ALL:
  {
    Serial.println("STOP ALL");
    f_stop = true;
    f_stop = false;
    uint8_t str_stop[] = "##STOP#ALL#00X";
    str_stop[11] = uuid[0];
    str_stop[12] = uuid[1];
    for (int j = 0; j < sizeof(str_stop); j++)
    {
      data[order] = str_stop[j];
      order++;
    }
    break;
  }
  }
  //manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
void listening_pg()
{
  uint32_t millix;
  bool intPg = false;
  uint8_t i = 0;     //initialize index
  millix = millis(); //start timer
  while ((unsigned long)(millis() - millix) < 100)
  { //while timer doesn't reach 100 ms
    while (softSerial.available())
    {                                  //while there is data to read
      pgData[i++] = softSerial.read(); //read byte and save it
      millix = millis();
    }
  }
  if (i > 10)
    intPg = true;
  if (intPg)
  {                      //if int pg was triggered
    intPg = false;       //clear int pg flag
    pgData[i] = '\0';    //add end of char
    pg = String(pgData); //convert message received into string
    DPRINT("PG: ");
    DPRINTLN(pg);
    // MANVALV START#01#0100#⸮&
    if (pg.indexOf("MANVALV START") > 0)
    {
      Serial.println("ES EL COMANDO DE ABRIR VALVULA MANUAL");
      uint8_t valve_number = (pgData[pg.indexOf("MANVALV START") + 14] - '0') * 10 + (pgData[pg.indexOf("MANVALV START") + 15] - '0');
      uint8_t valve_time_hour = (pgData[pg.indexOf("MANVALV START") + 14 + 3] - '0') * 10 + (pgData[pg.indexOf("MANVALV START") + 15 + 3] - '0');
      uint8_t valve_time_min = (pgData[pg.indexOf("MANVALV START") + 14 + 3 + 2] - '0') * 10 + (pgData[pg.indexOf("MANVALV START") + 15 + 3 + 2] - '0');
      Serial.print(valve_number);
      Serial.print(" valvula - ");
      Serial.print(valve_time_hour);
      Serial.print(" horas - ");
      Serial.print(valve_time_min);
      Serial.println(" minutos. ");
      //start_programA = true;

      //I set a flag, when the time of waking up starts all the messages
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = valve_time_hour;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = valve_time_min;

      //Serial.println("El valor de la radio waitting ha cambiado");
      //Serial.println(radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags]);
      //Serial.println(radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags]);
      //Serial.println(radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++]);

      prepare_message();
      //send_nodo(1, UUID_1, REQUEST_MANUAL, valve_number, valve_time_hour, valve_time_min, asignacion);
    }
    if (pg.indexOf("MANVALV STOP") > 0)
    {
      uint8_t valve_close = (pgData[pg.indexOf("MANVALV STOP") + 13] - '0') * 10 + (pgData[pg.indexOf("MANVALV STOP") + 14] - '0');
      Serial.println("El valor de la válvula a cerrar es el siguinte:");
      Serial.println(valve_close);
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_close;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = 0;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = 0;
      prepare_message();
    }
    else if (pg.indexOf("MANPRG START#") > 0)
    {
      uint8_t prog_name = (pgData[pg.indexOf("MANPRG START#") + 13] - 65);
      Serial.print("Programa:");
      Serial.println(prog_name);
      switch (prog_name)
      {
      case 0:
        start_programA = true;
        break;
      case 1:
        start_programB = true;
        break;
      case 2:
        start_programC = true;
        break;
      case 3:
        start_programD = true;
        break;
      case 4:
        start_programE = true;
        break;
      case 5:
        start_programF = true;
        break;
      default:
        break;
      }
      prepare_message();
    }
    else if (pg.indexOf("STOP ALL") > 0)
    {
      Serial.println("Paro TODO");
      start_programA = false;
      start_programA_ones = false;
      start_programB = false;
      start_programB_ones = false;
      start_programC = false;
      start_programC_ones = false;
      start_programD = false;
      start_programD_ones = false;
      radio_waitting_msg.request_STOP_ALL[radio_waitting_msg.num_message_flags++] = true;
      //send_nodo(1, UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
      prepare_message();
    }
    else if (pg.indexOf("SET TIME#") > 0)
    {
      uint8_t time_hours = (pgData[pg.indexOf("SET TIME#") + 9] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 10] - '0');
      uint8_t time_min = (pgData[pg.indexOf("SET TIME#") + 9 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 10 + 2] - '0');
      uint8_t time_day_week = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6] - '0');
      uint8_t time_year = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3] - '0');
      uint8_t time_month = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3 + 2] - '0');
      uint8_t time_day = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2 + 2 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3 + 2 + 2] - '0');
      Serial.print(" Hora: ");
      Serial.print(time_hours);
      Serial.print(":");
      Serial.print(time_min);
      Serial.print(" Fecha: ");
      Serial.print(time_day_week);
      Serial.print(" Hora: ");
      Serial.print(time_year);
      Serial.print("/");
      Serial.print(time_month);
      Serial.print("/");
      Serial.print(time_day);
      rtc.updateTime();
      Serial.println("");
      rtc.setTime(0, 0, time_min, time_hours, time_day, time_month, time_year, time_day_week);
      Serial.println(rtc.stringTime());
      Serial.println(rtc.stringDate());

      //send_nodo(1, UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
    }
    else if (pg.indexOf("PAIRING#") > 0)
    {
      //I always obtein the number of oasis without one unit due to format 8bit vs 16 bits
      String valve_number = getValue(pg, '#', 1);
      int valve_number_true = (int)strtol(&valve_number[0], NULL, 16);
      // Serial.println(valve_number_true);
      String valve_assigned;
      int oasis_valves[4];
      char temp_valve[4];
      for (int k = 0; k < 4; k++)
      {
        valve_assigned = getValue(getValue(pg, '#', 2), ' ', k);
        oasis_valves[k] = (int)strtol(&valve_assigned[0], NULL, 16);
        //Serial.println(oasis_valves[k]);
      }
      temp_valve[0] = (char)oasis_valves[0];
      temp_valve[1] = (char)oasis_valves[1];
      temp_valve[2] = (char)oasis_valves[2];
      temp_valve[3] = (char)oasis_valves[3];

      //I set the flag for sendding the messages:

      radio_waitting_msg.request_ASSIGNED_VALVES[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.assigned_info[0][radio_waitting_msg.num_message_flags] = (char)valve_number_true;
      radio_waitting_msg.assigned_info[1][radio_waitting_msg.num_message_flags] = temp_valve[0];
      radio_waitting_msg.assigned_info[2][radio_waitting_msg.num_message_flags] = temp_valve[1];
      radio_waitting_msg.assigned_info[3][radio_waitting_msg.num_message_flags] = temp_valve[2];
      radio_waitting_msg.assigned_info[4][radio_waitting_msg.num_message_flags] = temp_valve[3];

      for (uint8_t k = 0; k < 5; k++)
        Serial.println(radio_waitting_msg.assigned_info[k][radio_waitting_msg.num_message_flags]);
      radio_waitting_msg.num_message_flags++;
      prepare_message();
      //send_nodo(1, UUID_1, REQUEST_ASSIGNED_VALVES, valve_number_true + 1, 0, 0, temp_valve);
    }
    else if (pg.indexOf("SELECTOR#06") > 0)
    {
      Serial.println("STOP");
      start_programA = false;
      start_programA_ones = false;
      start_programB = false;
      start_programB_ones = false;
      start_programC = false;
      start_programC_ones = false;
      start_programD = false;
      start_programD_ones = false;
      radio_waitting_msg.request_STOP_ALL[radio_waitting_msg.num_message_flags++] = true;
      prepare_message();
    }
  }
}
void prepare_message()
{
  uint8_t long_message[4]; // = "_1581603360_XNX_##STOP#ALL#00_X_##ASIGNED#000#000:000:000:000#00_X_##MANVAL#000#00:00#00_X_##STOP#ALL#00";
  for (int i = 0; i < sizeof(data) / 2; i++)
    data[i] = 'z';

  // First obtein the timestamp and save to DATA radio
  //String timestamp = String(rtc.getTimestamp()); // from string I convert to char[]
  uint16_t index = 0;
  long_message[index++] = '_';
  //for (index = 1; index < timestamp.length() + 1; index++)
  //  long_message[index] = timestamp.charAt(index - 1);
  long_message[index++] = '_';

  //Second implement a system for adding more messages to DATA radio
  //The possibilities are:
  //##ASIGNED#000#000:000:000:000#00
  //##MANVAL#000#00:00#00
  //##STOP#ALL#00

  uint8_t msg_number = 0, msg_position = 0;
  //I just copy the buffer to data radio and send it
  memcpy(data, long_message, sizeof(long_message));
  counter_time_sms++;
  // Serial.println(counter_time_sms);
  if (counter_time_sms >= 10)
  {
    counter_time_sms = 0;
  }
  // send_nodo(index, UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
  //Serial.println("EL ENVIO EN MILLIS COMIENZA A LAS: ");
  //Serial.println(millis());
  //2.2 introduce the messages:
  for (uint8_t msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  {
    if (radio_waitting_msg.request_MANUAL[msg])
      send_nodo(index, UUID_1, REQUEST_MANUAL, radio_waitting_msg.valve_info[0][msg], radio_waitting_msg.valve_info[1][msg], radio_waitting_msg.valve_info[2][msg], asignacion);
    else if (radio_waitting_msg.request_ASSIGNED_VALVES[msg])
    {
      char temp_assigned[] = {radio_waitting_msg.assigned_info[1][msg], radio_waitting_msg.assigned_info[2][msg], radio_waitting_msg.assigned_info[3][msg], radio_waitting_msg.assigned_info[4][msg]};
      send_nodo(index, UUID_1, REQUEST_ASSIGNED_VALVES, radio_waitting_msg.assigned_info[0][msg], 0, 0, temp_assigned);
    }
    else if (radio_waitting_msg.request_STOP_ALL[msg])
      send_nodo(index, UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
  }
  radio_waitting_msg.num_message_flags = 0;
  manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
  for (int msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  {
    radio_waitting_msg.request_MANUAL[msg] = false; // the max n
    radio_waitting_msg.request_TIME[msg] = false;
    radio_waitting_msg.request_ASSIGNED_VALVES[msg] = false;
    radio_waitting_msg.request_STOP_ALL[msg] = false;
    radio_waitting_msg.request_FULL_MESSAGE[msg] = false;
  }
  radio_waitting_msg.num_message_flags = 0;
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
void rtcInt() //this callback funtion is called when rtc interrupt is triggered
{
  intRtc = true; //set flag to indicate that rtc interrupt was triggered
}
void rtc_node(int hour, int minute, int second, int day, int month, uint16_t &order)
{
  uint8_t send_time[] = "##TIME:H:XX/M:XX/S:XX/D:XX/M:XX/";
  send_time[7 + 2] = (hour / 10) + 0x30;
  send_time[8 + 2] = (hour % 10) + 0x30;
  send_time[12 + 2] = (minute / 10) + 0x30;
  send_time[13 + 2] = (minute % 10) + 0x30;
  send_time[17 + 2] = (second / 10) + 0x30;
  send_time[18 + 2] = (second % 10) + 0x30;
  send_time[22 + 2] = (day / 10) + 0x30;
  send_time[23 + 2] = (day % 10) + 0x30;
  send_time[27 + 2] = (month / 10) + 0x30;
  send_time[28 + 2] = (month % 10) + 0x30;
  for (int i = 0; i < sizeof(send_time); i++)
    data[order++] = send_time[i];
  //for (int i = 0; i < sizeof(send); i++)
  //  Serial.write(send[i]);
  //manager.sendtoWait(data, sizeof(send), CLIENT_ADDRESS);
}
void check_time()
{

  //I sleep the device for the first

  //Serial.print("check_time: ");
  rtc.updateTime();
  //Serial.println(rtc.stringTime());

  for (uint8_t index_program = 0; index_program < 6; index_program++)
  {
    for (uint8_t index_time_h = 0; index_time_h < 6; index_time_h++)
    {
      if (prog[index_program].start[index_time_h][0] == rtc.getHours())
      {
        if (prog[index_program].start[index_time_h][1] == rtc.getMinutes())
        {
          Serial.println("ES LA HORA BUENA");
          switch (index_program)
          {
          case 0:
            if (!start_programA_ones)
            {
              Serial.println("Encender programa A");
              start_programA = true;
              start_programA_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa A");
            break;
          case 1:
            if (!start_programB_ones)
            {
              Serial.println("Encender programa B");
              start_programB = true;
              start_programB_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa B");
            break;
          case 2:
            if (!start_programC_ones)
            {
              Serial.println("Encender programa C");
              start_programC = true;
              start_programC_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa C");
            break;
          case 3:
            if (!start_programD_ones)
            {
              Serial.println("Encender programa D");
              start_programD = true;
              start_programD_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa D");
            break;
          case 4:
            if (!start_programE_ones)
            {
              Serial.println("Encender programa E");
              start_programE = true;
              start_programE_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa E");
            break;
          case 5:
            if (!start_programF_ones)
            {
              Serial.println("Encender programa F");
              start_programF = true;
              start_programF_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa F");
            break;
          default:
            Serial.println("never");
            break;
          }
        }
      } // check if the hours are fix and we can start a program
    }
  }
}
void print_flash()
{
  Serial.print("The App name is: ");
  for (int a = 0; a < UUID_LEN; a++)
    Serial.write(sys.devUuid[a]);
  Serial.println(" ");
}