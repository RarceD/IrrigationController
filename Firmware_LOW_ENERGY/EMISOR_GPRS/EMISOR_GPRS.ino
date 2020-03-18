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

#include <JamSleep.h>

#define CLIENT_ADDRESS 3
#define SERVER_ADDRESS 2

#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)

#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGLN(...) Serial.println(__VA_ARGS__)

#define MAX_NODE_NUMBER 7
#define MAX_MANUAL_TIMERS 120
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000
#define MAX_NUM_MESSAGES 15

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

bool ones_time;
bool gprs_on = true, mode = false, comError[MAX_CHILD];

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
  // uint8_t buf[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, '0', '1', 0x23, '0', '0', '0', '1', 0x23, 0x03, 0, 0};
  // calcrc((char *)buf, sizeof(buf) - 2);
  // softSerial.write(buf, sizeof(buf));

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
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  // rtc.setAlarmMode(0);
  rtc.setAlarmMode(6);
  rtc.setAlarm(10, 0, 0, 0, 0);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  //rtc.setAlarm(0, 30, 0, 0, 0);
  // rtc.setToCompilerTime();

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
  delay(50);
  millix = millis();
  // char json[15];
  // DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  // JsonObject &root = jsonBuffer.createObject();
  // root.set("alive", "true");
  // root.printTo(json);
  // mqttClient.publish(String(sys.devUuid).c_str(), (const uint8_t *)json, strlen(json), false);
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

  if (millis() - millix >= 20000) // Printing that I am not dead
  {
    Serial.println("Alive");
    char send[] = "NOT_DEAD";
    mqttClient.publish(String(sys.devUuid).c_str(), (const uint8_t *)send, 9, false);
    millix = millis();
    //open_valve_pg(true, 1, 2, 5);
    //pgCommand(cmd_stop_manvalv, sizeof(cmd_stop_manvalv));
    // cmd_start_manvalv[16] = '0';
    // cmd_start_manvalv[17] = '1';
    // cmd_start_manvalv[19] = '0';
    // cmd_start_manvalv[20] = '1';
    // cmd_start_manvalv[21] = '0';
    // cmd_start_manvalv[22] = '1';
    // pgCommand(cmd_start_manvalv, sizeof(cmd_start_manvalv));
    // //delay(4000);
    //open_valve_pg(false, 1, 2, 5);
  }
  /*
  if (Serial.available())
  {
    int a = Serial.read();
    Serial.println(a);
    if (a = 97)
    {
      Serial.println("a");
    }
  }
  */
  // if (ones_time)
  // {
  // }
  //   cmd_start_manvalv[16] = '0';
  //   cmd_start_manvalv[17] = '1';
  //   cmd_start_manvalv[19] = '0';
  //   cmd_start_manvalv[20] = '1';
  //   cmd_start_manvalv[21] = '0';
  //   cmd_start_manvalv[22] = '1';
  //   pgCommand(cmd_start_manvalv, sizeof(cmd_start_manvalv));
  //   ones_time = false;
  // else
  // {
  //   cmd_stop_manvalv[15] = '0';
  //   cmd_stop_manvalv[16] = '1';
  //   pgCommand(cmd_stop_manvalv, sizeof(cmd_stop_manvalv));
  //   ones_time = true;
  // }
  /*
    uint16_t b, c;

    analogReference(INTERNAL);
    for (i = 0; i < 3; i++)
    {
      b = analogRead(PA0);
      delay(1);
    }
    analogReference(DEFAULT);
    for (i = 0; i < 3; i++)
    {
      c = analogRead(PA0);
      delay(1);
    }
    Serial.print(F("Voltaje Vout: "));
    Serial.println(b * 0.0190 - 2.0);
    Serial.print(F("Voltaje Vin:  "));
    Serial.println((b * 2.5132) / c);
    Serial.print(F("Battery: "));
    Serial.print(analogRead(A5) / 1024 * c);
    Serial.println(F("V"));

    // Esta mierda no funciona
    uint16_t b, c;
    analogReference(INTERNAL);
    for (i = 0; i < 3; i++)
    {
      b = analogRead(PA0);
      delay(1);
    }
    analogReference(DEFAULT);
    for (i = 0; i < 3; i++)
    {
      c = analogRead(PA0);
      delay(1);
    }
    Serial.print(F("Voltaje Vout: "));
    Serial.println(b * 0.0190);
    Serial.print(F("Voltaje Vin:  "));
    Serial.println((b * 2.5132) / c);

    Serial.print(F("Battery: "));
    Serial.println(batLevel());
}
    */
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
  if (!modem.gprsConnect(apn, user_apn, pass_apn))
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

  String clientId = "Arced" + modem.getIMEI();
  mqttClient.setServer(server_mqtt, 1883);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected())
  {
    if (mqttClient.connect(clientId.c_str(), user_mqtt, pass_mqtt))
    {
      mqttClient.subscribe(String(sys.devUuid).c_str());
      topic = String(sys.devUuid) + "/+/app";
      mqttClient.subscribe(topic.c_str());
    }
    else
    {
      Serial.println("Trying to connect to MQTT...");
      delay(1000);
    }
  }
  Serial.println("Successfully connected to MQTT");
  //I suscribe to all the topics
}
/*
this callback function is called everytime a subscription topic message is received
*/
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char json[MAX_JSON_SIZE];
  String sTopic, jsParsed;
  uint8_t i, id, len, val, h, mn, prevChildValves[4];
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  // root.set("id", parsed["id"]);

  if (!length)
    return;
  jsParsed = jam.byteArrayToString(payload, length);
  JsonObject &parsed = jsonBuffer.parseObject(jsParsed);
  DPRINT("Mqtt received: ");
  DPRINTLN(jsParsed.c_str());
  String identifier = parsed["id"].as<String>(); // This is for the sendding function app

  sTopic = String(topic);

  // Then I identify the topic related with
  if (sTopic.indexOf("manvalve") != -1)
  {
    Serial.println("Publish in /manvalve/app");
    //I firts of all parse the message
    JsonArray &valves = parsed["valves"];
    //I obtein the values of the parser info:
    String valve_time;
    int valve_number[4], valve_action[4], valve_min[4], valve_hours[4];
    //Start the game:
    for (uint8_t index_oasis = 0; index_oasis < valves.size(); index_oasis++)
    {
      valve_number[index_oasis] = valves[index_oasis]["v"].as<int>();
      valve_action[index_oasis] = valves[index_oasis]["action"].as<int>();
      if (valve_action[index_oasis] == 1) // If I have to open then I have to obtein the time
      {
        valve_time = valves[index_oasis]["time"].as<String>();
        // I parse the valve time
        valve_hours[index_oasis] = (valve_time.charAt(0) - '0') * 10 + (valve_time.charAt(1) - '0');
        valve_min[index_oasis] = (valve_time.charAt(3) - '0') * 10 + (valve_time.charAt(4) - '0');
        ///////////////////////////////////////////////////////////////////
        LOGLN(" ");
        LOG("El valor de válvula es la:");
        LOGLN(valve_number[index_oasis]);
        LOG("El tiempo que queda es de: ");
        LOG(valve_hours[index_oasis]);
        LOG(":");
        LOGLN(valve_min[index_oasis]);
        LOGLN(" ");

        // radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
        // radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number[index_oasis];
        // radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = valve_hours[index_oasis];
        // radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = valve_min[index_oasis];
      }
      else if (valve_action[index_oasis] == 0)
      {
        LOG("APAGAR LA:");
        LOGLN(valve_number[index_oasis]);
        // radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
        // radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number[index_oasis];
        // radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = 0;
        // radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = 0;
      }
      if (valve_number[index_oasis] <= 14) //Check if the valve is for me and then sent to the pg
      {
        action_valve_pg(valve_action[index_oasis], valve_number[index_oasis], valve_hours[index_oasis], valve_min[index_oasis]);
        delay(1500);
      }
    }
    send_web("/manvalve", sizeof("/manvalve"), identifier.c_str());

    //I send via radio to the receiver
    //prepare_message();
    //manager.sendtoWait(data, 50, CLIENT_ADDRESS);
  }
  else if (sTopic.indexOf("manprog") != -1)
  {
    Serial.println("Publish in  /manprog/app");
    //I obtein the values of the parser info:
    uint8_t activate = parsed["action"];
    Serial.println(activate);
    String manual_program;
    manual_program = parsed["prog"].as<String>();
    Serial.println(manual_program);

    if (manual_program.charAt(0) == 'A')
      if (activate == 1)
        Serial.println("Activate A program");
      else
        Serial.println("Desactivate A program");
    else if (manual_program.charAt(0) == 'B')
      if (activate == 1)
        Serial.println("Activate B program");
      else
        Serial.println("Desactivate B program");
    else if (manual_program.charAt(0) == 'C')
      if (activate == 1)
        Serial.println("Activate C program");
      else
        Serial.println("Desactivate C program");
    else if (manual_program.charAt(0) == 'D')
      if (activate == 1)
        Serial.println("Activate D program");
      else
        Serial.println("Desactivate D program");
    else if (manual_program.charAt(0) == 'E')
      if (activate == 1)
        Serial.println("Activate E program");
      else
        Serial.println("Desactivate E program");
    else if (manual_program.charAt(0) == 'F')
      if (activate == 1)
        Serial.println("Activate F program");
      else
        Serial.println("Desactivate F program");
    send_web("/manprog", sizeof("/manprog"), identifier.c_str());
  }
  else if (sTopic.indexOf("oasis") != -1)
  {
    Serial.println("Publish in /oasis/app");
    JsonArray &oasis = parsed["oasis"];
    Serial.println("The value of the assignations are:");
    uint8_t assigned_id[4];
    uint8_t ide[15];
    for (uint8_t oasis_num = 0; oasis_num < oasis.size(); oasis_num++)
    {
      ide[oasis_num] = oasis[oasis_num]["id"];
      LOG("El oasis ");
      LOGLN(ide[oasis_num]);
      LOG("Tiene Asignadas las siguientes salidas: ");
      for (uint8_t d = 0; d < sizeof(assigned_id); d++)
      {
        assigned_id[d] = oasis[oasis_num]["assign"][d];
        LOG(assigned_id[d]);
        LOG(", ");
      }
      LOGLN("");
    }
    send_web("/oasis", sizeof("/oasis"), identifier.c_str());
    // radio_waitting_msg.request_ASSIGNED_VALVES[radio_waitting_msg.num_message_flags] = true;
    // radio_waitting_msg.assigned_info[0][radio_waitting_msg.num_message_flags] = ide - 1;
    // radio_waitting_msg.assigned_info[1][radio_waitting_msg.num_message_flags] = assigned_id[0] - 1;
    // radio_waitting_msg.assigned_info[2][radio_waitting_msg.num_message_flags] = assigned_id[1] - 1;
    // radio_waitting_msg.assigned_info[3][radio_waitting_msg.num_message_flags] = assigned_id[2] - 1;
    // radio_waitting_msg.assigned_info[4][radio_waitting_msg.num_message_flags] = assigned_id[3] - 1;
    //I send via radio to the receiver
    // prepare_message();
    // manager.sendtoWait(data, 50, CLIENT_ADDRESS);
  }
  else if (sTopic.indexOf("program") != -1)
  {
    //I parse the letter of the program
    String manual_program = parsed["prog"].as<String>();
    //I parse the array of starts and valves, if there's any
    JsonArray &starts = parsed["starts"];
    if (starts.success())
    {
      LOG("Exists Starts");
      String str_time;
      uint8_t time_prog[6][2];
      for (uint8_t index_starts = 0; index_starts < starts.size(); index_starts++)
      {
        str_time = starts[index_starts].as<String>();
        time_prog[index_starts][0] = (str_time.charAt(0) - '0') * 10 + (str_time.charAt(1) - '0'); //save hours
        time_prog[index_starts][1] = (str_time.charAt(3) - '0') * 10 + (str_time.charAt(4) - '0'); //save minutes
        LOG("El valor del arranque ");
        LOG(index_starts);
        LOG(" es de: ");
        LOG(time_prog[index_starts][0]);
        LOG(":");
        LOGLN(time_prog[index_starts][1]);
      }
    }
    JsonArray &valves = parsed["valves"];
    if (valves.success())
    {
      LOG("Exists Valves");
      uint8_t time_valves[128][2]; //This is a huge and ridiculous
      String irrig_time;           //The string for the 00:00 time format
      uint8_t number_valves;
      for (uint8_t index_valves = 0; index_valves < valves.size(); index_valves++)
      {
        irrig_time = valves[index_valves]["time"].as<String>();
        number_valves = valves[index_valves]["v"].as<uint8_t>() - 1;
        time_valves[number_valves][0] = (irrig_time.charAt(0) - '0') * 10 + (irrig_time.charAt(1) - '0'); //save hours
        time_valves[number_valves][1] = (irrig_time.charAt(3) - '0') * 10 + (irrig_time.charAt(4) - '0'); //save minutes
        LOG("Los tiempos de la válvula ");
        LOG(number_valves + 1);
        LOG(" son: ");
        LOG(time_valves[number_valves][0]);
        LOG(":");
        LOGLN(time_valves[number_valves][1]);
      }
    }
    LOGLN(manual_program);
    send_web("/program", sizeof("/program"), identifier.c_str());
  }
}
void send_web(char *topic, unsigned int length, const char *id)
{
  String ack_topic = String(topic);
  String identifier = String(id);
  char json[MAX_JSON_SIZE];

  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  JsonObject &root = jsonBuffer.createObject();
  root.set("id", identifier);
  root.set("success", "true");
  root.printTo(json);

  mqttClient.publish((String(sys.devUuid) + ack_topic).c_str(), (const uint8_t *)json, strlen(json), false);
}
/*
void open_valve_pg(bool state, uint8_t valve, uint8_t time_hours, uint8_t time_minutes)
{
  //First add the number of the valve
  if (valve > 99)
  {
    valve -= 100;
    cmd_start_manvalv[15] = '1';
  }
  else
    cmd_start_manvalv[15] = '0';

  if (valve > 9)
  {
    cmd_start_manvalv[16] = valve / 10 + '0';
    cmd_start_manvalv[17] = (valve - (valve / 10) * 10) + '0';
  }
  else
  {
    cmd_start_manvalv[16] = '0';
    cmd_start_manvalv[17] = valve + '0';
  }
  if (state) // I open the valve
  {
    //I add the time
    if (time_hours > 9)
    {
      cmd_start_manvalv[19] = '1'; //hours
      cmd_start_manvalv[20] = (time_hours - 10) + '0';
    }
    else
    {
      cmd_start_manvalv[19] = '0'; //hours
      cmd_start_manvalv[20] = time_hours + '0';
    }
    if (time_minutes > 9)
    {
      cmd_start_manvalv[21] = (time_minutes / 10) + '0';
      cmd_start_manvalv[22] = (time_minutes - (time_minutes / 10) * 10) + '0'; //minutes
    }
    else
    {
      cmd_start_manvalv[21] = '0'; //hours
      cmd_start_manvalv[22] = time_minutes + '0';
    }
    // pgCommand(cmd_start_manvalv, sizeof(cmd_start_manvalv));
  }
  else // I close the valve
  {
    cmd_start_manvalv[19] = '0';
    cmd_start_manvalv[20] = '0';
    cmd_start_manvalv[21] = '0';
    cmd_start_manvalv[22] = '0';
  }
}
*/
void send_nodo_rf(uint16_t &order, uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
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
      send_nodo_rf(index, UUID_1, REQUEST_MANUAL, radio_waitting_msg.valve_info[0][msg], radio_waitting_msg.valve_info[1][msg], radio_waitting_msg.valve_info[2][msg], asignacion);
    else if (radio_waitting_msg.request_ASSIGNED_VALVES[msg])
    {
      char temp_assigned[] = {radio_waitting_msg.assigned_info[1][msg], radio_waitting_msg.assigned_info[2][msg], radio_waitting_msg.assigned_info[3][msg], radio_waitting_msg.assigned_info[4][msg]};
      send_nodo_rf(index, UUID_1, REQUEST_ASSIGNED_VALVES, radio_waitting_msg.assigned_info[0][msg], 0, 0, temp_assigned);
    }
    else if (radio_waitting_msg.request_STOP_ALL[msg])
      send_nodo_rf(index, UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
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

uint8_t batLevel()
{
  float res;
  uint8_t a;
  uint16_t b;
  ADCSRA |= (1 << 7);
  analogReference(INTERNAL);
  for (a = 0; a < 5; a++)
  {
    b = analogRead(VREF_IN);
    delay(1);
  }
  res = -0.0186 * pow(b, 2);
  res += 8.7291 * b - 922;
  if (res < 0)
    res = 0;
  if (res > 100)
    res = 100;
  return (res);
}
void pgCommand(uint8_t command[], uint8_t len)
{

  uint32_t millix;
  uint8_t i, attempt = 3;
  jam.calcrc((char *)command, len - 2);

  while (attempt)
  {
    if (gprs_on)
      mqttClient.loop();
    softSerial.write(command, len); //send command to PG
    millix = millis();              //start millis count
    while ((millis() - millix) < PG_TIMEOUT)
      ;                                        //wait one second
    i = 0;                                     //initialize pgdata index
    while (softSerial.available())             //while bytes available in serial port
      pgData[i++] = softSerial.read();         //read it
    if ((i == ACK_SIZE) && (pgData[2] == 'N')) //if is a not acknowledge message
      attempt--;                               //try again                                                                              //means sucess replay
    else                                       //otherwise
      break;
  }
  pgData[i - 1] = '\0';
}

int calcrc(char ptr[], int length)
{
  char i;
  int crc = 0;

  while (--length >= 0)
  {
    crc = crc ^ (int)*ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    } while (--i);
  }
  *ptr = crc & 0xff;
  *(++ptr) = crc >> 8;
  return (crc);
}
void action_valve_pg(bool state, uint8_t valve, uint8_t time_hours, uint8_t time_minutes)
{

  if (state) // I open the valve
  {
    //First add the number of the valve;
    if (valve > 9)
    {
      cmd_start_manvalv[16] = valve / 10 + '0';
      cmd_start_manvalv[17] = (valve - (valve / 10) * 10) + '0';
    }
    else
    {
      cmd_start_manvalv[16] = '0';
      cmd_start_manvalv[17] = valve + '0';
    }
    //I add the time
    if (time_hours > 9)
    {
      cmd_start_manvalv[19] = '1'; //hours
      cmd_start_manvalv[20] = (time_hours - 10) + '0';
    }
    else
    {
      cmd_start_manvalv[19] = '0'; //hours
      cmd_start_manvalv[20] = time_hours + '0';
    }
    if (time_minutes > 9)
    {
      cmd_start_manvalv[21] = (time_minutes / 10) + '0';
      cmd_start_manvalv[22] = (time_minutes - (time_minutes / 10) * 10) + '0'; //minutes
    }
    else
    {
      cmd_start_manvalv[21] = '0'; //hours
      cmd_start_manvalv[22] = time_minutes + '0';
    }
    calcrc((char *)cmd_start_manvalv, sizeof(cmd_start_manvalv) - 2);
    softSerial.write(cmd_start_manvalv, sizeof(cmd_start_manvalv));
    // pgCommand(cmd_start_manvalv, sizeof(cmd_start_manvalv));
  }
  else // I close the valve
  {
    if (valve > 9)
    {
      cmd_stop_manvalv[15] = valve / 10 + '0';
      cmd_stop_manvalv[16] = (valve - (valve / 10) * 10) + '0';
    }
    else
    {
      cmd_stop_manvalv[15] = '0';
      cmd_stop_manvalv[16] = valve + '0';
    }
    calcrc((char *)cmd_stop_manvalv, sizeof(cmd_stop_manvalv) - 2);
    softSerial.write(cmd_stop_manvalv, sizeof(cmd_stop_manvalv));
  }
}
void action_prog_pg(bool state, char program)
{
  if (state)
  {
    cmd_start_manprg[15] = program;
    calcrc((char *)cmd_start_manprg, sizeof(cmd_start_manprg) - 2);
    softSerial.write(cmd_start_manprg, sizeof(cmd_start_manprg));
  }
  else
  {
    cmd_stop_manprg[14] = program;
    calcrc((char *)cmd_stop_manprg, sizeof(cmd_stop_manprg) - 2);
    softSerial.write(cmd_stop_manprg, sizeof(cmd_stop_manprg));
  }
}