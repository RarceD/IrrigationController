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
  bool id_node[5];
} msg_received_all;

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);

RV1805 rtc;
Sleep lowPower;
// SimpleTimer timerA, timerB, timerC, timerD, timerE, timerF, timerCheck;
// radio_actions radio_waitting_msg;
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
// manual man;
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
// uint8_t index_prog_A, index_prog_B, index_prog_C, index_prog_D, index_prog_E, index_prog_F;
// bool start_programA, start_programB, start_programC, start_programD, start_programE, start_programF;
// bool start_programA_ones, start_programB_ones, start_programC_ones, start_programD_ones, start_programE_ones, start_programF_ones;

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
  flash.powerUp();
  flash.begin();
  /*
  char first_mem[] = "uuid_prueba_1_10";
  for (uint8_t aux = 0; aux < sizeof(first_mem); aux++)
    sys.devUuid[aux] = first_mem[aux];

  char ack[] = "##OK55##";
  for (int i = 0; i < sizeof(ack); i++)
    // sys.ack_msg[i] = ack[i];
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
  // timer_check = timerCheck.setInterval(20000, check_time);
  // radio_waitting_msg.num_message_flags = 0;
  for (int i = 0; i < sizeof(data); i++)
    data[i] = 'z';
  //for (int msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  //{
  //  radio_waitting_msg.request_MANUAL[msg] = false; // the max n
  //  radio_waitting_msg.request_TIME[msg] = false;
  //  radio_waitting_msg.request_ASSIGNED_VALVES[msg] = false;
  //  radio_waitting_msg.request_STOP_ALL[msg] = false;
  //  radio_waitting_msg.request_FULL_MESSAGE[msg] = false;
  //}
  print_flash();
  connectSIM();
  connectMqtt();
  delay(50);
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
  // Every 20 seconds I publish that I am ALIVE
  if (millis() - millix >= 20000) // Printing that I am not dead
  {
    uint16_t b, c;
    analogReference(INTERNAL);
    for (int i = 0; i < 3; i++)
    {
      b = analogRead(PA0);
      delay(1);
    }
    analogReference(DEFAULT);
    for (int i = 0; i < 3; i++)
    {
      c = analogRead(PA0);
      delay(1);
    }
    float bat = b * 0.0190 - 2.0;
    char json[100];
    DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
    JsonObject &root = jsonBuffer.createObject();
    root.set("id", sys.devUuid);
    root.set("battery", bat);
    root.set("success", "true");
    root.printTo(json);
    mqttClient.publish(String("debug_vyr").c_str(), (const uint8_t *)json, strlen(json), false);
    millix = millis();
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
      // mqttClient.subscribe(String(sys.devUuid).c_str());
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
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char json[MAX_JSON_SIZE];
  String sTopic, jsParsed;
  uint8_t i, id, len, val, h, mn, prevChildValves[4];
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  if (!length)
    return;
  jsParsed = jam.byteArrayToString(payload, length);
  JsonObject &parsed = jsonBuffer.parseObject(jsParsed);
  DPRINT("Mqtt received: ");
  DPRINTLN(jsParsed.c_str());
  delay(10);
  String identifier = parsed["id"].as<String>(); // This is for the sendding function app
  sTopic = String(topic);
  // Then I identify the topic related with
  if (sTopic.indexOf("manvalve") != -1)
  {
    Serial.println("Publish in /manvalve/app");
    send_web("/manvalve", sizeof("/manvalve"), identifier.c_str());
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
        LOG("El valor de válvula es la:");
        LOG(valve_number[index_oasis]);
        LOG("durante ");
        LOG(valve_hours[index_oasis]);
        LOG(":");
        LOGLN(valve_min[index_oasis]);
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
    //I send via radio to the receiver
  }
  else if (sTopic.indexOf("manprog") != -1)
  {
    Serial.println("Publish in  /manprog/app");
    //I obtein the values of the parser info:
    uint8_t activate = parsed["action"];
    // Serial.println(activate);
    String manual_program;
    manual_program = parsed["prog"].as<String>();
    // I first send it to
    send_web("/manprog", sizeof("/manprog"), identifier.c_str()); // I send ack to the app
    delay(500);
    action_prog_pg(activate, manual_program.charAt(0)); //I send the command tom PG
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
  }
  else if (sTopic.indexOf("program") != -1)
  {
    //I parse the letter of the program
    send_web("/program", sizeof("/program"), identifier.c_str());
    String manual_program = parsed["prog"].as<String>();
    //I parse the array of starts and valves, if there's any
    JsonArray &starts = parsed["starts"];
    //I determin the positions of the PG in which I have to write
    uint16_t position_starts = 416; // This is the position of the starts in PG EEPROM memmory
    uint16_t mem_pos = 1024;         // This is the position of the starts in PG EEPROM memmory
    if (manual_program.charAt(0) == 'B')
    {
      position_starts = 428;
      mem_pos = 1152;
    }
    else if (manual_program.charAt(0) == 'C')
    {
      position_starts = 440;
      mem_pos = 1280;
    }
    else if (manual_program.charAt(0) == 'D')
    {
      position_starts = 452;
      mem_pos = 1408;
    }
    else if (manual_program.charAt(0) == 'E')
    {
      position_starts = 464;
      mem_pos = 1536;
    }
    else if (manual_program.charAt(0) == 'F')
    {
      position_starts = 476;
      mem_pos = 1664;
    }
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
      uint16_t start_time_hours = 0, start_time_min = 0;
      String mem_starts_h;
      for (int index_complet = 0; index_complet < starts.size(); index_complet++)
      {
        //First format the info y PG language
        start_time_hours = time_to_pg_format(0, time_prog[index_complet][0]);
        String mem_time_start_hours = String(start_time_hours, HEX);
        mem_time_start_hours.toUpperCase();
        if (mem_time_start_hours.length() != 2)
          mem_time_start_hours = '0' + mem_time_start_hours;
        mem_starts_h = String(position_starts + 0, HEX);
        mem_starts_h.toUpperCase();
        //Not that is compleatly in HEX I copy in the data buffer
        cmd_write_data[13] = mem_starts_h.charAt(0);
        cmd_write_data[14] = mem_starts_h.charAt(1);
        cmd_write_data[15] = mem_starts_h.charAt(2);
        cmd_write_data[17] = mem_time_start_hours.charAt(0);
        cmd_write_data[18] = mem_time_start_hours.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data));
        delay(800);
        start_time_min = time_to_pg_format(0, time_prog[index_complet][1]);
        String mem_time_start_min = String(start_time_min, HEX);
        mem_time_start_min.toUpperCase();
        if (mem_time_start_min.length() != 2)
          mem_time_start_min = '0' + mem_time_start_min;
        String mem_starts_m = String(position_starts + 1, HEX);
        mem_starts_m.toUpperCase();
        cmd_write_data[13] = mem_starts_m.charAt(0);
        cmd_write_data[14] = mem_starts_m.charAt(1);
        cmd_write_data[15] = mem_starts_m.charAt(2);
        cmd_write_data[17] = mem_time_start_min.charAt(0);
        cmd_write_data[18] = mem_time_start_min.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data));
        position_starts += 2;
        delay(800);
      }
    }
    JsonArray &valves = parsed["valves"];
    if (valves.success())
    {
      // LOG("Exists Valves");
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
      //TODO: obtein the number of the valves and write the EEPROM memmory in correct positions
      for (int index_compleat = 0; index_compleat < valves.size(); index_compleat++)
      {
        uint16_t val = time_to_pg_format(time_valves[index_compleat][0], time_valves[index_compleat][1]);
        String mem_time = String(val, HEX);
        if (mem_time.length() == 1)
          mem_time = '0' + mem_time;
        mem_time.toUpperCase();
        Serial.println(mem_time);
        // Serial.println(mem_time.charAt(0));
        number_valves = valves[index_compleat]["v"].as<uint8_t>() - 1;
        String mem_starts = String(mem_pos + number_valves,HEX);
        mem_starts.toUpperCase();
        cmd_write_data[13] = mem_starts.charAt(0);
        cmd_write_data[14] = mem_starts.charAt(1);
        cmd_write_data[15] = mem_starts.charAt(2);
        cmd_write_data[17] = mem_time.charAt(0);
        cmd_write_data[18] = mem_time.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data));
        for (i = 0; i < sizeof(cmd_write_data); i++)
        {
          Serial.write(cmd_write_data[i]);
          // Serial.print(" ");
        }
        delay(1000);
        Serial.println(" ");
      }
    }
    uint8_t irrig_percent =  parsed["water"].as<uint8_t>();
    //TODO: write in the PG memmory
    LOGLN(manual_program);

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
void action_prog_pg(uint8_t state, char program)
{
  if (state == 1)
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
uint8_t time_to_pg_format(uint8_t hours, uint8_t minutes)
{
  if (hours == 0) //Less than an hour I just have to convert to hex:
    return minutes;
  else
  {
    //I obtein the units of the number and then decimal
    uint8_t min_units = minutes - (minutes / 10) * 10;
    if (min_units >= 5)
      min_units = 5;
    else
      min_units = 0;
    minutes = (minutes / 10) * 10 + min_units;
    hours--;
    //Now I have the time in a correct format and I can convert it
    uint8_t times_minutes = 0;
    while (minutes > 0)
    {
      minutes -= 5;
      times_minutes++;
    }
    return (60 + 12 * hours + times_minutes);
  }
}
