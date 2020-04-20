#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#include <JamAtm-Vyrsa.h>

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
bool program_active[6];

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);
SPIFlash flash(CS_M);
program prog[TOTAL_PROG];
SoftwareSerial softSerial(PG_RXD, PG_TXD);
RV1805 rtc;
Sleep lowPower;

Jam jam;
sysVar sys;

uint8_t data[RH_RF95_MAX_MESSAGE_LEN]; // Don't put this on the stack:
uint8_t buf[50];
bool rf_flag = false;
uint8_t UUID_1[] = {'A', '1'}; // THE EMITER MUST CHANGE THIS IN EVERY ONE
String pg;
char pgData[PG_MAX_LEN];
uint8_t i, j, rfId, cmd;
volatile uint8_t oldPort = 0x00;
volatile bool intRtc = false;
uint8_t valveDef[MAX_CHILD], progDef[TOTAL_PROG];
bool gprs_on = true, mode = false, comError[MAX_CHILD];

uint32_t millix;
bool ones_time;
bool pressed_times;

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
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  // rtc.setToCompilerTime();
  Serial.println(rtc.stringTime());
  Serial.println(rtc.stringDate());
  jam.ledBlink(LED_SETUP, 1000);
  //Set PG in time:
  change_time_pg(rtc.getWeekday() - 1, rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); //year/month/week/day/hour/min
  print_flash();
  connectSIM();
  connectMqtt();
  delay(500);
  millix = millis();
  char assigned[] = {21, 34, 54, 67};

  //At re-starting the pg is going to read all the info and send it to the web
  Serial.print("Sendding to app---");
  DPRINTLN(freeRam());
  // getAllFromPG(); // Have no idea why i can't do both at the same time
  // json_connect_app(); //for sendding to the web that everithing is ok
  // json_oasis_paring(true, 1, assigned); //For creating more oasis in the web
  // json_oasis_paring(false, 1, assigned); //Asigned all the valves
  // delay(500);
  // json_valve_action(false, 3, 0, 5);
  // delay(500);
  // json_program_action(false, 'B');

  Serial.print("Finish sending---");
  DPRINTLN(freeRam());
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{
  mqttClient.loop();
  listening_pg();

  if (!mqttClient.connected())
  {
    Serial.println("Mqtt connection fail");
    connectMqtt();
    delay(5);
  }
  // Every 20 seconds I publish that I am ALIVE
  if (millis() - millix >= 60000) // Printing that I am not dead
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
    char json[40];
    DynamicJsonBuffer jsonBuffer(32);
    JsonObject &root = jsonBuffer.createObject();
    root.set("voltage", bat);
    root.set("freeRam", String(freeRam()) + "B");
    root.printTo(json);
    mqttClient.publish((String(sys.devUuid) + "/uptime").c_str(), (const uint8_t *)json, strlen(json), false);
    DPRINTLN(freeRam());
    millix = millis();
  }
  if (!digitalRead(PCINT_PIN)) //If pressed the button syn with the web
  {
    if (!pressed_times)
    {
      //getAllFromPG(); // Have no idea why i can't do both at the same time
      pressed_times = true;
    }
    else
    {
      pressed_times = false;
      Serial.println("BUTTON PRESSED");
      delay(500);
      DPRINTLN(freeRam());
      getAllFromPG();
      for (int i = 0; i < 6; i++)
      {
        json_clear_starts(i);
        delay(50);
        json_week_days(i, prog[i].wateringDay);
        delay(50);
        json_program_starts(i);
        delay(50);
      }
      DPRINTLN(freeRam());
    }
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
  if (!length)
    return;
  char json[MAX_JSON_SIZE];
  String sTopic;
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  strcpy(json, (char *)payload); //one alternative is using c fucntion to convert
  JsonObject &parsed = jsonBuffer.parseObject(json);
  DPRINTLN(freeRam());
  //DPRINT("Mqtt received: ");
  //for (int i = 0; i < sizeof(json); i++)
  //  Serial.write(json[i]);
  int identifier = parsed["id"].as<int>(); // This is for the sendding function app
  sTopic = String(topic);
  // Then I identify the topic related with
  if (identifier > 1)
  {
    if (sTopic.indexOf("manvalve") != -1) //done
    {
      send_web("/manvalve", sizeof("/manvalve"), identifier);
      delay(1000);
      //I firts of all parse the message
      JsonArray &valves = parsed["valves"];
      //I obtein the values of the parser info:
      String valve_time;
      int valve_number[25], valve_action[25], valve_min[25], valve_hours[25];
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
          LOG("APAGAR LA: ");
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
    else if (sTopic.indexOf("manprog") != -1) //done
    {
      //I obtein the values of the parser info:
      uint8_t activate = parsed["action"];
      // Serial.println(activate);
      String manual_program;
      manual_program = parsed["prog"].as<String>();
      // I first send it to
      send_web("/manprog", sizeof("/manprog"), identifier); // I send ack to the app
      delay(500);
      action_prog_pg(activate, manual_program.charAt(0)); //I send the command tom PG
    }
    else if (sTopic.indexOf("oasis") != -1) //done
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
        change_oasis_assigned(ide[oasis_num], assigned_id);
      }

      send_web("/oasis", sizeof("/oasis"), identifier);
    }
    else if (sTopic.indexOf("program") != -1) //fail valves time
    {
      //I parse the letter of the program
      send_web("/program", sizeof("/program"), identifier);
      String manual_program = parsed["prog"].as<String>();
      //I determin the positions of the PG in which I have to write
      uint16_t position_starts = 416;     // This is the position of the starts in PG EEPROM memmory
      uint16_t mem_pos = 1024;            // This is the position of the starts in PG EEPROM memmory
      uint16_t position_percentage = 144; // This is the position of the irrig % in PG EEPROM memmory
      uint8_t position_week = 0xB0;       //This is for the week day starts
      switch (manual_program.charAt(0))
      {
      case 'B':
        position_starts = 428;
        mem_pos = 1152;
        position_percentage = 146;
        position_week = 0xB1;
        break;
      case 'C':
        position_starts = 440;
        mem_pos = 1280;
        position_percentage = 148;
        position_week = 0xB2;
        break;
      case 'D':
        position_starts = 452;
        mem_pos = 1408;
        position_percentage = 150;
        position_week = 0xB3;
        break;
      case 'E':
        position_starts = 464;
        mem_pos = 1536;
        position_percentage = 152;
        position_week = 0xB4;
      case 'F':
        position_starts = 476;
        mem_pos = 1664;
        position_percentage = 154;
        position_week = 0xB5;
      default:
        break;
      }
      //I parse the array of starts and valves, if there's any
      JsonArray &starts = parsed["starts"]; //done
      if (starts.success())
      {
        //First parse all the existed starts:
        String str_time; //For writting in the memmory
        uint8_t time_prog[6][2];
        uint16_t start_time_hours = 0, start_time_min = 0;
        String mem_starts_h;
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
          delay(800);
          for (uint8_t times_2 = 0; times_2 < 2; times_2++)
          {
            //First format the info y PG language
            start_time_hours = time_to_pg_format(0, time_prog[index_starts][times_2]);
            String mem_time_start_hours = String(start_time_hours, HEX);
            mem_time_start_hours.toUpperCase();
            if (mem_time_start_hours.length() != 2)
              mem_time_start_hours = '0' + mem_time_start_hours;
            mem_starts_h = String(position_starts + times_2, HEX);
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
          }
          position_starts += 2;
        }
        //Clear all the starts that are not defined and could be saved in pg memmory:
        uint8_t start_to_clear = 6 - starts.size();
        uint16_t position_end = position_starts + 6; // try to write in 0x1AA and then in 0x1A8
        while (start_to_clear > 0)
        {
          for (uint8_t times = 0; times <= 1; times++)
          {
            //First clear from the bottom to the top of the memmory:
            delay(800);
            mem_starts_h = String(position_end + times, HEX);
            mem_starts_h.toUpperCase();
            cmd_write_data[13] = mem_starts_h.charAt(0);
            cmd_write_data[14] = mem_starts_h.charAt(1);
            cmd_write_data[15] = mem_starts_h.charAt(2);
            cmd_write_data[17] = 'F';
            cmd_write_data[18] = 'F';
            calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
            softSerial.write(cmd_write_data, sizeof(cmd_write_data));
            //for (int i = 0; i < sizeof(cmd_write_data); i++)
            //  Serial.write(cmd_write_data[i]);
            Serial.println(" ");
            delay(800);
          }
          start_to_clear--;
          position_end -= 2;
        }
        //WEEK STARTS:
      }
      JsonArray &week_day = parsed["week_day"]; //done
      if (week_day.success())
      {
        String day = "";
        for (uint8_t items = 0; items < week_day.size(); items++)
          day += week_day[items].as<String>();
        char days[day.length() + 1];
        day.toCharArray(days, sizeof(days));
        change_week_pg(days, sizeof(days), position_week);
        delay(800);
      }
      JsonArray &valves = parsed["valves"]; //almost done
      if (valves.success())
      {
        // LOG("Exists Valves");
        uint8_t time_valves[128][2]; //This is a huge and ridiculous
        String irrig_time;           //The string for the 00:00 time format
        uint8_t number_valves;
        uint16_t val = 0;
        uint8_t clear_num = 1;
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
          delay(10);
          //Obtein the number of the valves and write the EEPROM memmory in correct positions
          while (number_valves + 1 > clear_num)
          {
            String mem_starts = String(mem_pos + clear_num - 1, HEX);
            mem_starts.toUpperCase();
            cmd_write_data[13] = mem_starts.charAt(0);
            cmd_write_data[14] = mem_starts.charAt(1);
            cmd_write_data[15] = mem_starts.charAt(2);
            cmd_write_data[17] = '0';
            cmd_write_data[18] = '0';
            calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
            softSerial.write(cmd_write_data, sizeof(cmd_write_data));
            for (i = 0; i < sizeof(cmd_write_data); i++)
              Serial.write(cmd_write_data[i]);
            delay(1000);
            Serial.println(" ");
            clear_num++;
          }
          val = time_to_pg_format(time_valves[number_valves][0], time_valves[number_valves][1]);
          String mem_time = String(val, HEX);
          if (mem_time.length() == 1)
            mem_time = '0' + mem_time;
          mem_time.toUpperCase();
          Serial.println(mem_time);
          String mem_starts = String(mem_pos + number_valves, HEX);
          mem_starts.toUpperCase();
          cmd_write_data[13] = mem_starts.charAt(0);
          cmd_write_data[14] = mem_starts.charAt(1);
          cmd_write_data[15] = mem_starts.charAt(2);
          cmd_write_data[17] = mem_time.charAt(0);
          cmd_write_data[18] = mem_time.charAt(1);
          calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
          softSerial.write(cmd_write_data, sizeof(cmd_write_data));
          for (i = 0; i < sizeof(cmd_write_data); i++)
            Serial.write(cmd_write_data[i]);
          delay(1000);
          Serial.println(" ");
          clear_num++;
        }
      }
      //Write in the PG memmory the irrig %
      uint16_t irrig_percent = parsed["water"].as<uint16_t>();
      if (parsed["water"].success())
        write_percentage_pg(position_percentage, irrig_percent);
      LOGLN(manual_program);
    }
    else if (sTopic.indexOf("query") != -1) //done
    {
      json_query(String(identifier).c_str(), "AUTO");
    }
    else if (sTopic.indexOf("general") != -1) //nothing to do here
    {
      send_web("/general", sizeof("/general"), identifier);
    }
    else if (sTopic.indexOf("stop") != -1) //nothing to do here
    {
      send_web("/stop", sizeof("/stop"), identifier);
    }
  }
  DPRINTLN(freeRam());
}
void send_web(char *topic, unsigned int length, int id)
{
  String ack_topic = String(topic);
  String identifier = String(id);
  char json[40];
  DynamicJsonBuffer jsonBuffer(27);
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
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  for (uint8_t index_program = 0; index_program < sizeof(program_letters); index_program++)
    if (program_letters[index_program] == program)
    {
      DPRINTLN("El valor del programa es: ");
      Serial.write(program);
      if (state == 1)
        program_active[index_program] = true;
      else
        program_active[index_program] = false;
    }

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
void write_percentage_pg(uint16_t position, uint16_t percentage)
{
  //First I obtein the value in PG format
  String str_percen = String(percentage, HEX);
  str_percen.toUpperCase();
  //If is less than 0x0F I add a 0
  if (str_percen.length() != 2)
    str_percen = '0' + str_percen;
  //Then I calculate the position, the first one is 0x90
  String irrig_pos = String(position, HEX);
  if (irrig_pos.length() != 2)
    irrig_pos = '0' + irrig_pos;
  irrig_pos.toUpperCase();
  if (percentage > 255)
  {
    //Then I copy in the buffer to send it
    cmd_write_data[13] = '0';
    cmd_write_data[14] = irrig_pos.charAt(0);
    cmd_write_data[15] = irrig_pos.charAt(1);
    cmd_write_data[17] = '0'; //str_percen.charAt(0);
    cmd_write_data[18] = '1';
    calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
    softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG

    delay(1000);
    //I change the irrig time also
    str_percen = String(percentage - 256, HEX);
    str_percen.toUpperCase();
    //If is less than 0x0F I add a 0
    if (str_percen.length() != 2)
      str_percen = '0' + str_percen;
  }
  else
  {
    cmd_write_data[13] = '0';
    cmd_write_data[14] = irrig_pos.charAt(0);
    cmd_write_data[15] = irrig_pos.charAt(1);
    cmd_write_data[17] = '0'; //str_percen.charAt(0);
    cmd_write_data[18] = '0';
    calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
    softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
    delay(1000);
  }
  for (i = 0; i < sizeof(cmd_write_data); i++)
  {
    Serial.write(cmd_write_data[i]);
  }
  Serial.println(" ");
  irrig_pos = String(position + 1, HEX);
  if (irrig_pos.length() != 2)
    irrig_pos = '0' + irrig_pos;
  irrig_pos.toUpperCase();
  cmd_write_data[13] = '0';
  cmd_write_data[14] = irrig_pos.charAt(0);
  cmd_write_data[15] = irrig_pos.charAt(1);
  cmd_write_data[17] = str_percen.charAt(0);
  cmd_write_data[18] = str_percen.charAt(1);
  calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
  softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
  for (i = 0; i < sizeof(cmd_write_data); i++)
    Serial.write(cmd_write_data[i]);
  delay(800);
  Serial.println(" ");
}
void getAllFromPG() //this function get all data from PG
{
  bool defined;
  uint8_t i, m, n, v;
  int index, addrInt;
  String aux, addr = "200";
  uint8_t bookEnd = ceil((double)(MAX_CHILD * 4) / 64);

  pgCommand(cmd_write_flag, sizeof(cmd_write_flag)); //clear memmory flag
  //getPGTime(true);                                   //read date and time
  memmoryHandler(3, false); //read arranque
  for (i = 0; i < TOTAL_PROG; i++)
  {                 //for each program
    progDef[i] = 0; //init flag as prog not defined
    for (j = 0; j < TOTAL_START; j++)
    { //for each arranque
      if ((prog[i].start[j][0] != 0xff) && (prog[i].start[j][1] != 0xff))
      {                 //if arranque at least one arranque is defined
        progDef[i] = 1; //set defined flag as true
        break;          //leave loop
      }
    }
    if (progDef[i])
    {                               //if al least one arranque is defined
      memmoryHandler(i + 8, false); //read tiempo de riego
      progDef[i] = 0;               //initialize defined flag to false
      for (v = 0; v < TOTAL_VALVE; v++)
      { //for each valve
        if (prog[i].irrigTime[v] != 0xff)
        {                 //if at least one tiempo de riego is defined
          progDef[i] = 1; //set defined flag as true
          break;          //leave loop
        }
      }
    }
    if (progDef[i])             //if at least one arranque and one tiempo de riego is defined
      memmoryHandler(1, false); //read aporte de agua, interval, starday and wateringday
  }

  for (i = 0, m = 0, valveDef[m] = 0; i < bookEnd; i++)
  {
    cmd_read_book[ADDR_INDEX] = addr.charAt(0);
    cmd_read_book[ADDR_INDEX + 1] = addr.charAt(1);
    cmd_read_book[ADDR_INDEX + 2] = addr.charAt(2);
    pgCommand(cmd_read_book, sizeof(cmd_read_book));
    pg = String(pgData);
    index = pg.indexOf("#") + 1;
    pg = pg.substring(index, index + BOOK_LEN);
    DPRINT("Assignment: ");
    DPRINTLN(pg.c_str());
    for (n = 0, index = 0; index < pg.length();)
    {
      aux = pg.substring(index, pg.indexOf(" ", index));
      if (aux != "FF")
      {
        sys.childValves[m][n] = strtol(aux.c_str(), NULL, HEX) + 1;
        valveDef[m] = 1;
      }
      else
        sys.childValves[m][n] = 0;
      index += aux.length() + 1;
      if (n == 3)
      {
        m++;
        n = 0;
        valveDef[m] = 0;
      }
      else
        n++;
    }
    addrInt = strtol(addr.c_str(), NULL, HEX);
    addrInt += 64;
    addr = String(addrInt, HEX);
    addr.toUpperCase();
  }
  digitalWrite(CS_RF, HIGH);
  flash.eraseSector(SYS_VAR_ADDR);
  flash.writeByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  digitalWrite(CS_RF, LOW);
}
void memmoryHandler(uint8_t pos, bool sendChange) //this function read memmory in a given range of address - It is not changed
{

  int index;
  String str, aux;
  uint8_t p, a, v, m, ptr;
  i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;

  if (pos == 1)
  {                                  //if bit 1 is 1
    cmd_read_book[ADDR_INDEX] = '0'; //set address to read
    cmd_read_book[ADDR_INDEX + 1] = '9';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_book, sizeof(cmd_read_book));    //send read page command to pg
    pg = String(pgData);                                //convert it into string
    index = pg.indexOf("#") + 1;                        //point index to first byte read
    str = pg.substring(index, index + APORTE_AGUA_LEN); //get aporte de agua bytes
    DPRINTLN(F("***** Aporte de agua *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1, i++)
    {                                                  //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr); //get first byte of aporte de agua
      data[i] = strtol(aux.c_str(), NULL, HEX);        //convert it and save into data
      prog[p].waterPercent = data[i] * 256;            //convert value to 2 byte integer
      ptr += aux.length() + 1;                         //point index to next byte
      aux = str.substring(str.indexOf(" ", ptr), ptr); //get second byte of aporte de agua
      data[++i] = strtol(aux.c_str(), NULL, HEX);      //convert it and save into data
      prog[p].waterPercent += data[i];                 //convert it in 2 byte integer and save it

      DPRINT(F("Prog "));
      DPRINT((char)(p + 'A'));
      DPRINTLN(": " + String(prog[p].waterPercent));
    }
    index += str.length() + 1;                          //point index to the next range of bytes
    str = pg.substring(index, index + INTERV_INIT_LEN); //get interval and start day bytes
    DPRINTLN(F("***** Intervalo y Start day *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1)
    {                                                                //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr);               //get interval byte
      data[i++] = prog[p].interval = strtol(aux.c_str(), NULL, HEX); //convert into integer and save it
      ptr += aux.length() + 1;                                       //point index to next byte
      aux = str.substring(str.indexOf(" ", ptr), ptr);               //get start day byte
      data[i++] = prog[p].startDay = strtol(aux.c_str(), NULL, HEX); //convert into integer and save it
      DPRINT(F("*Prog "));
      DPRINTLN((char)(p + 'A'));
      DPRINT(F("  interval: "));
      DPRINTLN(prog[p].interval);
      DPRINT(F("  startDay: "));
      DPRINTLN(prog[p].startDay);
    }
    index += str.length() + 1;                       //point index to next range of byte
    str = pg.substring(index, index + WATERING_LEN); //get watering days
    DPRINTLN(F("***** watering day *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1)
    {                                                                   //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr);                  //get watering day byte
      data[i++] = prog[p].wateringDay = strtol(aux.c_str(), NULL, HEX); //convert into integer and save it
      DPRINT(F("Prog "));
      DPRINT((char)(p + 'A'));
      DPRINTLN(": " + String(prog[p].wateringDay));
    }
  }
  else if (pos == 3)
  {                                  //if bit 3 is 1
    cmd_read_book[ADDR_INDEX] = '1'; //set address to read
    cmd_read_book[ADDR_INDEX + 1] = 'A';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_book, sizeof(cmd_read_book)); //read book
    pg = String(pgData);                             //convert it into string
    index = pg.indexOf("#") + 1;                     //point index to first byte read
    pg = pg.substring(index, index + BOOK_LEN);      //get all book bytes
    cmd_read_line[ADDR_INDEX] = '1';                 //set adrees to read remaining bytes
    cmd_read_line[ADDR_INDEX + 1] = 'E';
    cmd_read_line[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_line, sizeof(cmd_read_line)); //read line
    aux = String(pgData);                            //convert it into string
    index = aux.indexOf("#") + 1;                    //point index to first byte read
    aux = aux.substring(index, index + 23);          //get 8 byte from bytes read
    pg = pg + " " + aux;                             //add those 8 byte to pg string
    index = 0;
    DPRINTLN(F("***** Arranques *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, index += str.length() + 1, ptr = 0)
    {                                                  //for each program
      str = pg.substring(index, index + ARRANQUE_LEN); //get string with arranque of its program
      for (a = 0; a < TOTAL_START; a++, ptr += aux.length() + 1)
      {                                                                   //for each arraque
        aux = str.substring(str.indexOf(" ", ptr), ptr);                  //get hour
        data[i++] = prog[p].start[a][0] = strtol(aux.c_str(), NULL, HEX); //save it
        ptr += aux.length() + 1;                                          //point index to next value
        aux = str.substring(str.indexOf(" ", ptr), ptr);                  //get min
        data[i++] = prog[p].start[a][1] = strtol(aux.c_str(), NULL, HEX); //save it
      }
      DPRINT(F("Prog "));
      DPRINT((char)(p + 'A'));
      DPRINTLN(": " + String(prog[p].start[0][0]) + ":" + String(prog[p].start[0][1]) +
               ", " + String(prog[p].start[1][0]) + ":" + String(prog[p].start[1][1]) +
               ", " + String(prog[p].start[2][0]) + ":" + String(prog[p].start[2][1]) +
               ", " + String(prog[p].start[3][0]) + ":" + String(prog[p].start[3][1]) +
               ", " + String(prog[p].start[4][0]) + ":" + String(prog[p].start[4][1]) +
               ", " + String(prog[p].start[5][0]) + ":" + String(prog[p].start[5][1]));
    }
  }
  else if (pos > 7)
  {                                              //if one of the bit of second byte is 1
    p = pos - 8;                                 //define program
    cmd_read_book[ADDR_INDEX] = (pos / 2) + '0'; //set address to read
    (pos % 2) ? cmd_read_book[ADDR_INDEX + 1] = '8' : cmd_read_book[ADDR_INDEX + 1] = '0';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    DPRINTLN(F("***** Tiempos de riego *****"));
    DPRINT(F("Prog "));
    DPRINT((char)(p + 'A'));
    DPRINT(F(": "));
    for (v = 0, a = 0; a < 2; a++)
    {                                                  //read 64 bytes of irrigation time two times, 128
      pgCommand(cmd_read_book, sizeof(cmd_read_book)); //read first 64 valve irrigtation time
      pg = String(pgData);                             //convert it into string
      index = pg.indexOf("#") + 1;                     //point index to firt byte read
      str = pg.substring(index, index + IRRIG_TIME_LEN);
      for (m = 0, ptr = 0; m < 64; v++, m++, ptr += aux.length() + 1)
      {                                                                    //for each valve
        aux = str.substring(str.indexOf(" ", ptr), ptr);                   //get value
        data[i++] = prog[p].irrigTime[v] = strtol(aux.c_str(), NULL, HEX); //convert it to integer and save it
        DPRINT("v" + String(v + 1) + String(":"));
        DPRINT(prog[p].irrigTime[v]);
        DPRINT(" ");
      }
      if (cmd_read_book[ADDR_INDEX + 1] == '0') //set address to the next 64 bytes
        cmd_read_book[ADDR_INDEX + 1] = '4';
      else
        cmd_read_book[ADDR_INDEX + 1] = 'C';
    }
    DPRINTLN();
  }
  digitalWrite(CS_RF, HIGH);        //unselect rf
  flash.eraseSector(PROG_VAR_ADDR); //erase program variable sector
  flash.writeByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  ;                         //write program variable changes
  digitalWrite(CS_RF, LOW); //select again rf
  if (sendChange)
  {
    data[CMD_INDEX + 1] = pos; //set command parameter
    data[CMD_INDEX] = MEMMORY; //set comand id
    for (j = 0; j < sys.nChild; j++)
    { //for each child
      DPRINT("Send change to OASIS: ");
      DPRINTLN(j + 1);
      jam.fillWithString(data, String(sys.oasisUuid[j]), PAYLOAD_INDEX); //add child uuid to data
      //comError[j] = !sendCommand(data, i, sys.oasisRfId[j]);             //send command to child
    }
    // checkComError(i);
  }
}
void json_program_starts(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(128);
  JsonObject &root = jsonBuffer.createObject();
  uint8_t index_program = program; //I generate the json only for the program request
  root["prog"] = String(program_letters[index_program]);
  JsonArray &starts = root.createNestedArray("starts");
  //Generate all the starts
  for (uint8_t index_time = 0; index_time < 6; index_time++)
    if (prog[index_program].start[index_time][0] != 255)
    {
      String minutes, hours;
      if (prog[index_program].start[index_time][0] < 10)
        hours = '0' + String(prog[index_program].start[index_time][0]);
      else
        hours = String(prog[index_program].start[index_time][0]);
      if (prog[index_program].start[index_time][1] < 10)
        minutes = '0' + String(prog[index_program].start[index_time][1]);
      else
        minutes = String(prog[index_program].start[index_time][1]);
      starts.add(hours + ":" + minutes);
    }
  int water = (int)(prog[index_program].waterPercent);
  if (water < 0)
    water = 100;
  root["water"] = water;
  char json[150];
  root.printTo(json);
  mqttClient.publish((String(sys.devUuid) + "/program").c_str(), (const uint8_t *)json, strlen(json), false);
  DPRINT("Json_program_starts: ");
  DPRINTLN(freeRam());

  // root.prettyPrintTo(Serial);
}
void json_program_valves(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(500);
  JsonObject &root = jsonBuffer.createObject();
  uint8_t index_program = program; //I generate the json only for the program request
  root["prog"] = String(program_letters[index_program]);
  uint16_t time_compleat, time_in_format_h, time_in_format_m;
  bool done = false;
  for (uint8_t index_valves = 0; index_valves < 8; index_valves++)
    if (String(prog[index_program].irrigTime[index_valves]) != "255" && String(prog[index_program].irrigTime[index_valves]) != "247")
    {
      JsonArray &valves = root.createNestedArray("valves");
      JsonObject &irrig = root.createNestedObject("");
      irrig["v"] = index_valves + 1;
      time_compleat = pg_reag_to_web(prog[index_program].irrigTime[index_valves]);
      time_in_format_h = time_compleat / 60;
      time_in_format_m = time_compleat % 60;
      String time_h_json, time_m_json;
      if (time_in_format_h < 10)
        time_h_json = '0' + String(time_in_format_h);
      else
        time_h_json = String(time_in_format_h);
      if (time_in_format_m < 10)
        time_m_json = '0' + String(time_in_format_m);
      else
        time_m_json = String(time_in_format_m);

      irrig["time"] = time_h_json + ":" + time_m_json;
      valves.add(irrig);
    }
  char json[500];
  root.printTo(json);
  DPRINTLN(freeRam());
  mqttClient.publish((String(sys.devUuid) + "/program").c_str(), (const uint8_t *)json, strlen(json), false);
}
void json_clear_starts(uint8_t program)
{
  //I clear the starts info
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  DynamicJsonBuffer jsonBuffer(43);
  JsonObject &root = jsonBuffer.createObject();
  root["prog"] = String(program_letters[program]);
  JsonArray &starts = root.createNestedArray("starts");
  char json[43];
  root.printTo(json);
  mqttClient.publish((String(sys.devUuid) + "/program").c_str(), (const uint8_t *)json, strlen(json), false);
}
uint16_t pg_reag_to_web(uint16_t pg_time) //It return the time ok in minutes to the web
{
  uint8_t combinations[12][2] = {{72, 1}, {84, 2}, {96, 3}, {108, 4}, {120, 5}, {132, 6}, {144, 7}, {156, 8}, {168, 9}, {180, 10}, {192, 11}, {204, 12}};
  if (pg_time < 60)
    return pg_time;
  //First obtein the hours
  uint8_t index = 0;
  while (index++ < 12)
    if (pg_time < combinations[index][0])
      break;
  //Obtein the hours and minutes and print them
  uint16_t hours = (uint16_t)combinations[index][1];
  uint16_t min = (pg_time - 60 - 12 * (hours - 1)) * 5;
  printf("La hora real es: %i:%i \n", hours, min);
  return (hours * 60 + min);
}
void json_connect_app()
{
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(53);
  JsonObject &root = jsonBuffer.createObject();
  root["uuid"] = String(sys.devUuid);
  root["model"] = "6011";
  char json[60];
  root.printTo(json);
  mqttClient.publish((String(sys.devUuid) + "/connect").c_str(), (const uint8_t *)json, strlen(json), false);
}
void json_query(const char id[], char status[])
{
  uint16_t b;
  DynamicJsonBuffer jsonBuffer(500);
  JsonObject &root = jsonBuffer.createObject();
  analogReference(INTERNAL);
  for (uint8_t i = 0; i < 3; i++)
  {
    b = analogRead(PA0);
    delay(1);
  }
  float bat = (b * 0.0190 - 2.2); //This value is not perfect "-2,2" added
                                  //REAL ; MEASSURE
                                  //6,6 ; 8
                                  //7,4 ; 9
                                  //8,1 ; 9,9
                                  //9,1 ; 11,1
                                  //10 ; 12,1
                                  //11,3 ; 13.8
                                  //12 ; 14.6

  int bat_percentage = map((int)round(bat), 6, 15, 5, 100);
  if (bat_percentage > 100)
    bat_percentage = 100;
  root["id"] = String(id);
  root["status"] = String(status);
  JsonObject &battery = root.createNestedObject("battery");
  battery["com"] = bat_percentage;
  battery["prog"] = bat_percentage;
  JsonObject &oasis = battery.createNestedObject("oasis");
  oasis["id"] = 1;
  oasis["bat"] = 78;
  root["connection"] = map(modem.getSignalQuality(), 15, 30, 5, 100);
  //I send the time to the web:
  root["date"] = String(rtc.stringDate()) + " " + String(rtc.stringTime());
  //I check if there is any prpogram active and I send to the web:
  JsonArray &active = root.createNestedArray("prog_active");
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  for (uint8_t prog = 0; prog < 6; prog++)
    if (program_active[prog])
      active.add(String(program_letters[prog]));
  //I generate the main json frame
  char json[300];
  root.printTo(json);
  mqttClient.publish((String(sys.devUuid) + "/query").c_str(), (const uint8_t *)json, strlen(json), false);
  // root.prettyPrintTo(Serial);
}
void change_week_pg(char *date, uint8_t lenght, uint8_t program)
{
  uint8_t value_to_memory = 0;
  for (uint8_t index = 0; index < lenght; index++)
    switch (*(date + index))
    {
    case '1':
      value_to_memory += 1;
      break;
    case '2':
      value_to_memory += 2;
      break;
    case '3':
      value_to_memory += 4;
      break;
    case '4':
      value_to_memory += 8;
      break;
    case '5':
      value_to_memory += 16;
      break;
    case '6':
      value_to_memory += 32;
      break;
    case '7':
      value_to_memory += 64;
      break;
    default:
      break;
    }

  String str_pos_week = String(program, HEX); //Obtein the position of the program memmory
  str_pos_week.toUpperCase();
  String str_days = String(value_to_memory, HEX); //The value to write in PG
  str_days.toUpperCase();
  //If is less than 0x0F I add a 0
  if (str_days.length() != 2)
    str_days = '0' + str_days;
  cmd_write_data[13] = '0';
  cmd_write_data[14] = str_pos_week.charAt(0);
  cmd_write_data[15] = str_pos_week.charAt(1);
  cmd_write_data[17] = str_days.charAt(0);
  cmd_write_data[18] = str_days.charAt(1);
  calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
  softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
                                                            //for (i = 0; i < sizeof(cmd_write_data); i++)
                                                            //  Serial.write(cmd_write_data[i]);
}
void json_week_days(uint8_t program, uint8_t week_day)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  String str_week_day;
  uint8_t compare_operation = 1;
  for (int i = 1; i <= 7; i++)
  {
    if (week_day & compare_operation)
    {
      str_week_day += String(i);
      str_week_day += ",";
    }
    compare_operation *= 2;
  }
  str_week_day.remove(str_week_day.length() - 1, 1);
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();
  root["prog"] = String(program_letters[program]);
  root["week_day"] = "[" + str_week_day + "]";
  char json[200];
  root.printTo(json);
  mqttClient.publish((String(sys.devUuid) + "/program").c_str(), (const uint8_t *)json, strlen(json), false);
  // root.prettyPrintTo(Serial);
}
void change_time_pg(uint8_t week, uint8_t hours, uint8_t minutes, uint8_t seconds) //, uint8_t *day, uint8_t *hours, uint8_t *minutes)
{
  //First set the week of the day
  cmd_set_time[19] = week + '0';
  //Second set the hour
  String time = String(hours);
  if (time.length() == 1)
    time = '0' + time;
  cmd_set_time[11] = time.charAt(0);
  cmd_set_time[12] = time.charAt(1);
  //Third set the minutes
  time = String(minutes);
  if (time.length() == 1)
    time = '0' + time;
  cmd_set_time[13] = time.charAt(0);
  cmd_set_time[14] = time.charAt(1);
  //Fourth set the seconds
  time = String(seconds);
  if (time.length() == 1)
    time = '0' + time;
  cmd_set_time[15] = time.charAt(0);
  cmd_set_time[16] = time.charAt(1);
  calcrc((char *)cmd_set_time, sizeof(cmd_set_time) - 2);
  softSerial.write(cmd_set_time, sizeof(cmd_set_time)); //real send to PG
}
void json_oasis_paring(bool pairing, uint8_t id_uuid, char *assigned)
{
  //First to the pairing topic with the uuid and the to oasis
  //The uuid and the id are the same
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();
  JsonObject &oasis = root.createNestedObject("oasis");
  JsonObject &info = oasis.createNestedObject("");
  info["id"] = id_uuid;
  if (pairing)
    info["uuid"] = id_uuid;
  JsonArray &data = info.createNestedArray("assign");
  uint8_t i = 0;
  while (i < 4)
    data.add(*(assigned + i++));
  char json[200];
  root.printTo(json);
  if (pairing)
    mqttClient.publish((String(sys.devUuid) + "/pairing").c_str(), (const uint8_t *)json, strlen(json), false);
  else
    mqttClient.publish((String(sys.devUuid) + "/oasis").c_str(), (const uint8_t *)json, strlen(json), false);
  // root.prettyPrintTo(Serial);
}
void json_valve_action(bool open, uint8_t valve, uint8_t hours, uint8_t minutes)
{
  DynamicJsonBuffer jsonBuffer(100);
  JsonObject &root = jsonBuffer.createObject();
  JsonObject &oasis = root.createNestedObject("valves");
  JsonObject &info = oasis.createNestedObject("");
  info["v"] = valve;
  if (open)
  {
    info["action"] = 1;
    String str_hours = String(hours);
    if (str_hours.length() == 1)
      str_hours = '0' + str_hours;
    String str_minutes = String(minutes);
    if (str_minutes.length() == 1)
      str_minutes = '0' + str_minutes;
    info["time"] = str_hours + ":" + str_minutes;
  }
  else
    info["action"] = 0;
  char json[200];
  root.printTo(json);
  // root.prettyPrintTo(Serial);
  mqttClient.publish((String(sys.devUuid) + "/manvalve").c_str(), (const uint8_t *)json, strlen(json), false);
}
void json_program_action(bool open, char program)
{
  //Publish in /manprog topic and send to the app whats happening
  DynamicJsonBuffer jsonBuffer(100);
  JsonObject &root = jsonBuffer.createObject();
  root["prog"] = String(program);
  if (open)
    root["action"] = 1;
  else
    root["action"] = 0;
  char json[200];
  root.printTo(json);
  // root.prettyPrintTo(Serial);
  mqttClient.publish((String(sys.devUuid) + "/manprog").c_str(), (const uint8_t *)json, strlen(json), false);
}
void change_oasis_assigned(uint8_t oasis_number, uint8_t *assigned)
{
  //First prepare the oasis to be write in pg memmory
  oasis_number--;
  String str_pos, str_assigned;
  for (uint8_t index_outputs = 0; index_outputs < 4; index_outputs++)
  {
    str_pos = String(0x200 + 4 * oasis_number + index_outputs, HEX);
    str_pos.toUpperCase();
    str_assigned = String(*(assigned + index_outputs) - 1, HEX);
    if (str_assigned.length() != 2)
      str_assigned = '0' + str_assigned;
    str_assigned.toUpperCase();
    cmd_write_data[13] = str_pos.charAt(0);
    cmd_write_data[14] = str_pos.charAt(1);
    cmd_write_data[15] = str_pos.charAt(2);
    cmd_write_data[17] = str_assigned.charAt(0);
    cmd_write_data[18] = str_assigned.charAt(1);
    calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
    softSerial.write(cmd_write_data, sizeof(cmd_write_data));
    //for (i = 0; i < sizeof(cmd_write_data); i++)
    //  Serial.write(cmd_write_data[i]);
    delay(1000);
    Serial.println(" ");
  }
}
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void listening_pg()
{
  String pg;
  char pgData[PG_MAX_LEN];
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
  {
    //if int pg was triggered
    intPg = false;       //clear int pg flag
    pgData[i] = '\0';    //add end of char
    pg = String(pgData); //convert message received into string
    DPRINT("PG: ");
    DPRINTLN(pg);
    if (pg.indexOf("MANVALV START") > 0)
    {
      DPRINTLN("ES EL COMANDO DE ABRIR VALVULA MANUAL");
      char valve_num_aux[2];
      valve_num_aux[0] = pgData[pg.indexOf("MANVALV START") + 14];
      valve_num_aux[1] = pgData[pg.indexOf("MANVALV START") + 15];
      uint8_t valve_number = hex2int(valve_num_aux[0]) * 16 + hex2int(valve_num_aux[1]);
      uint8_t valve_time_hour = hex2int(pgData[pg.indexOf("MANVALV START") + 14 + 3]) * 10 + hex2int(pgData[pg.indexOf("MANVALV START") + 15 + 3]);
      uint8_t valve_time_min = hex2int(pgData[pg.indexOf("MANVALV START") + 14 + 3 + 2]) * 10 + hex2int(pgData[pg.indexOf("MANVALV START") + 15 + 3 + 2]);
      DPRINT(valve_number);
      DPRINT(" valvula - ");
      DPRINT(valve_time_hour);
      DPRINT(" horas - ");
      DPRINT(valve_time_min);
      DPRINTLN(" minutos. ");
      bool open = true;
      if (valve_time_min == 0 && valve_time_hour == 0)
        open = false;
      json_valve_action(open, valve_number, valve_time_hour, valve_time_min);
    }
    if (pg.indexOf("MANVALV STOP") > 0)
    {
      DPRINTLN("ES EL COMANDO DE ABRIR VALVULA MANUAL");
      char valve_num_aux[2];
      valve_num_aux[0] = pgData[pg.indexOf("MANVALV STOP") + 13];
      valve_num_aux[1] = pgData[pg.indexOf("MANVALV STOP") + 14];
      uint8_t valve_number = hex2int(valve_num_aux[0]) * 16 + hex2int(valve_num_aux[1]);
      json_valve_action(false, valve_number, 0, 0);
    }
    else if (pg.indexOf("MANPRG START#") > 0)
    {
      json_program_action(true, (pgData[pg.indexOf("MANPRG START#") + 13]));
    }
    else if (pg.indexOf("MANPRG STOP#") > 0)
    {
      json_program_action(false, (pgData[pg.indexOf("MANPRG STOP#") + 12]));
    }
    else if (pg.indexOf("STOP ALL") > 0)
    {
      DPRINTLN("Paro TODO"); // I clear all the variables of the programs
    }
    else if (pg.indexOf("SET TIME#") > 0)
    {
      uint8_t time_hours = (pgData[pg.indexOf("SET TIME#") + 9] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 10] - '0');
      uint8_t time_min = (pgData[pg.indexOf("SET TIME#") + 9 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 10 + 2] - '0');
      uint8_t time_day_week = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6] - '0');
      uint8_t time_year = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3] - '0');
      uint8_t time_month = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3 + 2] - '0');
      uint8_t time_day = (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 2 + 2 + 2] - '0') * 10 + (pgData[pg.indexOf("SET TIME#") + 9 + 2 + 6 + 3 + 2 + 2] - '0');
      DPRINT(" Hora: ");
      DPRINT(time_hours);
      DPRINT(":");
      DPRINT(time_min);
      DPRINT(" Fecha: ");
      DPRINT(time_day_week);
      DPRINT(" Hora: ");
      DPRINT(time_year);
      DPRINT("/");
      DPRINT(time_month);
      DPRINT("/");
      DPRINT(time_day);
      rtc.updateTime();
      DPRINTLN("");
      rtc.setTime(0, 0, time_min, time_hours, time_day, time_month, time_year, time_day_week);
      DPRINTLN(rtc.stringTime());
      DPRINTLN(rtc.stringDate());

      //send_nodo(1, sys_rf.UUID_RF, REQUEST_TIME, 0, 0, 0, asignacion);
    }
    else if (pg.indexOf("PAIRING#") > 0)
    {
      //I always obtein the number of oasis without one unit due to format 8bit vs 16 bits
      String valve_number = getValue(pg, '#', 1);
      int valve_number_true = (int)strtol(&valve_number[0], NULL, 16);
      // DPRINTLN(valve_number_true);
      String valve_assigned;
      int oasis_valves[4];
      for (int k = 0; k < 4; k++)
      {
        valve_assigned = getValue(getValue(pg, '#', 2), ' ', k);
        oasis_valves[k] = (int)strtol(&valve_assigned[0], NULL, 16);
        DPRINTLN(oasis_valves[k]);
      }

      //send_nodo(1, sys_rf.UUID_RF, REQUEST_ASSIGNED_VALVES, valve_number_true + 1, 0, 0, temp_valve);
    }
    else if (pg.indexOf("SELECTOR#06") > 0)
    {
      DPRINTLN("STOP");
    }
    else if (pg.indexOf("MEMMORY#") > 0)
    {
      digitalWrite(LED_SETUP, HIGH);
      DPRINTLN("PG TOUCH");
      getAllFromPG();
      for (int i = 0; i < 6; i++)
      {
        json_clear_starts(i);
        delay(50);
        json_week_days(i, prog[i].wateringDay);
        delay(50);
        json_program_starts(i);
        delay(50);
      }
      digitalWrite(LED_SETUP, LOW);
    }
  }
}
uint8_t hex2int(char ch) // For converting the manual valve action
{
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  return -1;
}
String getValue(String data, char separator, int index)
{

  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++)
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}