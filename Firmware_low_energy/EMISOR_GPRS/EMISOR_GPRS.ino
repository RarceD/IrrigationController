#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#include <JamAtm-Vyrsa.h>

// #define DEBUG_ON
#ifdef DEBUG_ON
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#define LOG(...)
#define LOGLN(...)
#endif

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

typedef struct
{
  bool manual_web;
  bool active[10];
  uint8_t number_timers;
  uint8_t valve_number[10];
  uint32_t millix[10];
  uint32_t times[10];
} stopManualWeb;

typedef struct
{
  uint8_t id;
  char devUuid[UUID_LEN];
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
typedef struct //If I recevived a stop command in the web I have to know what to stop
{
  bool valves[14];
  bool programs[6];
} active_to_stop;

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);
SoftwareSerial softSerial(PG_RXD, PG_TXD);
SPIFlash flash(CS_M);
// RH_RF95 driver(CS_RF, INT_RF);
// RHReliableDatagram manager(driver, SERVER_ADDRESS);
program prog[TOTAL_PROG];
RV1805 rtc;
sysVar sys;
active_to_stop active;
stopManualWeb stop_man_web; //MAX 10 valves open at the same time

uint8_t data[RH_RF95_MAX_MESSAGE_LEN]; // Don't put this on the stack:
uint8_t buf[50];
bool rf_flag = false;
uint8_t UUID_1[] = {'A', '6'}; // THE EMITER MUST CHANGE THIS IN EVERY ONE
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
bool read_pg_web = false;
bool send_stop_program = false;

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
  softSerial.begin(9600);
  flash.powerUp();
  flash.begin();
  /*
  char first_mem[] = "VYR_OASIS_A6";
  for (uint8_t aux = 0; aux < sizeof(first_mem); aux++)
    sys.devUuid[aux] = first_mem[aux];
  flash.eraseSector(SYS_VAR_ADDR);
  flash.writeAnything(SYS_VAR_ADDR, sys);
  */
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  //manager.init();
  //manager.setRetries(8);
  //manager.setTimeout(250);
  //driver.setTxPower(20, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  // rtc.setToCompilerTime();
  DPRINT(rtc.stringTime());
  DPRINT(" ");
  DPRINT(rtc.stringDate());
  DPRINT(" ");
  DPRINTLN(rtc.getWeekday()); //This value is 0 for sunday
  uint16_t d = rtc.getDate();
  uint16_t m = rtc.getMonth();
  uint16_t y = 2000 + rtc.getYear();
  uint16_t weekday = (d += m < 3 ? y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) % 7;
  //Set PG in time:
  if (weekday == 0)
    weekday = 7;
  change_time_pg(weekday, rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); //year/month/week/day/hour/min
  print_flash();
  connectSIM();
  connectMqtt();
  millix = millis();
  // At re-starting the pg is going to read all the info and send it to the web
  // getAllFromPG(); // Have no idea why i can't do both at the same time
  // char assigned[] = {21, 34, 54, 67};
  // json_oasis_paring(true, 1, assigned); //For creating more oasis in the web
  // json_oasis_paring(false, 1, assigned); //Asigned all the valves
  // json_valve_action(false, 3, 0, 5);
  stop_man_web.manual_web = false;
  stop_man_web.number_timers = 0;
  for (uint8_t t = 0; t < 10; t++)
    stop_man_web.active[t] = false;
  // json_connect_app(); //for sendding to the web that everything is ok
  getAllFromPG();
  for (int i = 0; i < 6; i++)
  {
    json_clear_starts(i);
    delay(50);
    json_week_days(i, prog[i].wateringDay);
    delay(50);
    json_program_starts(i);
    delay(50);
    json_program_valves(i);
    delay(50);
  }
  /*
  */
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{
  mqttClient.loop();
  listening_pg();
  if (!mqttClient.connected())
  {
    DPRINTLN("Mqtt connection fail");
    connectMqtt();
    delay(5);
  }
  // Every 30 seconds I publish that I am ALIVE
  if (millis() - millix >= 50000) // Printing that I am not dead
  {
    //First I check if its the irrig AUTO time
    check_time();
    //Second I check if there is any manual open valve ON in order to close
    if (stop_man_web.manual_web)
    {
      for (uint8_t t = 0; t < 10; t++)
        if (stop_man_web.active[t])
        {
          DPRINTLN("CHECK");
          if (millis() - stop_man_web.millix[t] >= stop_man_web.times[t] - 10000)
          {
            DPRINTLN("I close the manual valve with my timer");
            DPRINTLN(stop_man_web.valve_number[t]);
            action_valve_pg(0, stop_man_web.valve_number[t], 0, 0);
            //Save the valve number:
            stop_man_web.valve_number[t] = 0;
            //Th current time on timer 0
            stop_man_web.millix[t] = 0;
            //The time in ms to stop the valve
            stop_man_web.times[t] = 0;
            //The flag of the valve in the struct:
            stop_man_web.active[t] = false;
            if (stop_man_web.number_timers > 0)
              stop_man_web.number_timers--;
            else
              stop_man_web.manual_web = false;
          }
        }
    }
    //I send I'm alive to the web
    uint16_t b;
    analogReference(INTERNAL);
    for (int i = 0; i < 3; i++)
    {
      b = analogRead(PA0);
      delay(1);
    }
    float bat = b * 0.0190 - 2.0 - 2.9;
    char json[40];
    DynamicJsonBuffer jsonBuffer(32);
    JsonObject &root = jsonBuffer.createObject();
    root.set("voltage", String(bat) + "V");
    root.set("freeRam", String(float(freeRam() / 1024.0)) + "kB");
    root.printTo(json);
    mqttClient.publish((String(sys.devUuid) + "/uptime").c_str(), (const uint8_t *)json, strlen(json), false);
    DPRINTLN(freeRam());
    millix = millis();
  }
  if (read_pg_web)
  {
    read_pg_web = false;
    getAllFromPG();
  }
  if (send_stop_program)
  {
    DPRINTLN("IN");
    send_stop_program = false;
    int index_close;
    char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
    for (index_close = 0; index_close < 6; index_close++)
    {
      json_program_action(false, program_letters[index_close]);
      delay(80);
    }
  }
  /*
  if (!digitalRead(PCINT_PIN)) //If pressed the button syn with the web
    DPRINTLN("BUTTON PRESSED");
  */
}
/*******************************************************************   functions     ************************************************************************************/
void connectSIM()
{
  delay(500);
  DPRINTLN("Oasis-Com starts");
  pinMode(SIM_PWR, OUTPUT);
  digitalWrite(SIM_PWR, LOW);
  delay(1200);
  DPRINTLN("Initializing modem...");
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

  DPRINT("Modem: ");
  DPRINTLN(modemInfo);
  Serial1.print("AT+IPR=115200");
  modem.init();
  Serial1.begin(115200);
  delay(2000);
  Serial1.print("AT+IPR=115200");
  modem.simUnlock("8724");
  //DPRINT("AT+CSCLK=2");

  DPRINT("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    DPRINTLN(" fail");
    while (true)
      ;
  }
  DPRINTLN(" succeed");

  DPRINT("Connecting to ");
  DPRINT(apn);
  if (!modem.gprsConnect(apn, user_apn, pass_apn))
  {
    DPRINTLN(" fail");
    while (true)
      ;
  }
  DPRINTLN(" succeed");
  int signalq = modem.getSignalQuality();
  DPRINTLN("Signal quality: " + String(signalq));
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
      DPRINTLN("Trying to connect to MQTT...");
      delay(1000);
    }
  }
  DPRINTLN("Successfully connected to MQTT");
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
      uint8_t valve_number[25], valve_action[25], valve_min[25], valve_hours[25];
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
        }
        if (valve_number[index_oasis] <= 14) //Check if the valve is for me and then sent to the pg
        {
          action_valve_pg(valve_action[index_oasis], valve_number[index_oasis], valve_hours[index_oasis], valve_min[index_oasis]);
          //If is for me I active the flag to stop in case web want it
          if (valve_action[index_oasis] == 0)
          {
            active.valves[valve_number[index_oasis] - 1] = false;
            DPRINTLN("NOP");
            if (stop_man_web.manual_web)
              for (uint8_t t = 0; t < 10; t++)
                if (stop_man_web.active[t] && stop_man_web.valve_number[t] == valve_number[index_oasis])
                {
                  DPRINTLN("I turn of the timer set for me, KILL IT");
                  DPRINTLN(stop_man_web.valve_number[t]);
                  //Save the valve number:
                  stop_man_web.valve_number[t] = 0;
                  //Th current time on timer 0
                  stop_man_web.millix[t] = 0;
                  //The time in ms to stop the valve
                  stop_man_web.times[t] = 0;
                  //The flag of the valve in the struct:
                  stop_man_web.active[t] = false;
                  if (stop_man_web.number_timers <= 0)
                    stop_man_web.manual_web = false;
                  else
                    stop_man_web.number_timers--;
                }
          }
          else
          {
            //This is for general STOP in web:
            active.valves[valve_number[index_oasis] - 1] = true;

            //This is for stopping the PG problem:
            stop_man_web.manual_web = true;
            stop_man_web.active[stop_man_web.number_timers] = true;
            //Save the valve number:
            stop_man_web.valve_number[stop_man_web.number_timers] = valve_number[index_oasis];
            //Th current time on timer 0
            stop_man_web.millix[stop_man_web.number_timers] = millis();
            //The time in ms to stop the valve
            stop_man_web.times[stop_man_web.number_timers] = (uint32_t)valve_hours[index_oasis] * 60 * 60 * 1000 + (uint32_t)valve_min[index_oasis] * 60 * 1000;
            DPRINTLN("TIME: ");
            Serial.println(stop_man_web.times[stop_man_web.number_timers]);
            //I increment this pointer:
            stop_man_web.number_timers++;
            DPRINTLN("I have set a timer for stopping the valve when min pass");
          }
          delay(1500);
        }
      }
      //I send via radio to the receiver
    }
    else if (sTopic.indexOf("manprog") != -1) //done
    {
      send_web("/manprog", sizeof("/manprog"), identifier); // I send ack to the app
      //I obtein the values of the parser info:
      uint8_t activate = parsed["action"];
      // DPRINTLN(activate);
      String manual_program;
      manual_program = parsed["prog"].as<String>();
      // I first send it to
      delay(500);
      action_prog_pg(activate, manual_program.charAt(0)); //I send the command tom PG
      char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
      for (uint8_t index_prog; index_prog < 6; index_prog++)
        if (program_letters[index_prog] == manual_program.charAt(0))
          if (activate == 1)
            active.programs[index_prog] = true;
          else
            active.programs[index_prog] = false;
    }
    else if (sTopic.indexOf("oasis") != -1) //done
    {
      DPRINTLN("Publish in /oasis/app");
      JsonArray &oasis = parsed["oasis"];
      DPRINTLN("The value of the assignations are:");
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
    else if (sTopic.indexOf("program") != -1) //done
    {
      //I parse the letter of the program
      send_web("/program", sizeof("/program"), identifier);
      String manual_program = parsed["prog"].as<String>();
      //I determin the positions of the PG in which I have to write
      uint16_t position_starts = 416;     // This is the position of the starts in PG EEPROM memmory
      uint16_t mem_pos = 1024;            // This is the position of the starts in PG EEPROM memmory
      uint16_t position_percentage = 144; // This is the position of the irrig % in PG EEPROM memmory
      uint8_t position_week = 0xB0;       //This is for the week day starts
      uint8_t position_from_to = 0xC0;    //For changing the starting and stop irrig day, not year

      switch (manual_program.charAt(0))
      {
      case 'B':
        position_starts = 428;
        mem_pos = 1152;
        position_percentage = 146;
        position_week = 0xB1;
        position_from_to = 0xC4;
        break;
      case 'C':
        position_starts = 440;
        mem_pos = 1280;
        position_percentage = 148;
        position_week = 0xB2;
        position_from_to = 0xC8;
        break;
      case 'D':
        position_starts = 452;
        mem_pos = 1408;
        position_percentage = 150;
        position_week = 0xB3;
        position_from_to = 0xCC;
        break;
      case 'E':
        position_starts = 464;
        mem_pos = 1536;
        position_percentage = 152;
        position_week = 0xB4;
        position_from_to = 0xD0;
      case 'F':
        position_starts = 476;
        mem_pos = 1664;
        position_percentage = 154;
        position_week = 0xB5;
        position_from_to = 0xD4;
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
            DPRINTLN(" ");
            delay(800);
          }
          start_to_clear--;
          position_end -= 2;
        }
        //WEEK STARTS:
        read_pg_web = true; //This flag is for modified flash memmory;
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
            DPRINTLN(" ");
            clear_num++;
          }
          val = time_to_pg_format(time_valves[number_valves][0], time_valves[number_valves][1]);
          String mem_time = String(val, HEX);
          if (mem_time.length() == 1)
            mem_time = '0' + mem_time;
          mem_time.toUpperCase();
          DPRINTLN(mem_time);
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
          DPRINTLN(" ");
          clear_num++;
        }
      }
      //Write in the PG memmory the irrig %
      uint16_t irrig_percent = parsed["water"].as<uint16_t>();
      if (parsed["water"].success())
        write_percentage_pg(position_percentage, irrig_percent);

      if (parsed["from"].success())
      {
        String time_from = parsed["from"].as<String>();
        write_start_stop_pg(true, time_from, position_from_to);
        String time_to = parsed["to"].as<String>();
        write_start_stop_pg(false, time_to, position_from_to);
      }
      LOGLN(manual_program);
    }
    else if (sTopic.indexOf("query") != -1) //done
    {
      // send_web("/query", sizeof("/query"), identifier);
      json_query(String(identifier).c_str(), "AUTO");
    }
    else if (sTopic.indexOf("general") != -1) //receive the delay between valves and mv and valve
    {
      send_web("/general", sizeof("/general"), identifier);
      uint8_t pump_delay = parsed["pump_delay"].as<uint8_t>();
      uint8_t valve_delay = parsed["valve_delay"].as<uint8_t>();
      String web_time = parsed["date"].as<String>();
      if (parsed["pump_delay"].success())
      {
        if (pump_delay < 0)
          pump_delay = 0xFF + pump_delay + 1;
        cmd_write_data[13] = '0';
        cmd_write_data[14] = '5';
        cmd_write_data[15] = '1';
        String ret_valve = String(valve_delay, HEX);
        if (ret_valve.length() == 1)
          ret_valve = '0' + ret_valve;
        ret_valve.toUpperCase();
        cmd_write_data[17] = ret_valve.charAt(0);
        cmd_write_data[18] = ret_valve.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
        delay(800);
      }
      if (parsed["valve_delay"].success())
      {
        cmd_write_data[13] = '0';
        cmd_write_data[14] = '5';
        cmd_write_data[15] = '0';
        String ret_mv = String(pump_delay, HEX);
        if (ret_mv.length() == 1)
          ret_mv = '0' + ret_mv;
        ret_mv.toUpperCase();
        cmd_write_data[17] = ret_mv.charAt(0);
        cmd_write_data[18] = ret_mv.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
        for (i = 0; i < sizeof(cmd_write_data); i++)
          Serial.write(cmd_write_data[i]);
      }
      if (parsed["date"].success())
      {
        uint8_t time_hours = (web_time.charAt(11) - '0') * 10 + (web_time.charAt(12) - '0');
        uint8_t time_min = (web_time.charAt(14) - '0') * 10 + (web_time.charAt(15) - '0');
        uint8_t time_day = (web_time.charAt(0) - '0') * 10 + (web_time.charAt(1) - '0');
        uint8_t time_month = (web_time.charAt(3) - '0') * 10 + (web_time.charAt(4) - '0');
        uint8_t time_year = (web_time.charAt(8) - '0') * 10 + (web_time.charAt(9) - '0');

        uint16_t d = time_day;
        uint16_t m = time_month;
        uint16_t y = 2000 + time_year;
        uint16_t weekday = (d += m < 3 ? y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) % 7;
        //Set PG in time:
        if (weekday == 0)
          weekday = 7;

        DPRINTLN("");
        rtc.updateTime();
        rtc.setDate(time_day);
        rtc.setMonth(time_month);
        rtc.setYear(time_year);
        rtc.setMinutes(time_min);
        rtc.setHours(time_hours);
        change_time_pg(weekday, rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); //year/month/week/day/hour/min
        DPRINTLN(rtc.stringTime());
        DPRINTLN(rtc.stringDate());
      }
      //TODO: Write in PG MEMMORY
    }
    else if (sTopic.indexOf("stop") != -1) //I stop all the stuff open
    {
      send_web("/stop", sizeof("/stop"), identifier);
      uint8_t index_close = 0;
      for (; index_close < 14; index_close++)
        if (active.valves[index_close])
        {
          action_valve_pg(false, index_close + 1, 0, 0);
          delay(1000);
        }
      char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
      for (index_close = 0; index_close < 6; index_close++)
        if (active.programs[index_close])
        {
          action_prog_pg(0, program_letters[index_close]); //I send the command tom PG
          active.programs[index_close] = false;
          send_stop_program = true;
          delay(800);
        }
      stop_man_web.manual_web = false;
      stop_man_web.number_timers = 0;
      for (uint8_t c = 0; c < 10; c++)
      {
        stop_man_web.active[c] = false;
        stop_man_web.valve_number[c] = 0;
        stop_man_web.millix[c] = 0;
        stop_man_web.times[c] = 0;
      }
    }
  }
}
void write_start_stop_pg(bool from, String time, uint8_t dir)
{
  //Obtein the time in the correct format:
  uint8_t time_day = (time.charAt(0) - '0') * 10 + (time.charAt(1) - '0');
  uint8_t time_month = (time.charAt(3) - '0') * 10 + (time.charAt(4) - '0');

  String date_day = String(time_day, HEX);
  if (date_day.length() != 2)
    date_day = '0' + date_day;
  date_day.toUpperCase();

  String date_month = String(time_month, HEX);
  if (date_month.length() != 2)
    date_month = '0' + date_month;
  date_month.toUpperCase();

  uint8_t add_from = 0;
  if (!from)
    add_from = 2;
  //I set the position to write:
  String mem_pos = String(dir + add_from, HEX);
  mem_pos.toUpperCase();
  cmd_write_data[13] = '0';
  cmd_write_data[14] = mem_pos.charAt(0);
  cmd_write_data[15] = mem_pos.charAt(1);
  cmd_write_data[17] = date_day.charAt(0);
  cmd_write_data[18] = date_day.charAt(1);
  calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
  softSerial.write(cmd_write_data, sizeof(cmd_write_data));
  delay(800);
  //And then the month
  mem_pos = String(dir + add_from + 1, HEX);
  mem_pos.toUpperCase();
  cmd_write_data[13] = '0';
  cmd_write_data[14] = mem_pos.charAt(0);
  cmd_write_data[15] = mem_pos.charAt(1);
  cmd_write_data[17] = date_month.charAt(0);
  cmd_write_data[18] = date_month.charAt(1);
  calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
  softSerial.write(cmd_write_data, sizeof(cmd_write_data));
  delay(800);
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
void print_flash()
{
  DPRINT("The App name is: ");
  for (int a = 0; a < UUID_LEN; a++)
    Serial.write(sys.devUuid[a]);
  DPRINTLN(" ");
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
  calcrc((char *)command, len - 2);

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
        active.programs[index_program] = true;
      else
        active.programs[index_program] = false;
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
  DPRINTLN(" ");
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
  DPRINTLN(" ");
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
        // sys.childValves[m][n] = strtol(aux.c_str(), NULL, HEX) + 1;
        valveDef[m] = 1;
      }

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
    for (j = 0; j < 2; j++)
    { //for each child
      DPRINT("Send change to OASIS: ");
      DPRINTLN(j + 1);
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
}
void json_program_valves(uint8_t program)
{
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  //Generate the structure of the program via json
  root["prog"] = String(program_letters[program]);
  JsonArray &valves = root.createNestedArray("valves");
  for (uint8_t index_valves = 0; index_valves < 14; index_valves++)
  {
    DPRINTLN(prog[program].irrigTime[index_valves]);
    if (prog[program].irrigTime[index_valves] <= 200)
    {
      JsonObject &irrig = root.createNestedObject("");
      DPRINTLN(prog[program].irrigTime[index_valves]);
      irrig["v"] = index_valves + 1;
      int time_compleat = pg_reag_to_web((uint16_t)prog[program].irrigTime[index_valves]);
      uint16_t time_in_format_h;
      uint16_t time_in_format_m;
      if (time_compleat >= 60)
      {
        time_in_format_h = time_compleat / 60;
        time_in_format_m = time_compleat % 60;
        String h = String(time_in_format_h);
        String m = String(time_in_format_m);
        if (h.length() == 1)
          h = '0' + h;
        if (m.length() == 1)
          m = '0' + m;
        irrig["time"] = h + ":" + m;
      }
      else
      {
        String x = String(time_compleat);
        if (x.length() == 1)
          x = '0' + x;
        irrig["time"] = "00:" + x;
      }
      valves.add(irrig);
    }
  }
  char json[300];
  root.printTo(json);
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
  mqttClient.publish("connect", (const uint8_t *)json, strlen(json), false);
}
void json_query(const char id[], char status[])
{
  uint16_t b;
  DynamicJsonBuffer jsonBuffer(244);
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
  rtc.updateTime();
  root["date"] = String(rtc.stringDate()) + " " + String(rtc.stringTime());
  //I check if there is any prpogram active and I send to the web:
  JsonArray &active_json = root.createNestedArray("prog_active");
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  for (uint8_t prog = 0; prog < 6; prog++)
    if (active.programs[prog])
      active_json.add(String(program_letters[prog]));
  //I generate the main json frame
  char json[250];
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
    DPRINTLN(" ");
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
      //This command received 0123456 and the RTC received: 1234560 for monday to sunday
      if (time_day_week == 6)
        time_day_week = 0;
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
        json_program_valves(i);
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
void check_time() // This function test if the current time fix with the program time
{
  rtc.updateTime();
  uint8_t rtc_value = rtc.getWeekday();
  uint8_t value_to_AND = 0;
  for (uint8_t index_program = 0; index_program < 6; index_program++)
  {
    if (rtc_value == 0)
      rtc_value = 7;
    value_to_AND = pow(2, rtc_value - 1);
    if (prog[index_program].wateringDay & value_to_AND)
      for (uint8_t index_time_h = 0; index_time_h < 6; index_time_h++)
        if (prog[index_program].start[index_time_h][0] == rtc.getHours())
          if (prog[index_program].start[index_time_h][1] == rtc.getMinutes())
          {
            DPRINTLN("ES LA HORA BUENA");
            if (index_program == 0)
            {
              json_program_action(true, 'A');
              active.programs[0] = true;
            }
            else if (index_program == 1)
            {
              json_program_action(true, 'B');
              active.programs[1] = true;
            }
            else if (index_program == 2)
            {
              json_program_action(true, 'C');
              active.programs[2] = true;
            }
            else if (index_program == 3)
            {
              json_program_action(true, 'D');
              active.programs[3] = true;
            }
            else if (index_program == 4)
            {
              active.programs[4] = true;
              json_program_action(true, 'E');
            }
            else if (index_program == 5)
            {
              active.programs[5] = true;
              json_program_action(true, 'F');
            }
          }
    // check if the hours are fix and we can start a program
  }
}