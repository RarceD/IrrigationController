#include <ArduinoJson.h>
#include <JamAtm-Vyrsa.h>

#define CS_M 22
#define CS_RF 23

#define MAX_NODE_NUMBER 7
#define MAX_MANUAL_TIMERS 120
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000
#define MAX_NUM_MESSAGES 15

#define VREF_IN 24
#define VREF_EXT 29

SPIFlash flash(CS_M);
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

program prog[6];
sysVar sys;
void setup()
{
  Serial.begin(115200);
  flash.powerUp();
  flash.begin();
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
}
uint16_t pg_reag_to_web(uint16_t pg_time);
char assignation[] = {8, 33, 6, 2};
void loop()
{
  if (Serial.available())
  {
    int a = Serial.read();
    if (a == 97) //Pulse: a
      json_program(0);
    if (a == 98) //Pulse: b
      json_clear_starts(0);
    if (a == 99) //Pulse: c
      json_program_valves(0);
    if (a == 100) //Pulse: d
      json_connect_app();
    if (a == 101) //Pulse: e
      json_query("1233", "AUTO");
    if (a == 102) //Pulse f
      json_week_days(4, prog[4].wateringDay);
    if (a == 103) //Pulse g
      json_oasis_paring(true, 1, 1, assignation);
    if (a == 104) //Pulse h
      json_oasis_paring(false,1, assignation);
  }
}
void json_program(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(200);
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
  root["water"] = (int)(prog[index_program].waterPercent);
  JsonArray &valves = root.createNestedArray("valves");
  for (uint8_t index_valves = 0; index_valves < 80; index_valves++)
    if (String(prog[index_program].irrigTime[index_valves]) != "255" && String(prog[index_program].irrigTime[index_valves]) != "247")
    {
      JsonObject &irrig = root.createNestedObject("");
      irrig["v"] = index_valves + 1;
      uint16_t time_compleat = pg_reag_to_web(prog[index_program].irrigTime[index_valves]);
      uint16_t time_in_format_h = time_compleat / 60;
      uint16_t time_in_format_m = time_compleat % 60;
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

  JsonArray &week_day = root.createNestedArray("week_day");

  Serial.println();
  root.prettyPrintTo(Serial);
}
void json_clear_starts(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};

  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();
  root["prog"] = String(program_letters[program]);
  root["starts"] = "[]";
  Serial.println();
  root.prettyPrintTo(Serial);
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
void json_program_valves(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(500);
  JsonObject &root = jsonBuffer.createObject();
  uint8_t index_program = program; //I generate the json only for the program request
  root["prog"] = String(program_letters[index_program]);
  JsonArray &valves = root.createNestedArray("valves");
  for (uint8_t index_valves = 0; index_valves < 15; index_valves++)
    if (String(prog[index_program].irrigTime[index_valves]) != "255" && String(prog[index_program].irrigTime[index_valves]) != "247")
    {
      JsonObject &irrig = root.createNestedObject("");
      irrig["v"] = index_valves + 1;
      uint16_t time_compleat = pg_reag_to_web(prog[index_program].irrigTime[index_valves]);
      uint16_t time_in_format_h = time_compleat / 60;
      uint16_t time_in_format_m = time_compleat % 60;
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
  root.prettyPrintTo(Serial);

  // root.printTo(json);
  // mqttClient.publish((String(sys.devUuid) + "/program").c_str(), (const uint8_t *)json, strlen(json), false);
}
void json_connect_app()
{
  //Generate the structure of the program via json
  DynamicJsonBuffer jsonBuffer(500);
  JsonObject &root = jsonBuffer.createObject();
  root["uuid"] = String(sys.devUuid);
  root["model"] = "6011";
  char json[500];
  root.prettyPrintTo(Serial);
}
void json_query(char id[], char status[])
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

  int bat_percentage = map((int)round(bat), 6, 13, 5, 100);
  root["id"] = String(id);
  root["status"] = String(status);
  JsonObject &battery = root.createNestedObject("battery");
  battery["com"] = bat_percentage - 2;
  battery["prog"] = bat_percentage;
  JsonObject &oasis = battery.createNestedObject("oasis");
  oasis["id"] = 1;
  oasis["bat"] = 78;
  root["connection"] = 68;
  JsonArray &active = root.createNestedArray("prog_active");
  char json[300];
  root.prettyPrintTo(Serial);
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
  char json[300];
  root.prettyPrintTo(Serial);
}
void json_oasis_paring(bool pairing, uint8_t id_uuid, char *assigned)
{
  //First to the pairing topic with the uuid and the to oasis
  //The uuid and the id are the same
  DynamicJsonBuffer jsonBuffer(500);
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
  root.prettyPrintTo(Serial);
}

/* PAIIIIIIRING first msg
{
  "oasis": {
    " ": {
      "id": 2,
      "uuid": "uuid_random_2",
      "assign": [
        1, 
        2,
        3,
        4
      ]
    }
  }
}
*/

/* OASIS second msg

{
  "oasis": {
    " ": {
      "id": 2,
      "assign": [
        11,
        22,
        33,
        44
      ]
    }
  }
}

*/