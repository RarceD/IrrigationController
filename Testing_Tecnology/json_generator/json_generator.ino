#include <ArduinoJson.h>
#include <JamAtm-Vyrsa.h>

#define CS_M 22
#define CS_RF 23

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
program prog[6];
void setup()
{
  Serial.begin(115200);
  flash.powerUp();
  flash.begin();
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
}
uint16_t pg_reag_to_web(uint16_t pg_time);
void loop()
{
  if (Serial.available())
  {
    int a = Serial.read();
    if (a == 97)
      json_program(0);
    if (a == 98)
      json_clear_starts(0);
    if (a == 99)
      Serial.println(pg_reag_to_web(85));
    if (a == 100)
      json_program_valves(0);
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
  Serial.println();
  root.prettyPrintTo(Serial);
}
void json_clear_starts(uint8_t program)
{
  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};

  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();
  root["program"] = String(program_letters[program]);
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