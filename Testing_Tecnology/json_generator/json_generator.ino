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

void loop()
{
  if (Serial.available())
  {
    int a = Serial.read();
    if (a == 97)
      json_program(0);
    if (a == 98)
      json_clear_starts(0);
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
    if (String(prog[index_program].irrigTime[index_valves]) != "255")
    {
      JsonObject &irrig = root.createNestedObject("");
      irrig["v"] = index_valves + 1;
      uint8_t time_in_format_h = prog[index_program].irrigTime[index_valves] / 60;
      uint8_t time_in_format_m = prog[index_program].irrigTime[index_valves] % 60;
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

  return 0;
}