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
  //Generate the structure of the program
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();

  char program_letters[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  for (uint8_t index_program = 0; index_program < 6; index_program++)
  {
    root["prog"] = String(program_letters[index_program]);
    JsonArray &starts = root.createNestedArray("starts");
    //Generate all the starts
    for (uint8_t index_time = 0; index_time < 6; index_time++)
      if (prog[index_program].start[index_time][0] != 255)
        starts.add(String(prog[index_program].start[index_time][0]) + ":" + String(prog[index_program].start[index_time][1]));
    root["water"] = (int)(prog[index_program].waterPercent);
    JsonArray &valves = root.createNestedArray("valves");
    for (uint8_t index_valves = 0; index_valves < 128; index_valves++)
    {
       if (String(prog[index_program].irrigTime[index_valves]) != "255")
       {
        JsonObject &irrig = root.createNestedObject("");
        irrig["v"] = index_valves + 1;
        uint8_t time_in_format_h = prog[index_program].irrigTime[index_valves] /60;
        uint8_t time_in_format_m = prog[index_program].irrigTime[index_valves] % 60;
        String formated_time_json = "00:00";
        if (time_in_format_h < 10){
          formated_time_json
        }  
        irrig["time"] = String(time_in_format_h) + ":" + String(time_in_format_m);
        valves.add(irrig);
      }
    }

    Serial.println();
    root.prettyPrintTo(Serial);
  }

  //https://arduinojson.org/v6/api/jsonobject/createnestedarray/

  //valves.add("time");
  // valves.add("time : 12:12");
  //JsonArray &irrig = root.createNestedArray("irrig");
  //irrig.add("12:45");
  //irrig.add("13:05");
  // This prints:
  // {
  //   "sensor": "gps",
  //   "time": 1351824120,
  //   "data": [
  //     48.756080,
  //     2.302038
  //   ]
  // }
}

void loop()
{
  if (Serial.available())
  {
    int a = Serial.read();
    /*
    if (a == 97)
    { //If I pressed A
      //First the program A
      //uint8_t start[6][2];
      //uint16_t irrigTime[128];
      for (int n = 0; n < 6; n++)
      {
        Serial.print(prog[0].start[n][0]);
        Serial.print(":");
        Serial.print(prog[0].start[n][1]);
        Serial.print("__");
      }
      Serial.println("");
      for (int n = 0; n < 50; n++)
      {
        Serial.print(prog[0].irrigTime[n]);
        Serial.print(":");
      }
      Serial.println("");
    }
    */
  }
}

// See also
// --------
//
// The website arduinojson.org contains the documentation for all the functions
// used above. It also includes an FAQ that will help you solve any
// serialization problem.
// Please check it out at: https://arduinojson.org/
//
// The book "Mastering ArduinoJson" contains a tutorial on serialization.
// It begins with a simple example, like the one above, and then adds more
// features like serializing directly to a file or an HTTP request.
// Please check it out at: https://arduinojson.org/book/