#include <ArduinoJson.h>

void setup()
{
  Serial.begin(115200);
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject &root = jsonBuffer.createObject();

  root["prog"] = 'A';
  JsonArray &starts = root.createNestedArray("starts");
  starts.add("12:45");
  starts.add("13:05");
  starts.add("02:35");
  starts.add("01:15");
  root["water"] = 100;
  JsonArray &valves = root.createNestedArray("valves");
 JsonArray &irrig = root.createNestedArray("irrig");

  valves.add(irrig);
  irrig.add(12);
  irrig.add(12);

  //irrig["hola"] = 12;
  //irrig["adios"] = 12;

  //valves.add("time");
  // valves.add("time : 12:12");
  //JsonArray &irrig = root.createNestedArray("irrig");
  //irrig.add("12:45");
  //irrig.add("13:05");
  Serial.println();

  root.prettyPrintTo(Serial);
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