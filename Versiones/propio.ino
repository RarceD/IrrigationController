typedef enum
{
  MANUAL_VALVE_COMMAND,
  MANUAL_PROG_COMMAND,
  TIME_CHANGE_COMMAND,
  STOP_ALL_COMMAND
} send_commands_oasis;

typedef enum{
   READ_PROGRAM_A,
   READ_PROGRAM_B,
   READ_PROGRAM_C,
   READ_PROGRAM_D,
   READ_PROGRAM_E,
   READ_PROGRAM_F,
   WRITE_PROGRAM
}request_commands_pg;


void loop()
{
  while (softSerial.available())
  {
    Serial.print((char)softSerial.read());
  }
  listening_pg();
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
      send_oasis(MANUAL_VALVE_COMMAND);
      Serial.print(valve_number);
      Serial.print(" valvula - ");
      Serial.print(valve_time_hour);
      Serial.print(" horas - ");
      Serial.print(valve_time_min);
      Serial.println(" horas. ");
    }
    else if (pg.indexOf("MANPRG START#") > 0)
    {
      uint8_t prog_name = (pgData[pg.indexOf("MANPRG START#") + 13] - 65);
      Serial.print("Programa:");
      Serial.println(prog_name);
      send_oasis(MANUAL_PROG_COMMAND);
    }
    else if (pg.indexOf("STOP ALL") > 0)
    {
      Serial.println("Paro TODO");
      send_oasis(STOP_ALL_COMMAND);
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

      send_oasis(TIME_CHANGE_COMMAND);
    }
    else if (pg.indexOf("PAIRING#") > 0)
    {
      //I always obtein the number of oasis without one unit due to format 8bit vs 16 bits
      String valve_number = getValue(pg, '#', 1);
      int valve_number_true = (int)strtol(&valve_number[0], NULL, 16);
      Serial.println(valve_number_true);
      String valve_assigned;
      int oasis_valves[4];
      for (int k = 0; k < 4; k++)
      {
        valve_assigned = getValue(getValue(pg, '#', 2), ' ', k);
        oasis_valves[k] = (int)strtol(&valve_assigned[0], NULL, 16);
        Serial.println(oasis_valves[k]);
      }
    }
  }
}
void send_oasis(uint8_t command)
{
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