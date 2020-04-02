#include <SoftwareSerial.h>
#define SSerialRX 16
#define SSerialTX 17
SoftwareSerial softSerial(SSerialRX, SSerialTX);

uint8_t buf[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, '1', '4', 0x23, '0', '0', '0', '1', 0x23, 0x03, 0, 0};

static uint8_t cmd_stop_manprg[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manprg[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manvalv[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, '0', '1', 0x23, '0', '0', '0', '1', 0x23, 0x03, 0, 0};
static uint8_t cmd_stop_manvalv[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};

static uint8_t cmd_set_time[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'T', 'I', 'M', 'E', 0x23, '0', '9', '0', '5', '0', '0', 0x23, '0', '3', 0x23, 0x03, 0, 0};

// static uint8_t cmd_read_time[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'T', 'I', 'M', 'E', 0x23, 0x03, 0, 0};

static uint8_t cmd_read_line[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'L', 'I', 'N', 'E', 0x23, '3', 'E', '0', 0x23, 0x03, 0, 0};

static uint8_t cmd_write_data[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'D', 'A', 'T', 'A', 0x23, ' ', ' ', ' ', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_read_time[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'T', 'I', 'M', 'E', 0x23, 0x03, 0, 0};

static uint8_t cmd_com_error[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'C', 'O', 'M', 0x23, 0, 0};
static uint8_t cmd_ok[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'O', 'K', 0x23, 0, 0};
static uint8_t cmd_nok[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'N', 'O', 'K', 0x23, 0, 0};

uint16_t i, j;
int calcrc(char ptr[], int length);
void open_valve_pg(bool state, uint8_t valve, uint8_t time_hours, uint8_t time_minutes);
void action_prog_pg(bool state, char program);

/******************************************************************* setup section ************************************************************************************/
void setup()
{

  Serial.begin(115200);
  // flash.powerUp();
  // flash.begin();
  // flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  // flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  softSerial.begin(9600);
  calcrc((char *)cmd_read_line, sizeof(cmd_read_line) - 2);
  softSerial.write(cmd_read_line, sizeof(cmd_read_line));
  Serial.print("Command send: ");
  Serial.println(" ");
  for (i = 0; i < sizeof(cmd_read_line); i++)
  {
    Serial.write(cmd_read_line[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Read response: ");
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{

  if (Serial.available())
  {
    int a = Serial.read();
    // Serial.println(a);
    if (a == 97) // Open all valves
      action_valve_pg(true, 1, 0, 35);
    if (a == 98) //Close all valves
      action_valve_pg(false, 1, 0, 35);
    if (a == 99) //Open all programs
    {
      action_prog_pg(true, 'C');
      delay(10000);
      Serial.println("YA");
      action_prog_pg(false, 'C');
    }
    if (a == 100) //Press the D for change PG memmory irrigation valves
    {
      // 13   14   15  the position in memmory
      // 17   18       what I write
      uint8_t time_valves[14][2]; //This is a huge and ridiculous PG valve register
      time_valves[0][0] = 0;      //Los horas de V1
      time_valves[0][1] = 27;     //Los minutos
      time_valves[1][0] = 1;      //Los horas de V2
      time_valves[1][1] = 24;     //Los minutos
      uint16_t mem_pos = 400;
      for (int index_compleat = 0; index_compleat < 2; index_compleat++)
      {
        uint16_t val = time_to_pg_format(time_valves[index_compleat][0], time_valves[index_compleat][1]);
        String mem_time = String(val, HEX);
        if (mem_time.length() != 2)
          mem_time = '0' + mem_time;
        mem_time.toUpperCase();
        Serial.println(mem_time.charAt(1));
        Serial.println(mem_time.charAt(0));

        String mem_starts = String(mem_pos + index_compleat);
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
          Serial.print(" ");
        }
        delay(800);
        Serial.println(" ");
      }
    }
    if (a == 101) // Press E to change the PG starts
    {
      uint16_t start_time_hours = 0, start_time_min = 0;
      uint16_t position_starts = 416;
      String mem_starts_h;
      uint8_t time_prog[6][2];
      time_prog[0][0] = 1;  //horas arranque 1 programa A
      time_prog[0][1] = 2;  //minutos
      time_prog[1][0] = 3;  //horas arranque 2 programa A
      time_prog[1][1] = 4;  //minutos
      time_prog[2][0] = 5;  //horas arranque 3 programa A
      time_prog[2][1] = 6;  //minutos
      time_prog[3][0] = 7;  //horas arranque 4 programa A
      time_prog[3][1] = 8;  //minutos
      time_prog[4][0] = 9;  //horas arranque 5 programa A
      time_prog[4][1] = 10; //minutos

      for (int index_complet = 0; index_complet < 5; index_complet++)
      {
        delay(800);
        start_time_hours = time_to_pg_format(0, time_prog[index_complet][0]);
        String mem_time_start_hours = String(start_time_hours, HEX);
        mem_time_start_hours.toUpperCase();
        if (mem_time_start_hours.length() != 2)
          mem_time_start_hours = '0' + mem_time_start_hours;
        mem_starts_h = String(position_starts + 0, HEX);
        mem_starts_h.toUpperCase();
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
      }
      Serial.println(" ");
    }
    if (a == 102) // Press F to change the irrigation percentage
    {
      write_percentage_pg(144, 22); //PROG A
      //write_percentage_pg(146, 22); //PROG B
      //write_percentage_pg(148, 22);//PROG C
      //write_percentage_pg(150, 22);//PROG D
      //write_percentage_pg(152, 33);//PROG E
      //write_percentage_pg(154, 33);//PROG F
    }
    if (a == 103) // PRess G to change time of PG
    {
      // change_time_pg(rtc.getYear(), rtc.getMonth(), rtc.getDate(),rtc.getWeekday(), rtc.getHours(),rtc.getMinutes()); //year/month/week/day/hour/min
      change_time_pg(1, 9, 39); //year/month/week/day/hour/min
    }
    if (a == 104) //press H
    {
      String day = "71";
      char days[day.length() + 1];
      day.toCharArray(days, sizeof(days));
      uint8_t position_week = 0x81;
      change_week_pg(days, sizeof(days), position_week);
    }
    if (a == 105) // press 'i' to clear starts
    {

      uint8_t start_to_clear = 6 - 0;
      String mem_starts_h;
      uint16_t position_start = 0x1B8;
      uint16_t position_end = position_start + 10; // try to write in 0x1AA and then in 0x1A8
      while (start_to_clear > 0)
      {
        for (uint8_t times = 0; times <= 1; times++)
        {
          //First clear from the bottom to the top of the memmory:
          mem_starts_h = String(position_end + times, HEX);
          mem_starts_h.toUpperCase();
          cmd_write_data[13] = mem_starts_h.charAt(0);
          cmd_write_data[14] = mem_starts_h.charAt(1);
          cmd_write_data[15] = mem_starts_h.charAt(2);
          cmd_write_data[17] = 'F';
          cmd_write_data[18] = 'F';
          calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
          softSerial.write(cmd_write_data, sizeof(cmd_write_data));
          for (int i = 0; i < sizeof(cmd_write_data); i++)
            Serial.write(cmd_write_data[i]);
          Serial.println(" ");
          delay(800);
        }
        start_to_clear--;
        position_end -= 2;
      }
    }
    if (a == 106) //press 'j'
    {
      uint8_t assign_test[] = {11, 22, 33, 44};
      change_oasis_assigned(6, assign_test);
    }
    if (a == 107) //prexx 'k'
    oasis_number
      String str_oasis_number = String(oasis_number++, HEX);
      if (str_oasis_number.length() == 1)
        str_oasis_number = '0' + str_oasis_number;
      str_oasis_number.toUpperCase();
      cmd_write_data[13] = '3';
      cmd_write_data[14] = 'E';
      cmd_write_data[15] = '0';
      cmd_write_data[17] = str_oasis_number.charAt(0);
      cmd_write_data[18] = str_oasis_number.charAt(1);
      calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
      softSerial.write(cmd_write_data, sizeof(cmd_write_data));
      for (i = 0; i < sizeof(cmd_write_data); i++)
        Serial.write(cmd_write_data[i]);
    }
  }

  while (softSerial.available())
    Serial.print((char)softSerial.read());
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
void change_time_pg(uint8_t week, uint8_t hours, uint8_t minutes) //, uint8_t *day, uint8_t *hours, uint8_t *minutes)
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
  calcrc((char *)cmd_set_time, sizeof(cmd_set_time) - 2);
  softSerial.write(cmd_set_time, sizeof(cmd_set_time)); //real send to PG
  for (i = 0; i < sizeof(cmd_set_time); i++)
    Serial.write(cmd_set_time[i]);
}
void change_week_pg(char *date, uint8_t lenght, uint8_t program)
{
  uint8_t value_to_memory = 0;
  for (uint8_t index = 0; index < lenght; index++)
  {
    Serial.write(*(date + index));
    Serial.println("");
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
    Serial.print("_____");
    Serial.println(value_to_memory);
  }

  String str_pos_week = String(program, HEX);
  str_pos_week.toUpperCase();

  String str_days = String(value_to_memory, HEX);
  str_days.toUpperCase();

  Serial.println(str_days);

  //If is less than 0x0F I add a 0
  if (str_days.length() != 2)
    str_days = '0' + str_days;
  cmd_write_data[13] = '0';
  cmd_write_data[14] = str_pos_week.charAt(0);
  cmd_write_data[15] = str_pos_week.charAt(1);
  cmd_write_data[17] = str_days.charAt(0);
  cmd_write_data[18] = str_days.charAt(1);
  calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
  // softSerial.write(cmd_write_data, sizeof(cmd_write_data)); //real send to PG
  for (i = 0; i < sizeof(cmd_write_data); i++)
    Serial.write(cmd_write_data[i]);
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
    for (i = 0; i < sizeof(cmd_write_data); i++)
      Serial.write(cmd_write_data[i]);
    delay(1000);
    Serial.println(" ");
  }
}
void fault_in_radio(uint8_t oasis_number, bool error)
{
  String str_oasis_number = String(oasis_number, HEX);
  if (str_oasis_number.length() == 1)
    str_oasis_number = '0' + str_oasis_number;
  str_oasis_number.toUpperCase();
  if (error)
  {
    cmd_nok[12] = str_oasis_number.charAt(0);
    cmd_nok[13] = str_oasis_number.charAt(1);
    calcrc((char *)cmd_nok, sizeof(cmd_nok) - 2);
    softSerial.write(cmd_nok, sizeof(cmd_nok));
    for (i = 0; i < sizeof(cmd_nok); i++)
      Serial.write(cmd_nok[i]);
  }
  else
  {
    cmd_ok[12] = str_oasis_number.charAt(0);
    cmd_ok[13] = str_oasis_number.charAt(1);
    calcrc((char *)cmd_ok, sizeof(cmd_ok) - 2);
    softSerial.write(cmd_ok, sizeof(cmd_ok));
    for (i = 0; i < sizeof(cmd_ok); i++)
      Serial.write(cmd_ok[i]);
  }
  Serial.println(" ");
}
