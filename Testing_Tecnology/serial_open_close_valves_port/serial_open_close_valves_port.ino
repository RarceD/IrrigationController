#include <SoftwareSerial.h>
#define SSerialRX 16
#define SSerialTX 17
SoftwareSerial softSerial(SSerialRX, SSerialTX);

uint8_t buf[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, '1', '4', 0x23, '0', '0', '0', '1', 0x23, 0x03, 0, 0};

static uint8_t cmd_stop_manprg[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manprg[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manvalv[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, '0', '1', 0x23, '0', '0', '0', '1', 0x23, 0x03, 0, 0};
static uint8_t cmd_stop_manvalv[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};

static uint8_t cmd_read_line[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'L', 'I', 'N', 'E', 0x23, '4', '0', '0', 0x23, 0x03, 0, 0};

static uint8_t cmd_write_data[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'D', 'A', 'T', 'A', 0x23, ' ', ' ', ' ', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};

uint16_t i, j;
int calcrc(char ptr[], int length);
void open_valve_pg(bool state, uint8_t valve, uint8_t time_hours, uint8_t time_minutes);
void action_prog_pg(bool state, char program);

/******************************************************************* setup section ************************************************************************************/
void setup()
{

  Serial.begin(115200);
  softSerial.begin(9600);
  calcrc((char *)cmd_read_line, sizeof(cmd_read_line) - 2);
  softSerial.write(cmd_read_line, sizeof(cmd_read_line));
  Serial.print("Command send: ");
  for (i = 0; i < sizeof(cmd_read_line); i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
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
    Serial.println(a);
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
      uint16_t val = time_to_pg_format(0, 3);
      String mem_time = String(val, HEX);
      if (mem_time.length() != 2)
        mem_time = '0' + mem_time;
      mem_time.toUpperCase();
      Serial.println(mem_time.charAt(1));
      Serial.println(mem_time.charAt(0));
      // I set starts:
      for (int index_mem = 0; index_mem < 5; index_mem++)
      {
        String mem_starts = String(400 + index_mem);
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
          // Serial.print(cmd_write_data[i], HEX);
          Serial.write(cmd_write_data[i]);
          Serial.print(" ");
        }
        delay(1000);
        Serial.println(" ");
      }
    }
    if (a == 101) // Press E to change the PG starts
    {
      uint16_t start_time_hours = 0, start_time_min = 0;
      uint16_t position_starts = 416;
      String mem_starts_h;
      uint8_t time_prog[6][2];
      time_prog[0][0] = 1; //horas arranque 1 programa A
      time_prog[0][1] = 2; //minutos
      time_prog[1][0] = 3; //horas arranque 2 programa A
      time_prog[1][1] = 4; //minutos
      time_prog[2][0] = 5; //horas arranque 3 programa A
      time_prog[2][1] = 6; //minutos
      time_prog[3][0] = 7; //horas arranque 4 programa A
      time_prog[3][1] = 8; //minutos
      time_prog[4][0] = 9; //horas arranque 5 programa A
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

      /*
      for (int index_starts = 1; index_starts < 20; index_starts++)
      {
        String mem_starts = String(position_starts + index_starts, HEX);
        mem_starts.toUpperCase();
        cmd_write_data[13] = mem_starts.charAt(0);
        cmd_write_data[14] = mem_starts.charAt(1);
        cmd_write_data[15] = mem_starts.charAt(2);
        cmd_write_data[17] = mem_time_start.charAt(0);
        cmd_write_data[18] = mem_time_start.charAt(1);
        calcrc((char *)cmd_write_data, sizeof(cmd_write_data) - 2);
        softSerial.write(cmd_write_data, sizeof(cmd_write_data));
        for (i = 0; i < sizeof(cmd_write_data); i++)
        {
          // Serial.print(cmd_write_data[i], HEX);
          Serial.write(cmd_write_data[i]);
          Serial.print(" ");
        }*/
      Serial.println(" ");
      delay(800);
    }
  }

  while (softSerial.available())
  {

    Serial.print((char)softSerial.read());
  }
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
