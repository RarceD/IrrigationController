#include <JamAtm-Vyrsa.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
/******************************************************************* debug ********************************************************************************************/

#define DEBUG_ON
#ifdef DEBUG_ON
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif
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

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

#define MAX_NODE_NUMBER 7
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000

/******************************************************************* declarations  ************************************************************************************/

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
  uint8_t currentFwVer[10];
} sysVar;
typedef struct
{
  uint8_t interval;
  uint8_t startDay;
  uint8_t wateringDay;
  uint16_t waterPercent;
  uint8_t start[TOTAL_START][2];
  uint8_t irrigTime[TOTAL_VALVE];
} program;
typedef enum
{
  MANUAL_VALVE_COMMAND,
  MANUAL_PROG_COMMAND,
  TIME_CHANGE_COMMAND,
  STOP_ALL_COMMAND
} send_commands_oasis;
typedef enum
{
  REQUEST_MANVAL,
  REQUEST_TIME,
  REQUEST_ASSIGNED_VALVES,
  REQUEST_STOP_ALL
} messages_radio;
typedef enum
{
  READ_PROGRAM_A,
  READ_PROGRAM_B,
  READ_PROGRAM_C,
  READ_PROGRAM_D,
  READ_PROGRAM_E,
  READ_PROGRAM_F,
  WRITE_PROGRAM
} request_commands_pg;

Jam jam;
sysVar sys;
RV1805 rtc;
//Sleep lowPower;
SimpleTimer timer;
SPIFlash flash(CS_M);
program prog[TOTAL_PROG];
SoftwareSerial softSerial(PG_RXD, PG_TXD);

RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);

uint8_t UUID_1[] = {29, 126, 254, 123, 181, 94, 75, 217, 175, 233, 194, 218, 54, 62, 115, 110};

// Dont put this on the stack:
uint8_t data[70];
// THE HUGE BUFFER for SRAM:
uint8_t buf[70];
bool rf_flag = false;

String pg;
int timer_1;
char pgData[PG_MAX_LEN];
uint8_t i, j, rfId, cmd;
volatile uint8_t oldPort = 0x00;
volatile bool intButton = false, intRtc = false;
bool gprs_on = true, mode = false, comError[MAX_CHILD];
uint8_t valveDef[MAX_CHILD], progDef[TOTAL_PROG];

/******************************************************************* setup section ************************************************************************************/
void setup()
{
#ifdef DEBUG_ON
  Serial.begin(115200);
#endif
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
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t *)&prog, sizeof(prog));
  DPRINT(", UUID: ");
  DPRINTLN(sys.devUuid);
  manager.init();
  manager.setRetries(10);
  manager.setTimeout(175);
  driver.setTxPower(20, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.setAlarmMode(4);
  rtc.setAlarm(0, 30, 0, 0, 0);
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  Serial.println(rtc.stringTime());
  Serial.println(rtc.stringDate());

  //getAllFromPG();
  for (i = 0; i < MAX_CHILD; i++)
    comError[i] = false;
  //jam.ledBlink(LED_SETUP, 1000);
}

char asignacion[] = {1, random(1, 128), random(1, 128), random(1, 128)};
/******************************************************************* main program  ************************************************************************************/
void loop()
{
  if (Serial.available())
  {
    uint8_t a = Serial.read();
    uint8_t nodo_envio[16];
    if (a == 97) // if I send letter a I request nodos
      send_nodo(UUID_1, REQUEST_MANVAL, 1, random(0, 12), random(1, 59), asignacion);
    if (a == 98) // if I send letter b I request just 1 NODES !
      send_nodo(UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
    if (a == 99)
      send_nodo(UUID_1, REQUEST_ASSIGNED_VALVES, 11, 0, 0, asignacion);
    if (a == 100) // CLOSE VALVE if d
      send_nodo(UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
  }
  listening_pg();
}

/*******************************************************************   functions     ************************************************************************************/
void send_nodo(uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
{
  //First write the destination of the message:
  bool f_man_valve = false, f_time = false, f_asigned = false, f_stop = false;
  switch (msg)
  {
  case REQUEST_MANVAL:
    f_man_valve = true;
    Serial.println("REQUEST MANVALVE");
    break;
  case REQUEST_TIME:
    f_time = true;
    Serial.println("REQUEST TIME");
    break;
  case REQUEST_ASSIGNED_VALVES:
    Serial.println("CHANGE ASIGNATION VALVE");
    f_asigned = true;
    break;
  case REQUEST_STOP_ALL:
    Serial.println("STOP ALL");
    f_stop = true;
    break;
  default:
    Serial.println("JAMAS SALE");
  }
  if (f_man_valve)
  {
    uint8_t str_manval[] = "##MANVAL#000#00:00#";
    if (valve > 99)
    {
      str_manval[9] = '1';
      str_manval[10] = ((valve - 100) / 10) + 0x30;
      str_manval[11] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_manval[9] = '0';
      str_manval[10] = (valve / 10) + 0x30;
      str_manval[11] = (valve % 10) + 0x30;
    }

    str_manval[13] = (hour / 10) + 0x30;
    str_manval[14] = (hour % 10) + 0x30;
    str_manval[16] = (minutes / 10) + 0x30;
    str_manval[17] = (minutes % 10) + 0x30;

    for (int i = 0; i < sizeof(str_manval); i++)
      data[i] = str_manval[i];
    f_man_valve = false;
  }
  else if (f_time)
  {
    rtc.updateTime();
    rtc_node((int)rtc.getHours(), (int)rtc.getMinutes(), (int)rtc.getSeconds(), (int)rtc.getDate(), (int)rtc.getMonth());
    f_time = false;
  }
  else if (f_asigned)
  {
    //VALVE es el ID que va del 1 al 250
    uint8_t str_assigned[] = "##ASIGNED#000#000:000:000:000#";
    if (valve > 99)
    {
      str_assigned[10] = '1';
      str_assigned[11] = ((valve - 100) / 10) + 0x30;
      str_assigned[12] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_assigned[10] = '0';
      str_assigned[11] = (valve / 10) + 0x30;
      str_assigned[12] = (valve % 10) + 0x30;
    }
    uint8_t index_assigned = 14;
    for (uint8_t out = 0; out < 4; out++, index_assigned += 4)
    {
      if (assigned[out] > 99)
      {
        str_assigned[index_assigned] = '1';
        str_assigned[index_assigned + 1] = ((assigned[out] - 100) / 10) + 0x30;
        str_assigned[index_assigned + 2] = ((assigned[out] - 100) % 10) + 0x30;
      }
      else
      {
        str_assigned[index_assigned] = '0';
        str_assigned[index_assigned + 1] = (assigned[out] / 10) + 0x30;
        str_assigned[index_assigned + 2] = (assigned[out] % 10) + 0x30;
      }
    }

    for (int j = 0; j < sizeof(str_assigned); j++)
      data[j] = str_assigned[j];
    f_asigned = false;
  }
  else if (f_stop)
  {
    f_stop = false;
    uint8_t str_stop[] = "##STOP#ALL#";
    for (int j = 0; j < sizeof(str_stop); j++)
      data[j] = str_stop[j];
  }
  manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
void rtc_node(int hour, int minute, int second, int day, int month)
{
  uint8_t send[] = "##TIME:H:XX/M:XX/S:XX/D:XX/M:XX/ ";
  send[7 + 2] = (hour / 10) + 0x30;
  send[8 + 2] = (hour % 10) + 0x30;
  send[12 + 2] = (minute / 10) + 0x30;
  send[13 + 2] = (minute % 10) + 0x30;
  send[17 + 2] = (second / 10) + 0x30;
  send[18 + 2] = (second % 10) + 0x30;
  send[22 + 2] = (day / 10) + 0x30;
  send[23 + 2] = (day % 10) + 0x30;
  send[27 + 2] = (month / 10) + 0x30;
  send[28 + 2] = (month % 10) + 0x30;
  for (int i = 0; i < sizeof(send); i++)
    data[i] = send[i];
  for (int i = 0; i < sizeof(send); i++)
    Serial.write(send[i]);
  //manager.sendtoWait(data, sizeof(send), CLIENT_ADDRESS);
}
void write_flash()
{
  for (int j = 0; j < sizeof(UUID_1); j++)
  {
    sys.nodes_uuid[j][0] = UUID_1[j];
    //sys.nodes_uuid[j][1] = UUID_2[j];
    //sys.nodes_uuid[j][2] = UUID_3[j];
    //sys.nodes_uuid[j][3] = UUID_4[j];
    //sys.nodes_uuid[j][4] = UUID_5[j];
    //sys.nodes_uuid[j][5] = UUID_6[j];
    //sys.nodes_uuid[j][6] = UUID_7[j];
  }
  for (int j = 0; j < 7; j++)
  {
    for (int i = 0; i < sizeof(UUID_1); i++)
      Serial.print(sys.nodes_uuid[i][j]);
    Serial.println(" ");
  }
}
void listen_nodo()
{
  Serial.println("");
  Serial.println("He recibido de nodos:");
  //uint8_t print_len = (buf[0] == 'S') ? 8 : 3;
  //print_len = (buf[0] == 'R') ? 14 : 70;
  for (uint8_t i = 0; i < 70; i++) //loop from the buffer looking for the end of message
    Serial.write(buf[i]);
}
/*
  this callback funtion is called when rtc interrupt is triggered
*/
void rtcInt()
{
  intRtc = true; //set flag to indicate that rtc interrupt was triggered
}
/*
  this function is called to execute button action
*/
void buttonHandler()
{
  uint32_t millix;
  DPRINTLN(F("Button pressed"));
  intButton = true;  //set flag to indicate that button pressed was handled
  millix = millis(); //start timer
  while ((!digitalRead(PCINT_PIN)) && ((unsigned long)(millis() - millix) < MAX_TEMP))
    ;                                                        //while button is pressed and does not reach max temp
  if ((unsigned long)(millis() - millix) >= FACTORY_TIMEOUT) //check if button pressed time reach 3s
    setupDevice();                                           //set device to factory state
}
/*
  this function read memmory in a given range of address
*/
void memmoryHandler(uint8_t pos, bool sendChange)
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
      comError[j] = !sendCommand(data, i, sys.oasisRfId[j]);             //send command to child
    }
    checkComError(i);
  }
}
/*
  this function get all data from PG
*/
void getAllFromPG()
{

  bool defined;
  uint8_t i, m, n, v;
  int index, addrInt;
  String aux, addr = "200";
  uint8_t bookEnd = ceil((double)(MAX_CHILD * 4) / 64);

  pgCommand(cmd_write_flag, sizeof(cmd_write_flag)); //clear memmory flag
  getPGTime(true);                                   //read date and time
  memmoryHandler(3, false);                          //read arranque
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
  ;
  digitalWrite(CS_RF, LOW);
}
/*
  this function get PG date and time
*/
void getPGTime(bool readOnly)
{

  int index;
  String aux;
  uint8_t i, h, mn, s, d, m, a, len;

  pgCommand(cmd_read_time, sizeof(cmd_read_time)); //read time from PG
  pg = String(pgData);                             //convert it into string
  index = pg.indexOf(":") + 2;                     //point index to first byte
  aux = pg.substring(index, index + 2);            //get hour
  h = aux.toInt();                                 //convert to integer and save it
  index += 2;                                      //point index to next byte
  aux = pg.substring(index, index + 2);            //get minutes
  mn = aux.toInt();                                //convert to integer and save it
  index += 2;                                      //point index to next byte
  aux = pg.substring(index, index + 2);            //get seconds
  s = aux.toInt();                                 //convert to integer and save it
  pgCommand(cmd_read_date, sizeof(cmd_read_date)); //read date from PG
  pg = String(pgData);                             //convert it into string
  index = pg.indexOf(":") + 2;                     //point index to first byte
  aux = pg.substring(index, index + 2);            //get year
  a = aux.toInt();                                 //convert to integer and save it
  index += 2;                                      //point index to next byte
  aux = pg.substring(index, index + 2);            //get month
  m = aux.toInt();                                 //convert to integer and save it
  index += 2;                                      //point index to next byte
  aux = pg.substring(index, index + 2);            //get day
  d = aux.toInt();                                 //convert to integer and save it
  rtc.setTime(0, s + 10, mn, h, d, m, a, 0);       //set date and time
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  if (readOnly)
    return;
  data[CMD_INDEX] = SET_TIME; //add command id
  data[CMD_INDEX + 1] = 1;    //add number of pending message
  for (i = 0; i < sys.nChild; i++)
  {                                                                          //for each child
    len = jam.fillWithString(data, String(sys.oasisUuid[i]), PAYLOAD_INDEX); //add child uuid
    data[len++] = rtc.getSeconds();                                          //add second
    data[len++] = rtc.getMinutes();                                          //add minutes
    data[len++] = rtc.getHours();                                            //add hour
    data[len++] = rtc.getDate();                                             //add day
    data[len++] = rtc.getMonth();                                            //get month
    data[len++] = rtc.getYear();                                             //add year
    DPRINT("Send data and time to OASIS: ");
    DPRINTLN(i + 1);
    comError[i] = !sendCommand(data, len, sys.oasisRfId[i]); //send new child id
  }
  checkComError(len);
}
bool sendCommand(uint8_t data[], uint8_t len, uint8_t id)
{

  bool ack;
  uint8_t i, awakeMsg[RH_RF95_MAX_MESSAGE_LEN];

  delay(250);
  driver.setPreambleLength(180);
  awakeMsg[CMD_INDEX] = AWAKE;
  awakeMsg[CMD_INDEX + 1] = 1;
  if (data[CMD_INDEX] == PLUG_PLAY)
    i = jam.fillWithString(awakeMsg, String(sys.oasisUuid[sys.nChild - 1]), PAYLOAD_INDEX);
  else
    i = jam.fillWithString(awakeMsg, String(sys.oasisUuid[id - 1]), PAYLOAD_INDEX);
  ack = manager.sendtoWait(awakeMsg, i, id);
  if (ack)
  {
    driver.setPreambleLength(8);
    DPRINTLN("Sent");
    ack = manager.sendtoWait(data, len, id);
  }
  //if (gprs_on)
  //  mqttClient.loop();
  //return ack;
}
/*
  this function check is there
*/
void checkComError(uint8_t len)
{

  String aux;
  uint8_t id, attempts;

  for (id = 0, attempts = 2; id < sys.nChild; id++)
  {
    if (!comError[id])
      continue;
    while (attempts)
    {
      if (comError[id])
      {
        DPRINTLN("Communication to OASIS " + String(id + 1) + " failed, " + String(attempts) + " left. Resend message");
        jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
        comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);
        if (comError[id])
        {
          attempts--;
          if (!attempts)
          {
            DPRINTLN("OASIS " + String(id + 1) + " is not responding");
            aux = (id < 10) ? "0" + String(id) : String(id);
            if (pg.indexOf("PAIRING") != -1)
            {
              cmd_nok[ADDR_INDEX] = aux.charAt(0);
              cmd_nok[ADDR_INDEX + 1] = aux.charAt(1);
              pgCommand(cmd_nok, sizeof(cmd_nok));
            }
            else
            {
              cmd_com_error[ADDR_INDEX] = aux.charAt(0);
              cmd_com_error[ADDR_INDEX + 1] = aux.charAt(1);
              pgCommand(cmd_com_error, sizeof(cmd_com_error));
            }
            comError[id] = false;
          }
        }
        else
        {
          DPRINTLN("Communication to OASIS " + String(id + 1) + " restablished. " + "Message sent with success");
          if (pg.indexOf("PAIRING") != -1)
          {
            cmd_ok[ADDR_INDEX] = aux.charAt(0);
            cmd_ok[ADDR_INDEX + 1] = aux.charAt(1);
            pgCommand(cmd_ok, sizeof(cmd_ok));
          }
          break;
        }
      }
    }
  }
}
/*
  this function send read command to PG and get response
*/
void pgCommand(uint8_t command[], uint8_t len)
{

  uint32_t millix;
  uint8_t i, attempt = 3;
  jam.calcrc((char *)command, len - 2);

  while (attempt)
  {
    //if (gprs_on)
    //  mqttClient.loop();
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
void sleepDevice()
{

  if (gprs_on)
  {
    ;
  }
  else
  {
    digitalWrite(CS_RF, HIGH);
    flash.powerDown();
    digitalWrite(CS_RF, LOW);
    driver.sleep();
    digitalWrite(CS_RF, HIGH);
    //lowPower.sleeper(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    digitalWrite(CS_RF, HIGH);
    flash.powerUp();
    digitalWrite(CS_RF, LOW);
  }
}
/*
  this funtion set device to factory state
*/
void setupDevice()
{

  bool na;
  uint8_t i;
  String str;
  uint16_t b, c;
  uint32_t millix;
  Serial.println(F("Brain Horus  Config/Test 5.27"));
  Serial.println(F("-------- Internal Devices --------"));
  if (strlen(sys.devUuid) != (UUID_LEN - 1))
  {
    //str = jam.getUUID();
    //strcpy(sys.devUuid, str.c_str());
    //sys.id = random(MAX_CHILD + 1, 0xfe);
  }
  /*** Flash Test ***/
  flash.eraseSector(0);
  flash.writeByte(0, 80);
  Serial.print(F("Flash Test:   "));
  i = flash.readByte(0);
  if (i == 80)
    Serial.println(F("Ok"));
  else
  {
    Serial.print(F("Fail"));
    Serial.println(i);
  }
  /*** rtc test ***/
  if (rtc.begin() == false)
    Serial.println(F("RTC Test:     Fail"));
  else
  {
    Serial.println(F("RTC Test:     Ok"));
    rtc.set24Hour();
  }
  if (rtc.updateTime() == false)
  {
    Serial.println("RTC failed to update");
  }
  else
  {
    if (rtc.getYear() == 0)
      Serial.println("First RTC timer and config Charge");
  }
  rtc.setTime(0, 0, 15, 12, 19, 8, 19, 0);
  rtc.updateTime();
  Serial.print(F("Date/Time:    "));
  Serial.print(rtc.stringDate());
  Serial.print(F(" "));
  Serial.println(rtc.stringTime());
  Serial.print(F("Timestamp:    "));
  Serial.println(rtc.getTimestamp());
  rtc.disableInterrupt(INTERRUPT_AIE);
  rtc.clearInterrupts();
  /*** VCC test ***/
  analogReference(INTERNAL);
  for (i = 0; i < 3; i++)
  {
    b = analogRead(PA0);
    delay(1);
  }
  analogReference(DEFAULT);
  for (i = 0; i < 3; i++)
  {
    c = analogRead(PA0);
    delay(1);
  }
  Serial.print(F("Voltaje Vout: "));
  Serial.println(b * 0.0190);
  Serial.print(F("Voltaje Vin:  "));
  Serial.println((b * 2.5132) / c);
  /*** RF test ***/
  if (!driver.init())
    Serial.println(F("RFM95 Lora:   Fail"));
  else
    Serial.println(F("RFM95 Lora:   Ok"));
  Serial.println(F("-------- OASIS-COM --------"));
  sys.id = 0;
  sys.nChild = 0;
  manager.setThisAddress(sys.id);
  digitalWrite(CS_RF, HIGH);
  flash.eraseSector(SYS_VAR_ADDR);
  flash.writeByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t *)&sys, sizeof(sys));
  digitalWrite(CS_RF, LOW);
  Serial.print(F("RF Id: "));
  Serial.println(sys.id);
  Serial.print(F("Number of child: "));
  Serial.println(sys.nChild);
  Serial.print(F("UUID: "));
  Serial.println(sys.devUuid);
  Serial.print(F("Battery: "));
  Serial.println(batLevel());
  jam.ledBlink(LED_SETUP, 3000);
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
      Serial.println(" minutos. ");
      send_nodo(UUID_1, REQUEST_MANVAL, valve_number, valve_time_hour, valve_time_min, asignacion);
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
      send_nodo(UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
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

      send_nodo(UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);

    }
    else if (pg.indexOf("PAIRING#") > 0)
    {
      //I always obtein the number of oasis without one unit due to format 8bit vs 16 bits
      String valve_number = getValue(pg, '#', 1);
      int valve_number_true = (int)strtol(&valve_number[0], NULL, 16);
      Serial.println(valve_number_true);
      String valve_assigned;
      int oasis_valves[4];
      char temp_valve[4];
      for (int k = 0; k < 4; k++)
      {
        valve_assigned = getValue(getValue(pg, '#', 2), ' ', k);
        oasis_valves[k] = (int)strtol(&valve_assigned[0], NULL, 16);
        Serial.println(oasis_valves[k]);
      }
      temp_valve[0] = (char)oasis_valves[0];
      temp_valve[1] = (char)oasis_valves[1];
      temp_valve[2] = (char)oasis_valves[2];
      temp_valve[3] = (char)oasis_valves[3];

      send_nodo(UUID_1, REQUEST_ASSIGNED_VALVES, 11, 0, 0, temp_valve);

    }
    else if (pg.indexOf("SELECTOR#00") > 0)
    {
      Serial.println("AUTO");
      //getAllFromPG();

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
