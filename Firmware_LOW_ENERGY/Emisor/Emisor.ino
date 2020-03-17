#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <JamAtm-Vyrsa.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
/******************************************************************* debug ********************************************************************************************/

#define DEBUG_ON
#ifdef DEBUG_ON
#define DPRINT(...)    Serial.print(__VA_ARGS__)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif
#define RF_RST    27
#define INT_RF    2
#define INT_RTC   14
#define CS_M      22
#define CS_RF     23
#define VREF_IN   24
#define VREF_EXT  29
#define PG_RXD    16
#define PG_TXD    17
#define SIM_PWR   26
#define SIM_AWK   21
#define LED_SETUP 3
#define PCINT_PIN 18
#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))
/******************************************************************* declarations  ************************************************************************************/

typedef struct {
  uint8_t id;
  uint8_t nChild;
  char devUuid[UUID_LEN];
  uint8_t oasisRfId[MAX_CHILD];
  char oasisUuid[MAX_CHILD][UUID_LEN];
  uint8_t childValves[MAX_CHILD][4];
} sysVar;

typedef struct {
  uint8_t interval;
  uint8_t startDay;
  uint8_t wateringDay;
  uint16_t waterPercent;
  uint8_t start[TOTAL_START][2];
  uint8_t irrigTime[TOTAL_VALVE];
} program;

Jam jam;
sysVar sys;
RV1805 rtc;
Sleep lowPower;
SimpleTimer timer;
SPIFlash flash(CS_M);
program prog[TOTAL_PROG];
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver);
SoftwareSerial softSerial(PG_RXD, PG_TXD);

const char apn[]  = "wlapn.com";
const char user[] = "vyr";
const char pass[] = "vyr";
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

String pg;
int timer_1;
char pgData[PG_MAX_LEN];
uint8_t i, j, rfId, cmd;
volatile uint8_t oldPort = 0x00;
volatile bool intButton = false, intRtc = false;
bool gprs_on = true, mode = false, comError[MAX_CHILD];
uint8_t valveDef[MAX_CHILD], progDef[TOTAL_PROG], data[RH_RF95_MAX_MESSAGE_LEN];

/***** functions Prototypes ******/
void pAndpLed();
void rfHandler();
void pgHandler();
void initDevice();
void connectSIM();
void toggleMode();
void setupDevice();
uint8_t batLevel();
void sleepDevice();
void getFirmwVer();
void connectMqtt();
void mqttHandler();
void buttonHandler();
void executeCommand();
void checkComError(uint8_t len);
void getPGTime(bool readOnly = true);
bool newChild(String dst, uint8_t id);
void pgCommand(uint8_t command[], uint8_t len);
void sendToApp(uint8_t cmd, uint8_t param = 0);
bool sendCommand(uint8_t data[], uint8_t len, uint8_t id);
void memmoryHandler(uint8_t pos, bool sendChange = true);
void mqttCallback(char* topic, byte* payload, unsigned int length);
/******************************************************************* setup section ************************************************************************************/
void setup() {

  initDevice();
}

/******************************************************************* main program  ************************************************************************************/
void loop() {

  if (!digitalRead(PCINT_PIN)) {
    if (!intButton)
      buttonHandler();
    else
      intButton = false;
  }
  if (intRtc) {
    intRtc = false;
    rtcHandle();
  }
  timer.run();
  if (mode)
    rfHandler();
  else
    sleepDevice();
  pgHandler();
  mqttHandler();
}

/*******************************************************************   functions     ************************************************************************************/
/*
  this function initializes hardware resources
*/
void initDevice() {

#ifdef DEBUG_ON
  Serial.begin(115200);
#endif
  delay(250);
  getFirmwVer();
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
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));
  flash.readByteArray(PROG_VAR_ADDR, (uint8_t*)&prog, sizeof(prog));
  DPRINT(", UUID: ");
  DPRINTLN(sys.devUuid);
  manager.setThisAddress(sys.id);
  manager.init();
  manager.setRetries(10);
  manager.setTimeout(175);
  driver.setTxPower(TX_PWR, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.setAlarmMode(4);
  rtc.setAlarm(0, 30, 0, 0, 0);
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  getAllFromPG();
  connectSIM();
  connectMqtt();
  sendToApp(CONNECT);
  for (i = 0; i < MAX_CHILD; i++) comError[i] = false;
  jam.ledBlink(LED_SETUP, 1000);
}

/*
  this function blink led to indicate that it is in plug&play mode
*/
void pAndpLed() {

  jam.ledBlink(LED_SETUP, 100);                                                           //blink led to indicate plug and play mode
}

/*
  this callback funtion is called when rtc interrupt is triggered
*/
void rtcInt() {

  intRtc = true;                                                                          //set flag to indicate that rtc interrupt was triggered
}

/*
  this function is called to execute button action
*/
void buttonHandler() {

  uint32_t millix;
  DPRINTLN(F("Button pressed"));
  intButton = true;                                                                       //set flag to indicate that button pressed was handled
  millix = millis();                                                                      //start timer
  while ((!digitalRead(PCINT_PIN)) && ((unsigned long)(millis() - millix) < MAX_TEMP));   //while button is pressed and does not reach max temp
  if ((unsigned long)(millis() - millix) >= FACTORY_TIMEOUT)                              //check if button pressed time reach 3s
    setupDevice();                                                                        //set device to factory state
  else                                                                                    //otherwise
    toggleMode();                                                                         //toggle to plug&play or run mode
}

/*
  this function toggle device to plug&play or run mode
*/
void toggleMode() {

  mode = !mode;
  if (mode) {                                                                             //if plug&play mode
    manager.setThisAddress(0);                                                            //set rf id to 0
    timer_1 = timer.setInterval(5000, pAndpLed);                                          //set timer to blink led every 5 seconds to indicate that it is in p&p mode
  }
  else {                                                                                  //otherwise it is in run mode
    timer.deleteTimer(timer_1);                                                           //delete led timer
    manager.setThisAddress(sys.id);                                                       //set run mode rf id
  }
}

/*
  this function handles rf messages
*/
void rfHandler() {

  uint8_t dataSize;
  if (manager.available()) {                                                              //check if there is any rf message available
    dataSize = RH_RF95_MAX_MESSAGE_LEN;                                                   //set rf data size
    if (manager.recvfromAck(data, &dataSize, &rfId))                                      //if data is into buffer
      executeCommand();                                                                   //execute command
  }
}

/*
  this callback funtion is called when rtc interruption is triggered
*/
void rtcHandle() {

  uint8_t h, m;
  DPRINTLN("Int rtc");
  byte rtcStatus = rtc.status();                                                          //get rtc status
  if (rtcStatus != 0) {                                                                   //if any interrupt was triggered
    if (rtcStatus & (1 << STATUS_AF))                                                     //if it is alarm interrupt
      getPGTime(false);                                                                   //get PG time and send to all oasis
  }
}

/*
  this function handles pg interruptions
*/
void pgHandler() {

  int index;
  String aux;
  uint32_t millix;
  bool intPg = false;
  uint8_t i, id, b1, b2, v, len;

  i = 0;                                                                                  //initialize index
  millix = millis();                                                                      //start timer
  while ((unsigned long)(millis() - millix) < 100) {                                      //while timer doesn't reach 100 ms
    while (softSerial.available()) {                                                      //while there is data to read
      pgData[i++] = softSerial.read();                                                    //read byte and save it
      millix = millis();
    }
  }
  if (i > 10)
    intPg = true;

  if (intPg) {                                                                            //if int pg was triggered
    intPg = false;                                                                        //clear int pg flag
    pgData[i] = '\0';                                                                     //add end of char
    pg = String(pgData);                                                                  //convert message received into string
    DPRINT("PG: ");
    DPRINTLN(pg.c_str());
    if ((index = pg.indexOf("PAIRING")) != -1) {                                          //if it is pairing message interrupt
      index = pg.indexOf("#", index) + 1;                                                 //point index to oasis id
      aux = pg.substring(index, index + 2);                                               //get oasis id
      id = aux.toInt();                                                                   //save it
      if ((id + 1) > sys.nChild) return;                                                  //if oasis does not exist, does not execute command
      len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);           //add oasis uuid to data
      for (i = 0, index += 3; i < TOTAL_OUTPUT; i++, len++) {                             //while all 4 valves was not saved
        aux = pg.substring(pg.indexOf(" ", index), index);                                //get valve number
        index += aux.length() + 1;                                                        //update index
        (aux != "FF") ? data[len] = strtol(aux.c_str(), NULL, HEX) + 1 : data[len] = 0;   //if output is assigned get valve value, otherwise set it to 0
        sys.childValves[id][i] = data[len];                                               //save valve assignment to system variable
      }
      DPRINT("Oasis: ");
      DPRINTLN(id + 1);
      for (i = 0; i < TOTAL_OUTPUT; i++) {
        DPRINT("salida " + String(i + 1) + ": ");
        DPRINTLN(sys.childValves[id][i]);
      }
      sendToApp(ASSIGN_VALVES, id);
      digitalWrite(CS_RF, HIGH);                                                          //unselect rf
      flash.eraseSector(SYS_VAR_ADDR);                                                    //erase sys variable sector
      flash.writeByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));;                   //write sys variable changes
      digitalWrite(CS_RF, LOW);                                                           //select again rf
      data[CMD_INDEX] = ASSIGN_VALVES;                                                    //set command id
      comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);                          //send command to oasis
      if (!comError[id]) {
        aux = (id < 10) ? "0" + String(id) : String(id);
        cmd_ok[ADDR_INDEX] = aux.charAt(0);
        cmd_ok[ADDR_INDEX + 1] = aux.charAt(1);
        pgCommand(cmd_com_error, sizeof(cmd_ok));
      }
      checkComError(len);
    }
    else if ((index = pg.indexOf("MANVALV")) != -1) {                                     //if it is manual valve message interrupt
      index = pg.indexOf("#", index) + 1;                                                 //point index to valve
      aux = pg.substring(index, index + 2);                                               //get valve
      v = strtol(aux.c_str(), NULL, HEX);                                                 //convert valve it to integer
      aux = pg.substring(index + 3, index + 5);
      data[CMD_INDEX + 3] = aux.toInt();
      aux = pg.substring(index + 5, index + 7);
      data[CMD_INDEX + 4] = aux.toInt();
      sendToApp(MANUAL_VALVE, v);
      for (id = 0; id < sys.nChild; id++) {                                               //for each oasis
        for (i = 0; i < TOTAL_OUTPUT; i++){                                               //go through each valve assigned
          if (sys.childValves[id][i] == v){                                               //if its valve is equalt to v
            data[CMD_INDEX] = MANUAL_VALVE;                                               //set command id
            if (pg.indexOf("START") != -1)                                                //if manual action is start
              data[CMD_INDEX + 1] = 1;                                                    //set command id to start valve
            else                                                                          //otherwise
              data[CMD_INDEX + 1] = 0;                                                    //set command id to stop valve
            data[CMD_INDEX + 2] = v;                                                      //add valve number
            len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);     //add oasis uuid to data
            comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);                    //send command
            DPRINTLN("Oasis: " + String(id + 1));
            DPRINTLN("Valve: " + String(v));
            DPRINTLN("Action: " + String(data[CMD_INDEX + 1]));
            if (pg.indexOf("START") != -1)
              DPRINTLN("Time: " + String(data[CMD_INDEX + 3]) + ":" + String(data[CMD_INDEX + 4]));
            break;
          }
        }
      }
      checkComError(len);
    }
    else if ((index = pg.indexOf("MANPRG")) != -1) {                                      //if it is a manual program message interrupt
      index = pg.indexOf("#", index) + 1;                                                 //point index to program letter
      aux = pg.substring(index, index + 1);                                               //get program letter
      data[CMD_INDEX] = MANPRG;                                                           //set command id
      if (pg.indexOf("START") != -1)                                                      //if manual action is start
        data[CMD_INDEX + 1] = 1;                                                          //set command id to start valve
      else                                                                                //otherwise
        data[CMD_INDEX + 1] = 0;                                                          //set command id to stop valve
      data[CMD_INDEX + 2] = aux.charAt(0) - 'A';
      sendToApp(MANPRG);
      for (id = 0; id < sys.nChild; id++) {                                               //for each oasis
        DPRINT("Send MANPRG to OASIS: ");
        DPRINTLN(id + 1);
        len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);         //add oasis uuid to data
        comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);                        //send command
      }
      checkComError(len);
    }
    else if ((index = pg.indexOf("SET TIME")) != -1)                                      //if it is set command message interrupt
      getPGTime(false);                                                                   //get new date and time from PG and send to all oasis
    else if (pg.indexOf("MEMMORY") != -1) {                                               //if it is memmory message interrupt
      pgCommand(cmd_read_flag, sizeof(cmd_read_flag));                                    //read memmory flag
      pg = String(&pgData[2]);                                                            //save it into string
      index = pg.indexOf("#") + 1;                                                        //point index to memmory bytes
      aux =  pg.substring(index, index + 2);                                              //get second byte
      b2 =  strtol(aux.c_str(), NULL, HEX);                                               //convert to integer and save it
      aux = pg.substring(index + 3, index + 5);                                           //get first byte
      b1 =  strtol(aux.c_str(), NULL, HEX);                                               //convert to integer and save it
      for (i = 0; i < 8; i++) {                                                           //for each bit
        if (bitRead(b1, i) == 1)                                                          //if bit of first byte is 1
          memmoryHandler(i);                                                              //call memmory handler function
        if (bitRead(b2, i) == 1)                                                          //if bit of second byte is 1
          memmoryHandler(i + 8);                                                          //call memmory handler function
      }
      pgCommand(cmd_write_flag, sizeof(cmd_write_flag));
    }
    else if (pg.indexOf("STOP") != -1) {
      data[CMD_INDEX] = STOP_ALL;
      for (id = 0; id < sys.nChild; id++) {
        DPRINT("Send STOP to OASIS: ");
        DPRINTLN(id + 1);
        len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
        comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);
      }
      checkComError(len);
    }
    else if ((index = pg.indexOf("SELECTOR")) != -1) {
      index = pg.indexOf("#", index) + 1;
      aux = pg.substring(index, index + 2);
      if (aux.toInt() == 0) {
        data[CMD_INDEX] = SELECTOR;
        data[CMD_INDEX + 1] = aux.toInt();
        for (id = 0; id < sys.nChild; id++) {
          DPRINT("Send SELECTOR to OASIS: ");
          DPRINTLN(id + 1);
          len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
          comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);
        }
      }
      checkComError(len);
    }
    delay(5);
  }
}

/*
  this function read memmory in a given range of address
*/
void memmoryHandler(uint8_t pos, bool sendChange) {

  int index;
  String str, aux;
  uint8_t p, a, v, m, ptr;
  i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;

  if (pos == 1) {                                                                         //if bit 1 is 1
    cmd_read_book[ADDR_INDEX] = '0';                                                      //set address to read
    cmd_read_book[ADDR_INDEX + 1] = '9';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_book, sizeof(cmd_read_book));                                      //send read page command to pg
    pg = String(pgData);                                                                  //convert it into string
    index = pg.indexOf("#") + 1;                                                          //point index to first byte read
    str =  pg.substring(index, index + APORTE_AGUA_LEN);                                  //get aporte de agua bytes
    DPRINTLN(F("***** Aporte de agua *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1, i++) {             //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr);                                    //get first byte of aporte de agua
      data[i] = strtol(aux.c_str(), NULL, HEX);                                           //convert it and save into data
      prog[p].waterPercent = data[i] * 256;                                               //convert value to 2 byte integer
      ptr += aux.length() + 1;                                                            //point index to next byte
      aux = str.substring(str.indexOf(" ", ptr), ptr);                                    //get second byte of aporte de agua
      data[++i] = strtol(aux.c_str(), NULL, HEX);                                         //convert it and save into data
      prog[p].waterPercent += data[i];                                                    //convert it in 2 byte integer and save it

      DPRINT(F("Prog "));
      DPRINT((char)(p + 'A'));
      DPRINTLN(": " + String(prog[p].waterPercent));
    }
    index += str.length() + 1;                                                            //point index to the next range of bytes
    str = pg.substring(index, index + INTERV_INIT_LEN);                                   //get interval and start day bytes
    DPRINTLN(F("***** Intervalo y Start day *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1) {                  //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr);                                    //get interval byte
      data[i++] = prog[p].interval = strtol(aux.c_str(), NULL, HEX);                      //convert into integer and save it
      ptr += aux.length() + 1;                                                            //point index to next byte
      aux = str.substring(str.indexOf(" ", ptr), ptr);                                    //get start day byte
      data[i++] = prog[p].startDay = strtol(aux.c_str(), NULL, HEX);                      //convert into integer and save it
      DPRINT(F("*Prog "));
      DPRINTLN((char) (p + 'A'));
      DPRINT(F("  interval: "));
      DPRINTLN(prog[p].interval);
      DPRINT(F("  startDay: "));
      DPRINTLN(prog[p].startDay);
    }
    index += str.length() + 1;                                                            //point index to next range of byte
    str = pg.substring(index, index + WATERING_LEN);                                      //get watering days
    DPRINTLN(F("***** watering day *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, ptr += aux.length() + 1) {                  //for each program
      aux = str.substring(str.indexOf(" ", ptr), ptr);                                    //get watering day byte
      data[i++] = prog[p].wateringDay = strtol(aux.c_str(), NULL, HEX);                   //convert into integer and save it
      DPRINT(F("Prog "));
      DPRINT((char)(p + 'A'));
      DPRINTLN(": " + String(prog[p].wateringDay));
    }
  }
  else if (pos == 3) {                                                                    //if bit 3 is 1
    cmd_read_book[ADDR_INDEX] = '1';                                                      //set address to read
    cmd_read_book[ADDR_INDEX + 1] = 'A';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_book, sizeof(cmd_read_book));                                      //read book
    pg = String(pgData);                                                                  //convert it into string
    index = pg.indexOf("#") + 1;                                                          //point index to first byte read
    pg = pg.substring(index, index + BOOK_LEN);                                           //get all book bytes
    cmd_read_line[ADDR_INDEX] = '1';                                                      //set adrees to read remaining bytes
    cmd_read_line[ADDR_INDEX + 1] = 'E';
    cmd_read_line[ADDR_INDEX + 2] = '0';
    pgCommand(cmd_read_line, sizeof(cmd_read_line));                                      //read line
    aux = String(pgData);                                                                 //convert it into string
    index = aux.indexOf("#") + 1;                                                         //point index to first byte read
    aux = aux.substring(index, index + 23);                                               //get 8 byte from bytes read
    pg = pg + " " + aux;                                                                  //add those 8 byte to pg string
    index = 0;
    DPRINTLN(F("***** Arranques *****"));
    for (p = 0, ptr = 0; p < TOTAL_PROG; p++, index += str.length() + 1, ptr = 0) {       //for each program
      str = pg.substring(index, index + ARRANQUE_LEN);                                    //get string with arranque of its program
      for (a = 0; a < TOTAL_START; a++, ptr += aux.length() + 1) {                        //for each arraque
        aux = str.substring(str.indexOf(" ", ptr), ptr);                                  //get hour
        data[i++] = prog[p].start[a][0] = strtol(aux.c_str(), NULL, HEX);                 //save it
        ptr += aux.length() + 1;                                                          //point index to next value
        aux = str.substring(str.indexOf(" ", ptr), ptr);                                  //get min
        data[i++] = prog[p].start[a][1] = strtol(aux.c_str(), NULL, HEX);                 //save it
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
  else if (pos > 7) {                                                                     //if one of the bit of second byte is 1
    p = pos - 8;                                                                          //define program
    cmd_read_book[ADDR_INDEX] = (pos / 2) + '0';                                          //set address to read
    (pos % 2) ? cmd_read_book[ADDR_INDEX + 1] = '8' : cmd_read_book[ADDR_INDEX + 1] = '0';
    cmd_read_book[ADDR_INDEX + 2] = '0';
    DPRINTLN(F("***** Tiempos de riego *****"));
    DPRINT(F("Prog "));
    DPRINT((char)(p + 'A'));
    DPRINT(F(": "));
    for (v = 0, a = 0; a < 2; a++) {                                                      //read 64 bytes of irrigation time two times, 128
      pgCommand(cmd_read_book, sizeof(cmd_read_book));                                    //read first 64 valve irrigtation time
      pg = String(pgData);                                                                //convert it into string
      index = pg.indexOf("#") + 1;                                                        //point index to firt byte read
      str = pg.substring(index, index + IRRIG_TIME_LEN);
      for (m = 0, ptr = 0; m < 64; v++, m++, ptr += aux.length() + 1) {                   //for each valve
        aux = str.substring(str.indexOf(" ", ptr), ptr);                                  //get value
        data[i++] = prog[p].irrigTime[v] = strtol(aux.c_str(), NULL, HEX);                //convert it to integer and save it
        DPRINT("v" + String(v + 1) + String(":"));
        DPRINT(prog[p].irrigTime[v]);
        DPRINT(" ");
      }
      if (cmd_read_book[ADDR_INDEX + 1] == '0')                                           //set address to the next 64 bytes
        cmd_read_book[ADDR_INDEX + 1] = '4';
      else
        cmd_read_book[ADDR_INDEX + 1] = 'C';
    }
    DPRINTLN();
  }
  digitalWrite(CS_RF, HIGH);                                                              //unselect rf
  flash.eraseSector(PROG_VAR_ADDR);                                                       //erase program variable sector
  flash.writeByteArray(PROG_VAR_ADDR, (uint8_t*)&prog, sizeof(prog));;                    //write program variable changes
  digitalWrite(CS_RF, LOW);                                                               //select again rf
  if (sendChange) {
    data[CMD_INDEX + 1] = pos;                                                            //set command parameter
    data[CMD_INDEX] = MEMMORY;                                                            //set comand id
    for (j = 0; j < sys.nChild; j++) {                                                    //for each child
      DPRINT("Send change to OASIS: ");
      DPRINTLN(j + 1);
      jam.fillWithString(data, String(sys.oasisUuid[j]), PAYLOAD_INDEX);                  //add child uuid to data
      comError[j] = !sendCommand(data, i, sys.oasisRfId[j]);                              //send command to child
    }
    checkComError(i);
  }
}

/*
  this function get all data from PG
*/
void getAllFromPG() {

  bool defined;
  uint8_t i, m, n, v;
  int index, addrInt;
  String aux, addr = "200";
  uint8_t bookEnd = ceil((double)(MAX_CHILD * 4) / 64);

  pgCommand(cmd_write_flag, sizeof(cmd_write_flag));                                      //clear memmory flag
  getPGTime(true);                                                                        //read date and time
  memmoryHandler(3, false);                                                               //read arranque
  for (i = 0; i < TOTAL_PROG; i++) {                                                      //for each program
    progDef[i] =  0;                                                                  //init flag as prog not defined
    for (j = 0; j < TOTAL_START; j++) {                                                   //for each arranque
      if ((prog[i].start[j][0] != 0xff) && (prog[i].start[j][1] != 0xff)) {               //if arranque at least one arranque is defined
        progDef[i] = 1;                                                                //set defined flag as true
        break;                                                                            //leave loop
      }
    }
    if (progDef[i]) {                                                                     //if al least one arranque is defined
      memmoryHandler(i + 8, false);                                                       //read tiempo de riego
      progDef[i] = 0;                                                                 //initialize defined flag to false
      for (v = 0; v < TOTAL_VALVE; v++) {                                                 //for each valve
        if (prog[i].irrigTime[v] != 0xff) {                                               //if at least one tiempo de riego is defined
          progDef[i] = 1;                                                              //set defined flag as true
          break;                                                                          //leave loop
        }
      }
    }
    if (progDef[i])                                                                       //if at least one arranque and one tiempo de riego is defined
      memmoryHandler(1, false);                                                           //read aporte de agua, interval, starday and wateringday
  }

  for (i = 0, m = 0, valveDef[m] = 0; i < bookEnd; i++) {
    cmd_read_book[ADDR_INDEX] = addr.charAt(0);
    cmd_read_book[ADDR_INDEX + 1] = addr.charAt(1);
    cmd_read_book[ADDR_INDEX + 2] = addr.charAt(2);
    pgCommand(cmd_read_book, sizeof(cmd_read_book));
    pg = String(pgData);
    index = pg.indexOf("#") + 1;
    pg = pg.substring(index, index + BOOK_LEN);
    DPRINT("Assignment: ");
    DPRINTLN(pg.c_str());
    for (n = 0, index = 0; index < pg.length();) {
      aux = pg.substring(index, pg.indexOf(" ", index));
      if (aux != "FF") {
        sys.childValves[m][n] = strtol(aux.c_str(), NULL, HEX) + 1;
        valveDef[m] = 1;
      }
      else
        sys.childValves[m][n] = 0;
      index += aux.length() + 1;
      if (n == 3) {
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
  flash.writeByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));;
  digitalWrite(CS_RF, LOW);
}


/*
  this function get PG date and time
*/
void getPGTime(bool readOnly) {

  int index;
  String aux;
  uint8_t i, h, mn, s, d, m, a, len;

  pgCommand(cmd_read_time, sizeof(cmd_read_time));                                        //read time from PG
  pg = String(pgData);                                                                    //convert it into string
  index = pg.indexOf(":") + 2;                                                            //point index to first byte
  aux = pg.substring(index, index + 2);                                                   //get hour
  h = aux.toInt();                                                                        //convert to integer and save it
  index += 2;                                                                             //point index to next byte
  aux = pg.substring(index, index + 2);                                                   //get minutes
  mn = aux.toInt();                                                                       //convert to integer and save it
  index += 2;                                                                             //point index to next byte
  aux = pg.substring(index, index + 2);                                                   //get seconds
  s = aux.toInt();                                                                        //convert to integer and save it
  pgCommand(cmd_read_date, sizeof(cmd_read_date));                                        //read date from PG
  pg = String(pgData);                                                                    //convert it into string
  index = pg.indexOf(":") + 2;                                                            //point index to first byte
  aux = pg.substring(index, index + 2);                                                   //get year
  a = aux.toInt();                                                                        //convert to integer and save it
  index += 2;                                                                             //point index to next byte
  aux = pg.substring(index, index + 2);                                                   //get month
  m = aux.toInt();                                                                        //convert to integer and save it
  index += 2;                                                                             //point index to next byte
  aux = pg.substring(index, index + 2);                                                   //get day
  d = aux.toInt();                                                                        //convert to integer and save it
  rtc.setTime(0, s + 10, mn, h, d, m, a, 0);                                              //set date and time
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  if (readOnly) return;
  data[CMD_INDEX] = SET_TIME;                                                             //add command id
  data[CMD_INDEX + 1] = 1;                                                                //add number of pending message
  for (i = 0; i < sys.nChild; i++) {                                                      //for each child
    len = jam.fillWithString(data, String(sys.oasisUuid[i]), PAYLOAD_INDEX);              //add child uuid
    data[len++] = rtc.getSeconds();                                                       //add second
    data[len++] = rtc.getMinutes();                                                       //add minutes
    data[len++] = rtc.getHours();                                                         //add hour
    data[len++] = rtc.getDate();                                                          //add day
    data[len++] = rtc.getMonth();                                                         //get month
    data[len++] = rtc.getYear();                                                          //add year
    DPRINT("Send data and time to OASIS: ");
    DPRINTLN(i + 1);
    comError[i] = !sendCommand(data, len, sys.oasisRfId[i]);                              //send new child id
  }
  checkComError(len);
}

/*
  this function sends all PG data to Oasis
*/
void sendPGtoOasis(uint8_t toOasis) {

  uint8_t i, len, id, j;

  id = toOasis;
  for (i = 0; i < TOTAL_PROG; i++) {                                                      //for each program
    if (progDef[i]) {
      len = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;                                      //initialize data index
      for (j = 0; j < TOTAL_VALVE; j++, len++)                                            //for each valve save
        data[len] = prog[i].irrigTime[j];                                                 //save tiempo de riego into data;
      for (j = 0; j < TOTAL_START; j++, len++) {                                          //for each arranque
        data[len++] = prog[i].start[j][0];                                                //save hora arranque into data
        data[len] = prog[i].start[j][1];                                                  //save minutes arranque into data
      }
      data[len++] = prog[i].waterPercent >> 8;                                            //save first byte of waterPercent into data
      data[len++] = prog[i].waterPercent & 0xff;                                          //save second byte of waterPerent into data
      data[len++] = prog[i].interval;                                                     //save interval into data
      data[len++] = prog[i].startDay;                                                     //save startDay into data
      data[len++] = prog[i].wateringDay;                                                  //save watering dar inyo data
      data[CMD_INDEX] = PROGRAM;                                                          //set comand id
      data[CMD_INDEX + 1] = i;                                                            //set program id
      DPRINT("Send program to OASIS: ");
      DPRINTLN(id + 1);
      jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);                 //add child uuid to data
      comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);                          //send program to child
      checkComError(len);
    }
  }
  if (valveDef[id]) {
    len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
    DPRINT("Send vale assignmet to Oasis: ");
    DPRINTLN(id + 1);
    for (i = 0; i < 4; i++, len++) {
      data[len] = sys.childValves[id][i];
      DPRINT("salida " + String(i + 1) + ": ");
      DPRINTLN(sys.childValves[id][i]);
    }
    data[CMD_INDEX] = ASSIGN_VALVES;
    comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);
    DPRINTLN("*****");
  }
  checkComError(len);
}

bool sendCommand(uint8_t data[], uint8_t len, uint8_t id) {

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
  if (ack) {
    driver.setPreambleLength(8);
    DPRINTLN("Sent");
    ack = manager.sendtoWait(data, len, id);
  }
  if (gprs_on) mqttClient.loop();
  return ack;
}


/*
  this function check is there
*/
void checkComError(uint8_t len) {

  String aux;
  uint8_t id, attempts;

  for (id = 0, attempts = 2; id < sys.nChild; id++) {
    if (!comError[id]) continue;
    while (attempts) {
      if (comError[id]) {
        DPRINTLN("Communication to OASIS " + String(id + 1) + " failed, "
                 + String(attempts) + " left. Resend message");
        jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
        comError[id] = !sendCommand(data, len, sys.oasisRfId[id]);
        if (comError[id]) {
          attempts--;
          if (!attempts) {
            DPRINTLN("OASIS " + String(id + 1) + " is not responding");
            aux = (id < 10) ? "0" + String(id) : String(id);
            if (pg.indexOf("PAIRING") != -1) {
              cmd_nok[ADDR_INDEX] = aux.charAt(0);
              cmd_nok[ADDR_INDEX + 1] = aux.charAt(1);
              pgCommand(cmd_nok, sizeof(cmd_nok));
            }
            else {
              cmd_com_error[ADDR_INDEX] = aux.charAt(0);
              cmd_com_error[ADDR_INDEX + 1] = aux.charAt(1);
              pgCommand(cmd_com_error, sizeof(cmd_com_error));
            }
            comError[id] = false;
          }
        }
        else {
          DPRINTLN("Communication to OASIS " + String(id + 1) + " restablished. "
                   + "Message sent with success");
          if (pg.indexOf("PAIRING") != -1) {
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
  this function execute command received
*/
void executeCommand() {

  String from;
  uint8_t len, id, cmd;

  cmd = data[CMD_INDEX];                                                                  //get command id
  switch (cmd) {
    case PLUG_PLAY:
      DPRINTLN(F("Plug&Play command"));
      from = jam.getDst(data);                                                            //get oasis uuid
      if (newChild(from, rfId)) {                                                         //check if it a new oasis
        id = sys.nChild;                                                                  //if so, set oasis id
        strcpy(sys.oasisUuid[id], from.c_str());                                          //save oasis uuid
        sys.oasisRfId[id] = id + 1;                                                       //assign new rf id to this oasis
        sys.nChild++;                                                                     //increase oasis number
        data[CMD_INDEX + 1] = sys.nChild;                                                 //add new child id assignation to data
        rtc.updateTime();                                                                 //get rtc data
        DPRINT(rtc.stringDate());
        DPRINT(" ");
        DPRINTLN(rtc.stringTime());
        i = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);           //add device uuid
        data[i++] = rtc.getSeconds();                                                     //add year to data
        data[i++] = rtc.getMinutes();                                                     //add month to data
        data[i++] = rtc.getHours();                                                       //add day of month to data
        data[i++] = rtc.getDate();                                                        //add hour to data
        data[i++] = rtc.getMonth();                                                       //get minutes to data
        data[i++] = rtc.getYear();                                                        //add seconds to data
        sendCommand(data, i, rfId);                                                       //send new child d
        digitalWrite(CS_RF, HIGH);                                                        //unselect rf
        flash.eraseSector(SYS_VAR_ADDR);                                                  //erase sys variable sector
        flash.writeByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));                  //write sys variable changes
        digitalWrite(CS_RF, LOW);                                                         //select again rf
        DPRINT(F("Child id: "));
        DPRINTLN(sys.oasisRfId[id]);
        DPRINT(F("Child uuid: "));
        DPRINTLN(sys.oasisUuid[id]);
        sendPGtoOasis(id);
        sendToApp(PLUG_PLAY, id);
        jam.ledBlink(LED_SETUP, 1000);
      }
      break;
  }
}

/*
  this function check if child is new, return true is so and false otherwise
*/
bool newChild(String dst, uint8_t id) {

  uint8_t i;
  for (i = 0; i < sys.nChild; i++) {                                                      //for each child
    if (dst == String(sys.oasisUuid[i]))                                                  //check if give rf id and uuid already existe
      return false;                                                                       //if so return false
  }
  return true;                                                                            //means child was not found and return true
}


/*
  this function send read command to PG and get response
*/
void pgCommand(uint8_t command[], uint8_t len) {

  uint32_t millix;
  uint8_t i, attempt = 3;
  jam.calcrc((char*)command, len - 2);

  while (attempt) {
    if (gprs_on) mqttClient.loop();
    softSerial.write(command, len);                                                       //send command to PG
    millix = millis();                                                                    //start millis count
    while ((millis() - millix) < PG_TIMEOUT);                                             //wait one second
    i = 0;                                                                                //initialize pgdata index
    while (softSerial.available())                                                        //while bytes available in serial port
      pgData[i++] = softSerial.read();                                                    //read it
    if ((i == ACK_SIZE) && (pgData[2] == 'N'))                                            //if is a not acknowledge message
      attempt--;                                                                          //try again                                                                              //means sucess replay
    else                                                                                  //otherwise
      break;
  }
  pgData[i - 1] = '\0';
}

/*
  this function connects to internet through sim card
*/
void connectSIM() {

  int signalq;
  String modemInfo;

  delay(500);
  pinMode(SIM_PWR, OUTPUT);
  digitalWrite(SIM_PWR, LOW);
  delay(1200);
  DPRINTLN("Initializing modem...");
  digitalWrite(SIM_PWR, HIGH);
  delay(1200);
  Serial1.begin(57600);
  modem.restart();

  modemInfo = modem.getModemInfo();
  if (modemInfo.indexOf("SIM800") == -1) {
    Serial1.end();
    digitalWrite(SIM_PWR, LOW);
    delay(1200);
    digitalWrite(SIM_PWR, HIGH);
    delay(1200);
    Serial1.begin(57600);
    modem.restart();
    modemInfo = modem.getModemInfo();
  }
  if (modemInfo == "") {
    DPRINTLN("GPRS is disabled!");
    gprs_on = false;
    return;
  }
  DPRINT("Modem: ");
  DPRINTLN(modemInfo);
  Serial1.print("AT+IPR=115200");
  modem.init();
  Serial1.begin(115200);
  delay(2000);
  Serial1.print("AT+IPR=115200");
  //modem.simUnlock("5994");

  DPRINT("Waiting for network...");
  if (!modem.waitForNetwork()) {
    DPRINTLN(" fail");
    DPRINTLN("GPRS is disabled!");
    gprs_on = false;
    return;
  }
  DPRINTLN(" succeed");

  DPRINT("Connecting to ");
  DPRINT(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    DPRINTLN("GPRS is disabled!");
    gprs_on = false;
    return;
  }
  DPRINTLN(" succeed");
  signalq = modem.getSignalQuality();
  DPRINTLN("Signal quality: " +  String(signalq));
}

/*
  this function connects to device to softeca broker
*/
void connectMqtt() {

  String topic , clientId;

  if (!gprs_on) return;
  clientId = "Jam-Idea" + modem.getIMEI();
  topic = String(sys.devUuid) + "/+/app";
  mqttClient.setServer("mqtt.pre.hydro-plus.es", 1883);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected()) {
    if (mqttClient.connect(clientId.c_str(), "hydroplus", "vyrsa"))
      mqttClient.subscribe(topic.c_str());
    else {
      DPRINTLN("Trying to connect to MQTT...");
      delay(1000);
    }
  }
  DPRINTLN("Successfully connected to MQTT");
  topic = String(sys.devUuid) + "/oasis/app";
  mqttClient.publish(topic.c_str(), (const uint8_t*)"", 0, true);
  topic = String(sys.devUuid) + "/manvalve/app";
  mqttClient.publish(topic.c_str(), (const uint8_t*)"", 0, true);
  topic = String(sys.devUuid) + "/manprg/app";
  mqttClient.publish(topic.c_str(), (const uint8_t*)"", 0, true);
  topic = String(sys.devUuid) + "/program/app";
  mqttClient.publish(topic.c_str(), (const uint8_t*)"", 0, true);
}

/*
  this function handler mqtt connection
*/
void mqttHandler() {

  if (!gprs_on) return;
  mqttClient.loop();
  if (!mqttClient.connected()) {
    DPRINTLN("Mqtt connection fail");
    connectMqtt();
  }
}


/*
  this function sends message to App
*/
void sendToApp(uint8_t cmd, uint8_t param) {

  uint8_t i, j;
  String topic, aux;
  bool defined = false;
  char json[MAX_JSON_SIZE];

  if (!gprs_on) return;
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& params = jsonBuffer.createObject();
  JsonArray& valves = jsonBuffer.createArray();
  JsonObject& thisValve = valves.createNestedObject();

  switch (cmd) {
    case CONNECT:
      root.set("uuid", sys.devUuid);
      root.set("model", "6011");
      topic = "connect";
      break;
    case MANPRG:
      aux = String((char)(data[CMD_INDEX + 2] +'A'));
      root.set("prog", aux.c_str());
      root.set("action",data[CMD_INDEX + 1]);
      topic += "/manprg";
      break;
    case MANUAL_VALVE:
      root.set("valves", valves);
      thisValve.set("v", param);
      if(pg.indexOf("START") != -1){
        thisValve.set("action", 1);
        j = data[CMD_INDEX + 3];
        aux = (j < 10) ? "0" + String(j) : String(j);
        aux += ":";
        j = data[CMD_INDEX + 4];
        aux += (j < 10) ? "0" + String(j) : String(j);
        thisValve.set("time", aux.c_str());
      }
      else
        thisValve.set("action", 0);
      topic += "/manvalve";
      break;
    case PLUG_PLAY:
    case ASSIGN_VALVES:
      JsonArray& oasis = jsonBuffer.createArray();
      root.set("oasis", oasis);
      JsonObject& thisOasis = oasis.createNestedObject();
      thisOasis.set("id", param + 1);
      thisOasis.set("uuid", sys.oasisUuid[param]);
      JsonArray& valve = thisOasis.createNestedArray("assign");
      for (i = 0; i < 4; i++) {
        if (sys.childValves[param][i]) {
          defined = true;
          break;
        }
      }
      if (defined) {
        for (i = 0; i < 4; i++)
          valve.add(sys.childValves[param][i]);
      }
      root.printTo(json);
      topic = String(sys.devUuid);
      (cmd == PLUG_PLAY) ? topic += "/pairing" : topic += "/oasis";
      break;
  }
  root.printTo(json);
  mqttClient.publish(topic.c_str(), (const uint8_t*)json, strlen(json), false);
  DPRINT("Mqtt sent: ");
  DPRINTLN(json);
}

/*
  this callback function is called everytime a subscription topic message is received
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {

  uint32_t dir;
  char json[MAX_JSON_SIZE];
  String sTopic, jsParsed, str, aux;
  uint8_t i, id, len, val, h, mn, prevChildValves[4];
  DynamicJsonBuffer jsonBuffer(MAX_JSON_SIZE);
  JsonObject& root = jsonBuffer.createObject();

  if (!length) return;
  jsParsed = jam.byteArrayToString(payload, length);
  JsonObject& parsed = jsonBuffer.parseObject(jsParsed);
  root.set("id", parsed["id"]);
  DPRINT("Mqtt received: ");
  DPRINTLN(jsParsed.c_str());

  sTopic = String(topic);
  if (sTopic.indexOf("pairing") != -1) {
    JsonArray& oasis = jsonBuffer.createArray();
    root.set("oasis", oasis);
    for (id = 0; id < sys.nChild; id++) {
      JsonObject& thisOasis = oasis.createNestedObject();
      thisOasis.set("id", id + 1);
      thisOasis.set("uuid", sys.oasisUuid[id]);
      JsonArray& valve = thisOasis.createNestedArray("assign");
      for (i = 0; i < 4; i++) {
        if (sys.childValves[id][i])
          valve.add(sys.childValves[id][i]);
      }
    }
    sTopic = String(sys.devUuid) + "/pairing";
  }
  else if (sTopic.indexOf("oasis") != -1) {
    data[CMD_INDEX] = ASSIGN_VALVES;
    JsonArray& oasis = parsed["oasis"];
    for (i = 0; i < oasis.size(); i++) {
      id = oasis[i]["id"].as<uint8_t>() - 1;
      if ((id + 1) > sys.nChild)
        continue;
      len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
      if (oasis[i]["assign"].size()) {
        for (j = 0; j < TOTAL_OUTPUT; j++, len++) {
          prevChildValves[j] = sys.childValves[id][j];
          sys.childValves[id][j] = oasis[i]["assign"][j];
          data[len] = sys.childValves[id][j];
        }
      }
      else {
        for (j = 0; j < TOTAL_OUTPUT; j++, len++) {
          prevChildValves[j] = sys.childValves[id][j];
          sys.childValves[id][j] = 0;
          data[len] = 0;
        }
      }
      DPRINT("Oasis: ");
      DPRINTLN(id + 1);
      for (j = 0; j < TOTAL_OUTPUT; j++) {
        DPRINT("salida " + String(j + 1) + ": ");
        DPRINTLN(sys.childValves[id][j]);
        if (prevChildValves[j] != sys.childValves[id][j]) {
          dir = 0x200 + (id * 4) + j;
          aux = String(dir, HEX);
          aux.toUpperCase();
          cmd_write_data[WR_ADDR_INDEX] = aux.charAt(0);
          cmd_write_data[WR_ADDR_INDEX + 1] = aux.charAt(1);
          cmd_write_data[WR_ADDR_INDEX + 2] = aux.charAt(2);
          val = sys.childValves[id][i];
          aux = (val < 0x10) ? "0" + String(val, HEX) : String(val, HEX);
          aux.toUpperCase();
          cmd_write_data[WR_DATA_INDEX] = aux.charAt(0);
          cmd_write_data[WR_DATA_INDEX + 1] = aux.charAt(1);
          pgCommand(cmd_write_data, sizeof(cmd_write_data));
        }
      }
      sendCommand(data, len, sys.oasisRfId[id]);
    }
    root.set("success", "true");
    sTopic = String(sys.devUuid) + "/oasis";
    digitalWrite(CS_RF, HIGH);
    flash.eraseSector(SYS_VAR_ADDR);
    flash.writeByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));;
    digitalWrite(CS_RF, LOW);
  }
  else if (sTopic.indexOf("manvalve") != -1) {
    data[CMD_INDEX] = MANUAL_VALVE;
    JsonArray& valves = parsed["valves"];
    for (i = 0; i < valves.size(); i++) {
      data[CMD_INDEX + 1] = valves[i]["action"];
      data[CMD_INDEX + 2] = valves[i]["v"];
      if (valves[i]["action"]) {
        str = valves[i]["time"].as<String>();
        aux = str.substring(0, str.indexOf(":"));
        data[CMD_INDEX + 3] = aux.toInt();
        aux = str.substring(str.indexOf(":") + 1);
        data[CMD_INDEX + 4] = aux.toInt();
      }
      for (id = 0; id < sys.nChild; id++) {
        for (j = 0; j < TOTAL_OUTPUT; j++) {
          if (sys.childValves[id][j] == valves[i]["v"]) {
            len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
            sendCommand(data, len, sys.oasisRfId[id]);
            DPRINTLN("Oasis: " + String(id + 1));
            DPRINTLN("Valve: " + String(data[CMD_INDEX + 2]));
            DPRINTLN("Action: " + String(data[CMD_INDEX + 1]));
            if (valves[i]["action"])
              DPRINTLN("Time: " + String(data[CMD_INDEX + 3]) + ":" + String(data[CMD_INDEX + 4]));
            break;
          }
        }
      }
      if (valves[i]["v"] <= 14) {
        DPRINT("PG valve: ");
        val = valves[i]["v"];
        aux = (val < 10) ? "0" + String(val) : String(val);
        DPRINTLN(aux.c_str());
        if (valves[i]["action"]) {
          cmd_start_manvalv[16] = aux.charAt(0);
          cmd_start_manvalv[17] = aux.charAt(1);
          h = data[CMD_INDEX + 3];
          aux = (h < 10) ? "0" + String(h) : String(h);
          cmd_start_manvalv[19] = aux.charAt(0);
          cmd_start_manvalv[20] = aux.charAt(1);
          mn = data[CMD_INDEX + 4];
          aux = (mn < 10) ? "0" + String(mn) : String(mn);
          cmd_start_manvalv[21] = aux.charAt(0);
          cmd_start_manvalv[22] = aux.charAt(1);
          pgCommand(cmd_start_manvalv, sizeof(cmd_start_manvalv));
        }
        else {
          cmd_stop_manvalv[15] = aux.charAt(0);
          cmd_stop_manvalv[16] = aux.charAt(1);
          pgCommand(cmd_stop_manvalv, sizeof(cmd_stop_manvalv));
        }
      }
      DPRINTLN();
    }
    root.set("success", "true");
    sTopic = String(sys.devUuid) + "/manvalves";
  }
  else if (sTopic.indexOf("manprog") != -1) {
    aux = parsed["prog"].as<String>();
    data[CMD_INDEX] = MANPRG;
    data[CMD_INDEX + 1] = parsed["action"];
    data[CMD_INDEX + 2] = aux.charAt(0) - 'A';
    for (id = 0; id < sys.nChild; id++) {
      DPRINT("Send MANPRG to OASIS: ");
      DPRINTLN(id + 1);
      len = jam.fillWithString(data, String(sys.oasisUuid[id]), PAYLOAD_INDEX);
      sendCommand(data, len, sys.oasisRfId[id]);
    }
    if(parsed["action"]){
      cmd_start_manprg[15] = aux.charAt(0);
      pgCommand(cmd_start_manprg, sizeof(cmd_start_manprg));
    }
    else{
      cmd_stop_manprg[14] = aux.charAt(0);
      pgCommand(cmd_stop_manprg, sizeof(cmd_stop_manprg));
    }
    root.set("success", "true");
    sTopic = String(sys.devUuid) + "/manprg";
  }
  else
    return;
  root.printTo(json);
  mqttClient.publish(sTopic.c_str(), (const uint8_t*)json, strlen(json), false);
  DPRINT("Mqtt sent: ");
  DPRINTLN(json);
}

/*
  this function¡ return battery level
*/
uint8_t batLevel() {
  float res;
  uint8_t a;
  uint16_t b;
  ADCSRA |= (1 << 7);
  analogReference(INTERNAL);
  for (a = 0; a < 5; a++) {
    b = analogRead(VREF_IN);
    delay(1);
  }
  res = -0.0186 * pow(b, 2);
  res += 8.7291 * b - 922;
  if (res < 0) res = 0;
  if (res > 100) res = 100;
  return (res);
}

void sleepDevice() {

  if (gprs_on) {
    ;
  }
  else {
    digitalWrite(CS_RF, HIGH);
    flash.powerDown();
    digitalWrite(CS_RF, LOW);
    driver.sleep();
    digitalWrite(CS_RF, HIGH);
    lowPower.sleeper(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    digitalWrite(CS_RF, HIGH);
    flash.powerUp();
    digitalWrite(CS_RF, LOW);
  }
}

/*
  this funtion get fimware version based on compilation date
*/
void getFirmwVer() {

  int index;
  String aux, ver, str;
  char compileDate[] = __DATE__;

  str = String(compileDate);
  str.replace("  ", " ");
  String month[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  index = str.indexOf(" ") + 1;
  aux = str.substring(0, index);
  for (i = 0; i < 12; i++) {
    if (aux.indexOf(month[i]) != -1)
      break;
  }
  i++;
  ver = (i < 10) ? "0" + String(i) : String(i);
  aux = str.substring(index, str.indexOf(" ", index));
  ver += (aux.toInt() < 10) ? "0" + aux : aux;
  DPRINT("OASIS-COM FW: ");
  DPRINT(ver.c_str());
}

/*
  this funtion set device to factory state
*/
void setupDevice() {

  bool na;
  uint8_t i;
  String str;
  uint16_t b, c;
  uint32_t millix;
  Serial.println(F("Brain Horus  Config/Test 5.27"));
  Serial.println(F("-------- Internal Devices --------"));
  if (strlen(sys.devUuid) != (UUID_LEN - 1)) {
    str = jam.getUUID();
    strcpy(sys.devUuid, str.c_str());
    sys.id = random(MAX_CHILD + 1, 0xfe);
  }
  /*** Flash Test ***/
  flash.eraseSector(0);
  flash.writeByte(0, 80);
  Serial.print(F("Flash Test:   "));
  i = flash.readByte(0);
  if (i == 80)
    Serial.println(F("Ok"));
  else {
    Serial.print(F("Fail"));
    Serial.println(i);
  }
  /*** rtc test ***/
  if (rtc.begin() == false)
    Serial.println(F("RTC Test:     Fail"));
  else {
    Serial.println(F("RTC Test:     Ok"));
    rtc.set24Hour();
  }
  if (rtc.updateTime() == false) {
    Serial.println("RTC failed to update");
  }
  else {
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
  for (i = 0; i < 3; i++) {
    b = analogRead(PA0);
    delay(1);
  }
  analogReference(DEFAULT);
  for (i = 0; i < 3; i++) {
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
  flash.writeByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));
  flash.readByteArray(SYS_VAR_ADDR, (uint8_t*)&sys, sizeof(sys));
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
