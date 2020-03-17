#include <JamAtm-Vyrsa.h>

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
#define WMOTOR_REF 31
#define PWREN 28
#define SLEEP1 17
#define SLEEP2 16
#define AIN1 18
#define AIN2 19
#define BIN1 30
#define BIN2 29
#define NFAULT 20
#define SW_SETUP 0
#define LED_SETUP 3
/******************************************************************* declarations  ************************************************************************************/
#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

#define UUID_LEN 37
#define TOTAL_VALVE 128
#define TOTAL_START 6
#define TOTAL_PROG 6
#define TIME_NDEF 87000
#define FACTORY_TIMEOUT 3000
#define MAX_TEMP 15000
#define PAYLOAD_INDEX 6
#define TX_PWR 20
#define SECTOR_SIZE 4096

#define PLUG_PLAY 0
#define ASSIGN_VALVES 1
#define MANUAL_VALVE 2
#define MEMMORY 3
#define PROGRAM 4
#define AWAKE 5
#define SET_TIME 6
#define MANPRG 7
#define STOP_ALL 8
#define SELECTOR 9
#define CONNECT 10
#define PG_ALL 11
#define ACK 12
#define CMD_INDEX 0

typedef enum
{
  RSENSORS_MSG = 'R',
  ACTION_MSG = 'A',
  SETUP_MSG = 'S',
  TIME_MSG = 'T'
} msg_receive;

typedef enum
{
  ACK_enum,
  FAULT,
  SENSORS
} msg_send;

typedef struct
{
  uint8_t id;
  uint8_t serverId;
  uint8_t activated;
  uint32_t earlierStart;
  char devUuid[UUID_LEN];
  uint8_t assignedValve[4];
  uint8_t interval[TOTAL_PROG];
  uint8_t startDay[TOTAL_PROG];
  uint8_t wateringDay[TOTAL_PROG];
  uint16_t waterPercent[TOTAL_PROG];
  uint32_t starts[TOTAL_PROG][TOTAL_START];
} sysVar;

typedef struct
{
  int action;
  bool match;
  uint8_t valve;
  uint32_t start;
  uint32_t schedule;
  uint8_t manual;
} programPtr;

typedef struct
{
  uint8_t valve[4];
  uint32_t schedule[4];
  uint8_t count;
} manualvalve;

Jam jam;
sysVar sys;
RV1805 rtc;
Sleep lowPower;
SPIFlash flash(CS_M);
manualvalve manValve;
programPtr prgPtr[TOTAL_PROG];
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver);
volatile bool intButton = false, intRtc = false, Global_Flag_int = false;

uint16_t Set_Vshot = 600;
bool newDay = false, stopAll = false;
uint8_t iOpen = 0, valveOpened[4], data[RH_RF95_MAX_MESSAGE_LEN];
uint32_t currentTime, alarmTime, alarmManVal, millix, irrigTime[TOTAL_VALVE];
uint8_t i, j, cmd, msgPending, minAlarm, hourAlarm, secAlarm, dataSize;

bool action_flag, setup_flag, time_flag, sensor_flag, rf_flag;
uint8_t data_size, buf[RH_RF95_MAX_MESSAGE_LEN];

/***** functions Prototypes ******/
void rtcHandler();
//void rfHandler();
void buttonInt();
void initDevice();
void plugAndPlay();
void setupDevice();
uint8_t batLevel();
void getFirmwVer();
void buttonHandler();
void getProgramPtr();
void executeCommand();
void chargeCapacitor();
void checkManValAlarm();
void executeProgram(bool mode);
void valveAction(uint8_t Valve, boolean Dir);
/******************************************************************* setup section ************************************************************************************/
void setup()
{
#ifdef DEBUG_ON
  Serial.begin(115200);
#endif
  delay(250);
  getFirmwVer();
  pinMode(CS_RF, OUTPUT);
  digitalWrite(CS_RF, HIGH);
  pinMode(RF_RST, OUTPUT);
  digitalWrite(RF_RST, HIGH);
  pinMode(INT_RF, INPUT_PULLUP);
  pinMode(SW_SETUP, INPUT);
  pinMode(INT_RTC, INPUT);
  pinMode(PWREN, OUTPUT);
  pinMode(SLEEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_SETUP, OUTPUT);
  digitalWrite(PWREN, LOW);
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  flash.powerUp();
  flash.begin();
  flash.readAnything(SYS_VAR_ADDR, sys);
  flash.readByteArray(IRRIG_ADDR, (uint8_t *)irrigTime, sizeof(irrigTime));
  DPRINT(F(", UUID: "));
  DPRINTLN(sys.devUuid);
  manager.setThisAddress(sys.id);
  manager.init();
  driver.setPreambleLength(8);
  driver.setTxPower(TX_PWR, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  attachPCINT(digitalPinToPCINT(SW_SETUP), buttonInt, FALLING);
  chargeCapacitor();
  jam.ledBlink(LED_SETUP, 1000);
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  for (i = 0; i < TOTAL_PROG; i++)
    prgPtr[i].manual = false;
  for (i = 0; i < 4; i++)
  {
    manValve.valve[i] = 0;
    valveOpened[i] = 0;
    manValve.schedule[i] = TIME_NDEF;
  }
  manValve.count = 0;
  //if (sys.activated)
  //  //executeProgram(true);
  //  delay(5000);
  //valveAction(3, false);
  //delay(10000);
  //valveAction(3, true);
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{

  if (Serial.available())
  {
    int a = Serial.read();
    Serial.println(a);
    if (a == 97)
    {
      delay(2000);
      valveAction(1, true);
      delay(2000);
      valveAction(1, false);
    }
    if (a == 98)
    {
      delay(2000);
      valveAction(2, true);
      delay(2000);
      valveAction(2, false);
    }
    if (a == 99)
    {
      delay(2000);
      valveAction(3, true);
      delay(2000);
      valveAction(3, false);
    }
  }
  //if (intRtc)
  //{
  //  intRtc = false;
  //  rtcHandler();
  //}
  //if (intButton)
  //{
  //  buttonHandler();
  //  intButton = false;
  //}
  //rfHandler();
  //driver.sleep();
  //lowPower.sleep_delay(288);
}

/*******************************************************************   functions     ************************************************************************************/
/*
this callback funtion is called when button is pressed
*/
void buttonInt()
{
  intButton = true; //set flag to indicate that button interrupt was triggered
  Global_Flag_int = true;
}
/*
this callback funtion is called when rtc interruot is triggered
*/
void rtcInt()
{
  intRtc = true; //set flag to indicate that button interrupt was triggered
  Global_Flag_int = true;
}
/*
  this function notify emissor as new device
*/
void plugAndPlay()
{

  uint8_t i;
  DPRINTLN(F("Send Plug&Play"));
  data[CMD_INDEX] = PLUG_PLAY;                                      //set command id ans plugandplay
  i = jam.fillWithString(data, String(sys.devUuid), PAYLOAD_INDEX); //add device name
  manager.setThisAddress(0xfd);                                     //set plug&play id
  manager.sendtoWait(data, i, sys.serverId);                        //send plugandplay command to emissor
}
/*
  this callback funtion is called when button is pressed
*/
void buttonHandler()
{

  DPRINTLN(F("Button pressed"));
  millix = millis(); //start millis count
  while ((!digitalRead(SW_SETUP)) && ((unsigned long)(millis() - millix) < MAX_TEMP))
    ;                                                        //while button is pressed and does not reach max temp
  if ((unsigned long)(millis() - millix) >= FACTORY_TIMEOUT) //check if it was long press button
    setupDevice();                                           //call setup Device Funtion
  else
  {                     //otherwise it is shor press button
    if (!sys.activated) //if device is not activated
      plugAndPlay();    //send plug&play command
  }
}
/*
this function execute command received 
*/

/*
  this function charge capacitor
*/
void chargeCapacitor()
{

  uint16_t a, Vshot;
  for (a = 0; a < 350; a++)
  {
    ADCSRA |= (1 << 7);
    if ((analogRead(WMOTOR_REF) - Vshot) < 5)
    {
      Set_Vshot = Vshot;
      break;
    }
    Vshot = analogRead(WMOTOR_REF);
    digitalWrite(PWREN, HIGH);
    lowPower.sleep_delay(16);

    digitalWrite(PWREN, LOW);
    lowPower.sleep_delay(16);
  }
}
/*
  this function return battery level
*/
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
/*
  this function open or close a valve
*/
void valveAction(uint8_t Valve, boolean Dir)
{

  uint16_t Vshot, a;
  bool defined = false;
  uint8_t i, Drv, Out, OutB, nAction = 0, aux;

  DPRINTLN(F("*Valve Action"));
  DPRINT(F("  valve:  "));
  DPRINTLN(Valve);
  DPRINT(F("  action: "));
  DPRINTLN(Dir);
  DPRINTLN(F("*"));
  delay(5);

  aux = Valve;
  for (i = 0; i < 4; i++)
  {
    if (i == Valve -1)
    {
      if (nAction > 0)
      {
        delay(5);
        lowPower.sleep_delay(2048);
      }
      Valve = i + 1;
      for (a = 0; a < 350; a++)
      {
        ADCSRA |= (1 << 7);
        if ((analogRead(WMOTOR_REF) - Vshot) < 5)
        {
          digitalWrite(PWREN, HIGH);
          lowPower.sleep_delay(64);

          digitalWrite(PWREN, LOW);
          Vshot = analogRead(WMOTOR_REF);
          break;
        }
        Vshot = analogRead(WMOTOR_REF);
        digitalWrite(PWREN, HIGH);
        lowPower.sleep_delay(16);
        digitalWrite(PWREN, LOW);
        lowPower.sleep_delay(16);
      }
      if (Valve > 2)
      {
        Drv = SLEEP2;
      }
      else
        Drv = SLEEP1;
      digitalWrite(Drv, HIGH);
      if (Valve % 2 == 0)
      {
        if (Dir)
        {
          Out = BIN2;
          OutB = BIN1;
        }
        else
        {
          Out = BIN1;
          OutB = BIN2;
        }
      }
      else
      {
        if (Dir)
        {
          Out = AIN2;
          OutB = AIN1;
        }
        else
        {
          Out = AIN1;
          OutB = AIN2;
        }
      }
      digitalWrite(Out, HIGH);
      lowPower.sleep_delay(128);

      digitalWrite(Out, LOW);
      digitalWrite(Out, HIGH);
      digitalWrite(OutB, HIGH);
      delay(1);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      delay(1);
      digitalWrite(Drv, LOW);
      digitalWrite(LED_SETUP, HIGH);
      lowPower.sleep_delay(512);

      digitalWrite(LED_SETUP, LOW);
      Serial.print(F("Salida: "));
      Serial.println(i + 1);
      Serial.print(F("Time Capacitor (ms):"));
      Serial.println(a * 30 + 64);
      Serial.print(F("Voltage:"));
      Serial.println(Vshot * 0.0107);
      nAction++;
    }
  }
  for (i = 0; i < 4; i++)
  {
    if (valveOpened[i] == aux)
    {
      if (Dir)
        defined = true;
      else
      {
        valveOpened[i] = 0;
        iOpen--;
      }
      break;
    }
  }
  if (Dir && !defined)
  {
    valveOpened[iOpen] = aux;
    iOpen++;
  }
  delay(5);
}
/*
  this funtion get fimware version based on compilation date
*/
void getFirmwVer()
{

  int index;
  String aux, ver, str;
  char compileDate[] = __DATE__;
  str = String(compileDate);
  str.replace("  ", " ");
  String month[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  index = str.indexOf(" ") + 1;
  aux = str.substring(0, index);
  for (i = 0; i < 12; i++)
  {
    if (aux.indexOf(month[i]) != -1)
      break;
  }
  i++;
  ver = (i < 10) ? "0" + String(i) : String(i);
  aux = str.substring(index, str.indexOf(" ", index));
  ver += (aux.toInt() < 10) ? "0" + aux : aux;
  DPRINT(F("OASIS FW: "));
  DPRINT(ver.c_str());
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
  Serial.println(F("Brain Horus  Config/Test 5.27"));
  Serial.println(F("-------- Internal Devices --------"));
  if (strlen(sys.devUuid) != (UUID_LEN - 1))
  {
    str = jam.getUUID();
    strcpy(sys.devUuid, str.c_str());
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
    Serial.println(F("RTC failed to update"));
  }
  else
  {
    if (rtc.getYear() == 0)
      Serial.println(F("First RTC timer and config Charge"));
  }
  rtc.updateTime();
  Serial.print(F("Date/Time:    "));
  Serial.print(rtc.stringDate());
  Serial.print(F(" "));
  Serial.println(rtc.stringTime());
  Serial.print(F("Timestamp:    "));
  Serial.println(rtc.getTimestamp());
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
  Serial.println(F("-------- OASIS --------"));
  sys.id = 0xfe;
  sys.serverId = 0;
  sys.activated = 0;
  manager.setThisAddress(sys.id);
  sys.earlierStart = TIME_NDEF;
  for (i = 0; i < 4; i++)
    sys.assignedValve[i] = 0;
  digitalWrite(CS_RF, HIGH);
  for (i = 0; i < TOTAL_PROG; i++)
  {
    //clearProgram(i);
    flash.eraseSector(IRRIG_ADDR + (i * SECTOR_SIZE));
    flash.writeByteArray(IRRIG_ADDR + (i * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
  }
  flash.eraseSector(SYS_VAR_ADDR);
  flash.writeAnything(SYS_VAR_ADDR, sys);
  flash.readAnything(SYS_VAR_ADDR, sys);
  flash.readByteArray(IRRIG_ADDR, (uint8_t *)irrigTime, sizeof(irrigTime));
  digitalWrite(CS_RF, LOW);
  Serial.print(F("UUID: "));
  Serial.println(sys.devUuid);
  Serial.print(F("RF Id: "));
  Serial.println(sys.id, HEX);
  Serial.print(F("Server Id: "));
  Serial.println(sys.serverId);
  Serial.print(F("Activated: "));
  Serial.println(sys.activated);
  Serial.print(F("Earlier start: "));
  (sys.earlierStart == TIME_NDEF) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Assigned alves: "));
  na = true;
  for (i = 0; i < 4; i++)
  {
    if (sys.assignedValve[i] != 0)
    {
      na = false;
      break;
    }
  }
  (na) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Starts: "));
  na = true;
  for (i = 0; i < TOTAL_START; i++)
  {
    if (sys.starts[0][i] != TIME_NDEF)
    {
      na = false;
      break;
    }
  }
  (na) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Valve irrigation time: "));
  na = true;
  for (i = 0; i < TOTAL_VALVE; i++)
  {
    if (irrigTime[i] != 0)
    {
      na = false;
      break;
    }
  }
  (na) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Water Percent: "));
  (sys.waterPercent[0] == 0) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Watering days: "));
  (sys.wateringDay[0] == 0) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Interval: "));
  (sys.interval[0] == 0) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Start day: "));
  (sys.startDay[0] == 0) ? Serial.println(F("n/a")) : Serial.println(F("fail"));
  Serial.print(F("Battery: "));
  Serial.println(batLevel());
  jam.ledBlink(LED_SETUP, 3000);
}
