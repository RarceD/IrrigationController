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

/***** functions Prototypes ******/
void rtcHandler();
void rfHandler();
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

  initDevice();

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
        valveAction(3, true);
  delay(5000);
  valveAction(3, false);
    }
  }
  // if (intRtc)
  // {
  //   intRtc = false;
  //   rtcHandler();
  // }
  // if (intButton)
  // {
  //   buttonHandler();
  //   intButton = false;
  // }
  // rfHandler();
  // driver.sleep();
  // lowPower.sleep_delay(288);
}

/*******************************************************************   functions     ************************************************************************************/

/*
  this function initializes hardware resources
*/
void initDevice()
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
  if (sys.activated)
    executeProgram(true);
}

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
this function handle rf messages
*/
void rfHandler()
{

  if (driver.isChannelActive())
  {
    driver.setModeRx();
    lowPower.sleep_delay(336);
    dataSize = RH_RF95_MAX_MESSAGE_LEN; //set rf data size
    if (manager.recvfromAck(data, &dataSize))
    {                                              //if data is into buffer
      cmd = data[CMD_INDEX];                       //get command id
      if (jam.getDst(data) == String(sys.devUuid)) //if message is for this device
        executeCommand();                          //execute command
    }
  }
  millix = millis(); //start millis count
  while (msgPending || ((unsigned long)(millis() - millix) < 5000))
  {                                     //while there is pending message or timer doesn't reach 5 sec
    dataSize = RH_RF95_MAX_MESSAGE_LEN; //set rf data size
    if (manager.recvfromAck(data, &dataSize))
    {                                              //if data is into buffer
      msgPending--;                                //decrease number of message pending
      cmd = data[CMD_INDEX];                       //get command id
      if (jam.getDst(data) == String(sys.devUuid)) //if message is for this device
        executeCommand();                          //execute command
    }
  }
}

/*
  this callback funtion is called when rtc interruption is triggered
*/
void rtcHandler()
{

  uint8_t i, h, m;

  byte rtcStatus = rtc.status(); //get rtc status
  if (rtcStatus != 0)
  { //if any interrupt was triggered
    if (rtcStatus & (1 << STATUS_AF))
    {                                                                                      //if it is alarm interrupt
      rtc.updateTime();                                                                    //get rtc time
      currentTime = ((uint32_t)rtc.getHours() * 3600) + ((uint32_t)rtc.getMinutes() * 60); //set current time in seconds
      DPRINTLN(F("****************"));
      DPRINT(F("Alarm trigger: "));
      h = currentTime / 3600;
      m = (currentTime - ((uint32_t)h * 3600)) / 60;
      DPRINT(h);
      DPRINT(F(":"));
      DPRINTLN(m);
      DPRINTLN(F("****************"));

      checkManValAlarm();
      if (currentTime == alarmTime)
      {
        if (currentTime == sys.earlierStart)
        {
          newDay = false;
          executeProgram(true);
        }
        else
          executeProgram(false);
      }
    }
  }
}

/*
this function execute command received 
*/
void executeCommand()
{

  int sec;
  uint32_t timeAux;
  uint16_t aux, percent;
  uint8_t pos, h, m, a, p;
  bool scheduleChange, opened;

  switch (cmd)
  {
  case AWAKE:
    DPRINTLN(F("AWAKE command "));
    if (msgPending = data[CMD_INDEX + 1])
      driver.setModeRx();
    break;
  case SET_TIME:
    DPRINTLN(F("Setime command "));
    i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;           //set data pointer to payload index
    rtc.setTime(0, data[i], data[i + 1], data[i + 2],      //set rtc time
                data[i + 3], data[i + 4], data[i + 5], 0); //and date
    rtc.updateTime();
    DPRINT(rtc.stringDate());
    DPRINT(F(" "));
    DPRINTLN(rtc.stringTime());
    executeProgram(true);
    break;
  case PLUG_PLAY:
    DPRINTLN(F("Plug&Play command"));
    sys.id = data[CMD_INDEX + 1];                          //get new rf id assigned
    manager.setThisAddress(sys.id);                        //set this new rf id to rf manager
    i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;           //set data pointer to payload index
    rtc.setTime(0, data[i], data[i + 1], data[i + 2],      //set rtc time
                data[i + 3], data[i + 4], data[i + 5], 0); //and date
    sys.activated = 1;                                     //set device as activated
    digitalWrite(CS_RF, HIGH);                             //unselect rf
    flash.eraseSector(SYS_VAR_ADDR);                       //erase sys variable sector
    flash.writeAnything(SYS_VAR_ADDR, sys);                //write sys variable changes
    digitalWrite(CS_RF, LOW);                              //select again rf
    DPRINT(F("id: "));
    DPRINTLN(sys.id);
    rtc.updateTime();
    DPRINT(rtc.stringDate());
    DPRINT(F(" "));
    DPRINTLN(rtc.stringTime());
    jam.ledBlink(LED_SETUP, 1000);
    break;
  case ASSIGN_VALVES:
    DPRINTLN(F("Assigned valve command"));
    i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1; //set pointer to valves assigned
    for (j = 0; j < TOTAL_OUTPUT; j++, i++)
    {                                 //for each output
      sys.assignedValve[j] = data[i]; //save valve assigned
      DPRINTLN("Salida " + String(j + 1) + ": " + String(data[i]));
    }
    digitalWrite(CS_RF, HIGH);              //unselect rf
    flash.eraseSector(SYS_VAR_ADDR);        //erase sys variable sector
    flash.writeAnything(SYS_VAR_ADDR, sys); //write sys variable changes
    digitalWrite(CS_RF, LOW);               //select again rf
    executeProgram(true);
    break;
  case STOP_ALL:
    DPRINTLN(F("Stop all command"));
    for (i = 0; i < 4; i++)
    {
      opened = false;
      if (sys.assignedValve[i])
      {
        for (j = 0; j < 4; j++)
        {
          if (sys.assignedValve[i] == valveOpened[j])
          {
            opened = true;
            break;
          }
        }
        if (opened)
        {
          valveAction(sys.assignedValve[i], 0);
          lowPower.sleep_delay(2048);
        }
      }
    }
    for (i = 0; i < 4; i++)
    {
      manValve.valve[i] = 0;
      manValve.schedule[i] = TIME_NDEF;
    }
    manValve.count = 0;
    stopAll = true;
    break;
  case SELECTOR:
    DPRINT(F("Selector command: "));
    DPRINTLN(data[CMD_INDEX + 1]);
    if (stopAll)
      stopAll = false;
    executeProgram(true);
    break;
  case MANUAL_VALVE:
    DPRINT(F("Manual valve command: "));
    DPRINT(data[CMD_INDEX + 3]);
    DPRINT(F(":"));
    DPRINTLN(data[CMD_INDEX + 4]);
    if (stopAll)
      stopAll = false;
    rtc.updateTime();
    currentTime = ((uint32_t)rtc.getHours() * 3600) + ((uint32_t)rtc.getMinutes() * 60); //get current time
    if (data[CMD_INDEX + 1])
    { //if action is to open valve
      for (i = 0, scheduleChange = false; i < TOTAL_PROG; i++)
      { //for each program
        if (prgPtr[i].match)
        { //if program match
          if ((prgPtr[i].valve == data[CMD_INDEX + 2]) &&
              (prgPtr[i].action == 0) && (prgPtr[i].schedule > currentTime))
          {                                                                //check if valve is already opened by a program
            prgPtr[i].schedule += ((uint32_t)data[CMD_INDEX + 3] * 3600) + //sum remaining opening valve time by program and manual valve time
                                  ((uint32_t)data[CMD_INDEX + 4] * 60);
            scheduleChange = true;
          }
        }
      }
      if (scheduleChange)
      { //if schedule was changed
        for (i = 0, alarmTime = TIME_NDEF; i < TOTAL_PROG; i++)
        { //go through all program
          if (prgPtr[i].match)
          { //if program match
            if ((prgPtr[i].schedule >= currentTime) &&
                (prgPtr[i].schedule < alarmTime) && (prgPtr[i].valve != 0)) //get lowest program schedule and update alarmTime
              alarmTime = prgPtr[i].schedule;
          }
        }
        timeAux = alarmTime; //save new schedule into timeAux
      }
      else
      {                                                        //if valve is not opened
        valveAction(data[CMD_INDEX + 2], data[CMD_INDEX + 1]); //open valve
        for (i = 0; i < 4; i++)
        { //for each manvalve action
          if (manValve.valve[i] == 0)
          {                                                                             //if action it is not defined
            manValve.valve[i] = data[CMD_INDEX + 2];                                    //define valve to be closed
            manValve.schedule[i] = currentTime + ((uint32_t)data[CMD_INDEX + 3] * 3600) //define manvalve schedule time
                                   + ((uint32_t)data[CMD_INDEX + 4] * 60) + rtc.getSeconds();
            break;
          }
        }
        manValve.count++;
        if (i == 0)
          alarmManVal = TIME_NDEF;                      //if it is first manvalve in queue, initialize alarManVal
        if (manValve.schedule[i] < alarmManVal)         //if manvalve schedule is less then alarmManVal
          timeAux = alarmManVal = manValve.schedule[i]; //update timeAux and alarmManVal value
        else                                            //otherwise
          return;                                       //do not change current manvale schedule
      }
      if ((timeAux < alarmTime) || newDay)
      {                                                           //if there is a schedule ealier then alarm time
        hourAlarm = timeAux / 3600;                               //set alarm hour
        minAlarm = (timeAux - ((uint32_t)hourAlarm * 3600)) / 60; //set alarm minutes
        rtc.setAlarmMode(4);                                      // set new alarm time
        rtc.setAlarm(rtc.getSeconds(), minAlarm, hourAlarm, 0, 0);
        DPRINT(F("set Alarm: "));
        DPRINT(hourAlarm);
        DPRINT(F(":"));
        DPRINT(minAlarm);
        DPRINT(F(":"));
        DPRINTLN(rtc.getSeconds());
      }
    }
    else
    {                                                        //if action is to close valve
      valveAction(data[CMD_INDEX + 2], data[CMD_INDEX + 1]); //close valve
      for (i = 0; i < 4; i++)
      { //for each manvalve action
        if (manValve.valve[i] == data[CMD_INDEX + 2] && manValve.schedule[i] != TIME_NDEF)
        {                        //if there is an action and schedule defines for this valve
          manValve.valve[i] = 0; //undefined it
          manValve.schedule[i] = TIME_NDEF;
          manValve.count--;
          break;
        }
      }
      if (!manValve.count)
      { //if there is no more manvalve in queue
        hourAlarm = alarmTime / 3600;
        minAlarm = (alarmTime - ((uint32_t)hourAlarm * 3600)) / 60;
        rtc.setAlarmMode(4);
        rtc.setAlarm(0, minAlarm, hourAlarm, 0, 0);
        DPRINT(F("set Alarm: "));
        DPRINT(hourAlarm);
        DPRINT(F(":"));
        DPRINTLN(minAlarm);
      }
    }
    break;
  case MANPRG:
    DPRINT(F("Manual Prog command: "));
    p = data[CMD_INDEX + 2];
    DPRINT((char)(p + 'A'));
    DPRINTLN(", " + String(data[CMD_INDEX + 1]));
    if (stopAll)
      stopAll = false;
    if ((data[CMD_INDEX + 1]))
    {
      prgPtr[p].manual = 1;
      executeProgram(true);
    }
    else
    {
      for (i = 0; i < 4; i++)
      {
        opened = false;
        if (sys.assignedValve[i])
        {
          for (j = 0; j < 4; j++)
          {
            if (sys.assignedValve[i] == valveOpened[j])
            {
              opened = true;
              break;
            }
          }
          if (opened)
          {
            valveAction(sys.assignedValve[i], 0);
            lowPower.sleep_delay(2048);
          }
        }
      }
      prgPtr[p].manual = 0;
      executeProgram(true);
    }
    break;
  case MEMMORY:
    i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;
    pos = data[CMD_INDEX + 1];
    if (pos == 1)
    {
      DPRINTLN(F("Memmory aporte, interval, watering"));
      for (j = 0; j < TOTAL_PROG; j++, i += 2)
      {                                                      //for each program
        if (data[i] == 0xff)                                 //if water percent is not defined
          percent = 100;                                     //set it to 0
        else                                                 //otherwise
          percent = (uint16_t)(data[i] * 256) + data[i + 1]; //calculate and save its value
        if (percent != sys.waterPercent[j])
        {
          sys.waterPercent[j] = percent;
          digitalWrite(CS_RF, HIGH);
          flash.readByteArray(IRRIG_ADDR + (j * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
          for (a = 0; a < TOTAL_VALVE; a++)
            irrigTime[a] *= ((float)percent / 100);
          flash.eraseSector(IRRIG_ADDR + (j * SECTOR_SIZE));
          flash.writeByteArray(IRRIG_ADDR + (j * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
          digitalWrite(CS_RF, LOW);
        }
        DPRINT(F("Prog "));
        DPRINT((char)(j + 'A'));
        DPRINTLN(": " + String(sys.waterPercent[j]));
      }
      for (j = 0; j < TOTAL_PROG; j++, i++)
      {                                                                        //for each program
        (data[i] == 0xff) ? sys.interval[j] = 0 : sys.interval[j] = data[i];   //if interval is not defined set it to 0, otherwise save its value
        (data[++i] == 0xff) ? sys.startDay[j] = 0 : sys.startDay[j] = data[i]; //if startDay is not defined set it to 0, otherwise save its value
        DPRINT(F("*Prog "));
        DPRINTLN((char)(j + 'A'));
        DPRINT(F("  interval: "));
        DPRINTLN(sys.interval[j]);
        DPRINT(F("  startDay: "));
        DPRINTLN(sys.startDay[j]);
      }
      for (j = 0; j < TOTAL_PROG; j++, i++)
      {                                                                               //for each program
        (data[i] == 0xff) ? sys.wateringDay[j] = 0x7f : sys.wateringDay[j] = data[i]; //if wateringDay is not defined set it to 0, otherwise save its value
        DPRINT(F("Prog "));
        DPRINT((char)(j + 'A'));
        DPRINTLN(": " + String(sys.wateringDay[j]));
      }
      digitalWrite(CS_RF, HIGH);
      flash.eraseSector(SYS_VAR_ADDR);
      flash.writeAnything(SYS_VAR_ADDR, sys);
      digitalWrite(CS_RF, LOW);
    }
    else if (pos == 3)
    {
      DPRINTLN(F("Memmory arranque"));
      sys.earlierStart = TIME_NDEF;
      for (j = 0; j < TOTAL_PROG; j++)
      {
        DPRINT(F("Prog "));
        DPRINT((char)(j + 'A'));
        for (a = 0; a < TOTAL_START; a++)
        { //for each start of each program
          if ((data[i] == 0xff) || (data[i + 1] == 0xff))
          {                               //if minutes or hour is not defined
            sys.starts[j][a] = TIME_NDEF; //set start as not defined
            i += 2;                       //go to next start
          }
          else
          {                                                  //if minutes and hour are defined
            sys.starts[j][a] = ((uint32_t)data[i++] * 3600); //convert hour in seconds
            sys.starts[j][a] += ((uint32_t)data[i++] * 60);  //convert minutes in secons and save start in sexond
          }
          if (sys.starts[j][a])
          {                                          //if start is different from 0
            if (sys.starts[j][a] < sys.earlierStart) //check if start is less then earlier start
              sys.earlierStart = sys.starts[j][a];   //if so save this start as earlier start
          }
        }
        DPRINTLN(": " + String(sys.starts[j][0]) + ", " + String(sys.starts[j][1]) +
                 ", " + String(sys.starts[j][2]) + ", " + String(sys.starts[j][3]) +
                 ", " + String(sys.starts[j][4]) + ", " + String(sys.starts[j][5]));
      }
      digitalWrite(CS_RF, HIGH);
      flash.eraseSector(SYS_VAR_ADDR);
      flash.writeAnything(SYS_VAR_ADDR, sys);
      digitalWrite(CS_RF, LOW);
    }
    else if (pos > 7)
    {
      pos = pos - 8;
      DPRINTLN(F("Memmory tiempo de riego"));
      DPRINT(F("Prog "));
      DPRINT((char)((pos) + 'A'));
      for (j = 0; j < TOTAL_VALVE; j++, i++)
      {                      //for each vale
        if (data[i] == 0xff) //if irrigation time is not defined
          irrigTime[j] = 0;  //set irrigation time to 0
        else
        { //otherwise
          if (data[i] > 60)
          {                                                           //if irrigation timer greater then 0
            aux = (uint16_t)((data[i] - 60) * 5) + 60;                //convert it
            h = aux / 60;                                             //get hour
            m = aux - (h * 60);                                       //get minutes
            irrigTime[j] = (uint32_t)(m * 60) + (uint32_t)(h * 3600); //set irrigation time in second
          }
          else                                     //otherwise
            irrigTime[j] = (uint32_t)data[i] * 60; //save irrigation time
          irrigTime[j] *= ((float)sys.waterPercent[pos] / 100);
        }
        DPRINT("v" + String(j + 1) + String(":"));
        DPRINT(irrigTime[j]);
        DPRINT(F(" "));
      }
      digitalWrite(CS_RF, HIGH);
      flash.eraseSector(IRRIG_ADDR + (pos * SECTOR_SIZE));
      flash.writeByteArray(IRRIG_ADDR + (pos * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
      digitalWrite(CS_RF, LOW);
    }
    executeProgram(true);
    break;
  case PROGRAM:
    p = data[CMD_INDEX + 1];
    DPRINT(F("Program command: "));
    DPRINTLN((char)(p + 'A'));
    i = PAYLOAD_INDEX + strlen(sys.devUuid) + 1;
    DPRINT(F("tiempo de riego: "));
    for (j = 0; j < TOTAL_VALVE; j++, i++)
    {                      //for each vale
      if (data[i] == 0xff) //if irrigation time is not defined
        irrigTime[j] = 0;  //set irrigation time to 0
      else
      { //otherwise
        if (data[i] > 60)
        {                                                           //if irrigation timer greater then 0
          aux = (uint16_t)((data[i] - 60) * 5) + 60;                //convert it
          h = aux / 60;                                             //get hour
          m = aux - (h * 60);                                       //get minutes
          irrigTime[j] = (uint32_t)(m * 60) + (uint32_t)(h * 3600); //set irrigation time in second
        }
        else                                     //otherwise
          irrigTime[j] = (uint32_t)data[i] * 60; //save irrigation time
        irrigTime[j] *= ((float)sys.waterPercent[pos] / 100);
      }
      DPRINT("v" + String(j + 1) + String(":"));
      DPRINT(irrigTime[j]);
      DPRINT(F(" "));
    }
    digitalWrite(CS_RF, HIGH);
    flash.eraseSector(IRRIG_ADDR + (p * SECTOR_SIZE));
    flash.writeByteArray(IRRIG_ADDR + (p * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
    digitalWrite(CS_RF, LOW);
    sys.earlierStart = TIME_NDEF;
    for (a = 0; a < TOTAL_START; a++)
    { //for each start of each program
      if ((data[i] == 0xff) || (data[i + 1] == 0xff))
      {                               //if minutes or hour is not defined
        sys.starts[p][a] = TIME_NDEF; //set start as not defined
        i += 2;                       //go to next start
      }
      else
      {                                                  //if minutes and hour are defined
        sys.starts[p][a] = ((uint32_t)data[i++] * 3600); //convert hour in seconds
        sys.starts[p][a] += ((uint32_t)data[i++] * 60);  //convert minutes in secons and save start in sexond
      }
      if (sys.starts[p][a])
      {                                          //if start is different from 0
        if (sys.starts[p][a] < sys.earlierStart) //check if start is less then earlier start
          sys.earlierStart = sys.starts[p][a];   //if so save this start as earlier start
      }
    }
    DPRINT(F("\narranque: "));
    DPRINTLN(": " + String(sys.starts[p][0]) + ", " + String(sys.starts[p][1]) +
             ", " + String(sys.starts[p][2]) + ", " + String(sys.starts[p][3]) +
             ", " + String(sys.starts[p][4]) + ", " + String(sys.starts[p][5]));
    if (data[i] == 0xff)                                 //if water percent is not defined
      percent = 100;                                     //set it to 0
    else                                                 //otherwise
      percent = (uint16_t)(data[i] * 256) + data[i + 1]; //calculate and save its value
    if (percent != sys.waterPercent[p])
    {
      sys.waterPercent[p] = percent;
      digitalWrite(CS_RF, HIGH);
      flash.readByteArray(IRRIG_ADDR + (p * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
      for (a = 0; a < TOTAL_VALVE; a++)
        irrigTime[a] *= ((float)percent / 100);
      flash.eraseSector(IRRIG_ADDR + (p * SECTOR_SIZE));
      flash.writeByteArray(IRRIG_ADDR + (p * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
      digitalWrite(CS_RF, LOW);
    }
    i += 2;
    (data[i] == 0xff) ? sys.interval[p] = 0 : sys.interval[p] = data[i];            //if interval is not defined set it to 0, otherwise save its value
    (data[++i] == 0xff) ? sys.startDay[p] = 0 : sys.startDay[p] = data[i];          //if startDay is not defined set it to 0, otherwise save its value
    (data[++i] == 0xff) ? sys.wateringDay[p] = 0x7f : sys.wateringDay[p] = data[i]; //if wateringDay is not defined set it to 0, otherwise save its value
    digitalWrite(CS_RF, HIGH);
    flash.eraseSector(SYS_VAR_ADDR);
    flash.writeAnything(SYS_VAR_ADDR, sys);
    digitalWrite(CS_RF, LOW);
    DPRINT(F("waterPercent: "));
    DPRINTLN(sys.waterPercent[p]);
    DPRINT(F("interval: "));
    DPRINTLN(sys.interval[p]);
    DPRINT(F("startDay: "));
    DPRINTLN(sys.startDay[p]);
    DPRINT(F("wateringDay: "));
    DPRINTLN(sys.wateringDay[p]);
    executeProgram(true);
    break;
  }
  delay(5);
}

/*
  this function execute valves program
*/
void executeProgram(bool mode)
{

  uint8_t i;
  bool scheduleFound = false;
  rtc.updateTime();
  currentTime = ((uint32_t)rtc.getHours() * 3600) + ((uint32_t)rtc.getMinutes() * 60);

  if (stopAll)
    return;
  if (!dayMatch())
  {                                                        //check if there no match for current day
    alarmTime = sys.earlierStart;                          //set alarm time as earlier start from next day
    hourAlarm = sys.earlierStart / 3600;                   //if so, set alarm hour from earlier start of next day
    minAlarm = (sys.earlierStart - hourAlarm * 3600) / 60; //set alarm minutes from earlier start of next day
    newDay = true;
    DPRINTLN(F("No match, schedule to new day"));
  }
  else
  {
    if (mode)
      getProgramPtr(); //get program pointer
    else
    {
      for (i = 0; i < TOTAL_PROG; i++)
      { //go through all program
        if (prgPtr[i].match)
        { //if program match
          if (currentTime == prgPtr[i].schedule)
          {                                                 //if currentTime is equal to this program schedule
            valveAction(prgPtr[i].valve, prgPtr[i].action); //actuate in it valve program
            progPtrNext(i, prgPtr[i].valve);                //point program to the next action
          }
        }
      }
    }
    for (i = 0, alarmTime = TIME_NDEF; i < TOTAL_PROG; i++)
    { //go through all program
      if (prgPtr[i].match)
      { //if program match
        if ((prgPtr[i].schedule < alarmTime) && (prgPtr[i].valve != 0))
        {                                 //if next action time for this program is less then alarmtime and valve to actuate is defined
          alarmTime = prgPtr[i].schedule; //update lowest alarm time
          scheduleFound = true;
        }
      }
    }
    if (scheduleFound)
    {                                                             //if schedule found
      hourAlarm = alarmTime / 3600;                               //define alarm hour
      minAlarm = (alarmTime - ((uint32_t)hourAlarm * 3600)) / 60; //define alarm minutes
      DPRINT(F("Schedule found: "));
      DPRINT(hourAlarm);
      DPRINT(F(":"));
      DPRINTLN(minAlarm);
    }
    else
    { //if any schedule is defined
      DPRINT(F("New day: "));
      newDay = true;
      alarmTime = sys.earlierStart;                                      //set alarmTime
      hourAlarm = sys.earlierStart / 3600;                               //set alarm hour from earlier start of next day
      minAlarm = (sys.earlierStart - ((uint32_t)hourAlarm * 3600)) / 60; //set alarm minutes from earlier start of next day
      for (i = 0; i < TOTAL_PROG; i++)
      { //for each program
        if (sys.interval[i] && prgPtr[i].match)
        {                                     //if interval is defined and program match
          sys.startDay[i] += sys.interval[i]; //update starting day adding interval
          if (sys.startDay[i] > 7)
            sys.startDay[i] -= 7;                 //if starting day overpass sunday, adjust it
          digitalWrite(CS_RF, HIGH);              //unselect rf
          flash.eraseSector(SYS_VAR_ADDR);        //erase sys variable sector
          flash.writeAnything(SYS_VAR_ADDR, sys); //write sys variable changes
          digitalWrite(CS_RF, LOW);
        }
      }
      DPRINT(hourAlarm);
      DPRINT(F(":"));
      DPRINTLN(minAlarm);
    }
  }
  rtc.setAlarmMode(4);                        //set alarm mod
  rtc.setAlarm(0, minAlarm, hourAlarm, 0, 0); //set alarm
  DPRINT(F("set Alarm: "));
  DPRINT(hourAlarm);
  DPRINT(F(":"));
  DPRINTLN(minAlarm);
}

/* 
  this function check if current dayOfWeek match to watering days program
*/
bool dayMatch()
{

  bool match = false;
  uint8_t i = 0, dayOfWeek;

  rtc.updateTime();
  dayOfWeek = rtc.dayOfWeek(); //get day of week
  for (i = 0; i < TOTAL_PROG; i++)
  { //for each program
    if (sys.starts[i][0] == TIME_NDEF)
      continue;               //if program not defined, jump program
    if (prgPtr[i].manual)     //if program is in manual mode
      prgPtr[i].match = true; //set match as true to run it no matter what
    else
    {
      prgPtr[i].match = false; //initialize match flag as false
      if (sys.interval[i])
      {                                   //check if interval is defined
        if (sys.startDay[i] == dayOfWeek) //if starting day is equal to current dayOfWeek
          prgPtr[i].match = true;
      }
      else
      {                                               //if interval is not define, check watering day
        (dayOfWeek) ? dayOfWeek -= 1 : dayOfWeek = 6; //if dayOfWeek is sunday set dayOfWeek index to 6, otherwise decrease 1
        if (bitRead(sys.wateringDay[i], dayOfWeek))
          prgPtr[i].match = true; //if watering day match to current dayOfWeek set match to true
      }
    }
    if (prgPtr[i].match)
      match = true;
  }
  return match;
}

/*
  this function get previous and next start according to current time
*/
void getProgramPtr()
{

  uint8_t i, j;
  uint32_t timeAux;
  bool startFound = false, valveFound = false;

  for (i = 0; i < TOTAL_PROG; i++)
  { //for each program
    if (!prgPtr[i].match)
      continue;          //if this prog doesn't match for today, jump to next prog
    prgPtr[i].start = 0; //initialize previous start
    prgPtr[i].valve = 0; //initialize valve number to actuate as not difined
    if (prgPtr[i].manual)
    {
      prgPtr[i].start = currentTime;
      startFound = true;
    }
    else
    {
      for (j = 0; j < TOTAL_START; j++)
      { //go through all start time to get previous start
        if (sys.starts[i][j] <= currentTime)
        { //if start time is less than current time
          if (sys.starts[i][j] >= prgPtr[i].start)
          {                                     //check if start time is greater than current time
            prgPtr[i].start = sys.starts[i][j]; //update previous time
            startFound = true;
          }
        }
      }
    }
    if (startFound)
    { //if privious start was found
      digitalWrite(CS_RF, HIGH);
      flash.readByteArray(IRRIG_ADDR + (i * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime)); //read irrigation time of this program from flash
      digitalWrite(CS_RF, LOW);
      timeAux = prgPtr[i].start; //initialize timeAux
      for (j = 0; j < TOTAL_VALVE; j++)
      {                          //go throught all valves
        timeAux += irrigTime[j]; //add valve irrigation time to timeAux
        if (timeAux > currentTime)
        { //if timeAux greater than current time
          if (isValveAssigned(j + 1) && irrigTime[j] != 0)
          {                          //if valve is assigned and irrigation time for this valve is defined
            prgPtr[i].action = 1;    //set action to open valve
            prgPtr[i].valve = j + 1; //set valve number
            if ((timeAux - currentTime) <= irrigTime[j])
            {                                                 //if it is an incomplete irrigation
              valveAction(prgPtr[i].valve, prgPtr[i].action); //open valve
              prgPtr[i].action = !prgPtr[i].action;           //update his state
              prgPtr[i].schedule = timeAux;                   //schedule to the next action
            }
            else
            {                                              //otherwise
              prgPtr[i].action = 1;                        //set action as close
              prgPtr[i].schedule = timeAux - irrigTime[j]; //schedule to the next action
            }
            valveFound = true;
            break;
          }
        }
      }
    }
    if (!valveFound)   //if action valve was not found
      getNextStart(i); //get valve action for next start
  }
}

/*
  this function point program to the next action
*/
void progPtrNext(uint8_t p, uint8_t v)
{

  uint8_t i;
  uint32_t timeAux;
  bool valveFound = false;

  digitalWrite(CS_RF, HIGH);
  flash.readByteArray(IRRIG_ADDR + (p * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
  digitalWrite(CS_RF, LOW);
  if (prgPtr[p].action)
  {                                         //if valve was opened
    prgPtr[p].action = 0;                   //set action as close
    prgPtr[p].schedule += irrigTime[v - 1]; //set schedule to close valve
  }
  else
  {
    timeAux = prgPtr[p].schedule; //initialize timeAux
    prgPtr[p].valve = 0;          //initialize valve number to actuate as not difined
    for (i = v; i < TOTAL_VALVE; timeAux += irrigTime[i++])
    { //go throught all valves
      if (isValveAssigned(i + 1) && (irrigTime[i] != 0))
      {                               //if valve is assigned and irrigation time for this valve is defnes
        prgPtr[p].action = 1;         //set action to open valve
        prgPtr[p].valve = i + 1;      //set valve number
        prgPtr[p].schedule = timeAux; //schedule time to next action
        if (prgPtr[p].schedule == currentTime)
        {                                                 //if schedule equal to current time
          valveAction(prgPtr[p].valve, prgPtr[p].action); //actuate in this valve
          prgPtr[p].action = !prgPtr[p].action;           //update his state
          prgPtr[p].schedule = timeAux + irrigTime[i];    //set next schedule
        }
        valveFound = true;
        break;
      }
    }
    if (!valveFound)
    { //if no valve to actuate was found in this start
      if (prgPtr[p].manual)
        prgPtr[p].manual = 0;
      getNextStart(p);                  //get next start
      if (prgPtr[p].start == TIME_NDEF) //if next start was not found
        prgPtr[p].match = false;        //set match to false
    }
  }
}

/*
  this function get next start
*/
void getNextStart(uint8_t p)
{

  uint8_t i;
  uint32_t timeAux;
  bool startFound = false;

  prgPtr[p].start = TIME_NDEF; //initialize next start
  for (i = 0; i < TOTAL_START; i++)
  { //go through all start time to get next start
    if (sys.starts[p][i] >= currentTime)
    { //if start time is greater than current time
      if (sys.starts[p][i] < prgPtr[p].start)
      {                                     //check if start time is less than next start time defined till now
        prgPtr[p].start = sys.starts[p][i]; //update next start time
        startFound = true;
      }
    }
  }
  if (startFound)
  {
    digitalWrite(CS_RF, HIGH);
    flash.readByteArray(IRRIG_ADDR + (p * SECTOR_SIZE), (uint8_t *)irrigTime, sizeof(irrigTime));
    digitalWrite(CS_RF, LOW);
    timeAux = prgPtr[p].start; //initialize timeAux
    for (i = 0; i < TOTAL_VALVE; timeAux += irrigTime[i++])
    { //go throught all valves
      if (isValveAssigned(i + 1) && irrigTime[i] != 0)
      {                               //if valve is assigned and irrigation time for this valve is defnes
        prgPtr[p].action = 1;         //set action to open valve
        prgPtr[p].valve = i + 1;      //set valve number
        prgPtr[p].schedule = timeAux; //schedule time to next action
        break;
      }
    }
  }
}

/*
  this function check if valve is assigned, return true if so and false otherwise
*/
bool isValveAssigned(uint8_t v)
{

  uint8_t i;
  for (i = 0; i < 4; i++)
  {                                //go through all valve assigned
    if (sys.assignedValve[i] == v) //check if given valve is an assigned valve
      return true;                 //if so return true
  }
  return false;
}

/*
  this function check if alarm was triggered for manvalve, if so handle it
*/
void checkManValAlarm()
{

  uint32_t timeAux;

  for (i = 0, timeAux = currentTime + rtc.getSeconds(); i < 4; i++)
  { //for each manvalve in queue
    if ((timeAux - manValve.schedule[i]) <= 5)
    {                                    //if alarm was triggered by this schedule
      valveAction(manValve.valve[i], 0); //close valve
      manValve.valve[i] = 0;             //clear this manvalve action in queue
      manValve.schedule[i] = TIME_NDEF;
      manValve.count--;
      break;
    }
  }
  if (manValve.count)
  { //if there is any manvalve in queue
    for (i = 0, timeAux = TIME_NDEF; i < 4; i++)
    { //go through all manvalve schedule and update alarmManVal to the lowest one
      if (manValve.schedule[i] != TIME_NDEF)
      {
        if (manValve.schedule[i] < timeAux)
          timeAux = manValve.schedule[i];
      }
    }
    alarmManVal = timeAux;
  }
  else
    alarmManVal = TIME_NDEF;

  ((alarmManVal != TIME_NDEF) && //if manvalve alarm is defined and
   ((alarmManVal < alarmTime) || newDay))
      ? timeAux = alarmManVal
      : timeAux = alarmTime; //if it is lower than alarmTime, set timeAux equal to alarmManVal, otherwise set alarmManVal equal to alarmTime
  hourAlarm = timeAux / 3600;
  minAlarm = (timeAux - ((uint32_t)hourAlarm * 3600)) / 60;
  secAlarm = timeAux - ((uint32_t)hourAlarm * 3600) - ((uint32_t)minAlarm * 60);
  rtc.setAlarmMode(4);
  rtc.setAlarm(secAlarm, minAlarm, hourAlarm, 0, 0);
  DPRINT(F("set Alarm: "));
  DPRINT(hourAlarm);
  DPRINT(F(":"));
  DPRINT(minAlarm);
  DPRINT(F(":"));
  DPRINTLN(secAlarm);
}

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
    if (sys.assignedValve[i] == aux)
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
}*/

/*
  this function old programs
*/
void clearProgram(uint8_t m)
{

  uint8_t i;

  sys.waterPercent[m] = 100;
  sys.wateringDay[m] = 0x7f;
  sys.interval[m] = 0;
  sys.startDay[m] = 0;
  for (i = 0; i < TOTAL_START; i++)
    sys.starts[m][i] = TIME_NDEF;
  for (i = 0; i < TOTAL_VALVE; i++)
    irrigTime[i] = 0;
  sys.earlierStart = TIME_NDEF;
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
    clearProgram(i);
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
