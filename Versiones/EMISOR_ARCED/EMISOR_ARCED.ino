#include <JamAtm-Vyrsa.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <JamSleep.h>
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
#define MAX_MANUAL_TIMERS 5
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
} sysVar;
typedef struct
{
  uint8_t interval;
  uint8_t startDay;
  uint8_t wateringDay;
  uint16_t waterPercent;
  uint8_t start[6][2];
  uint16_t irrigTime[128];
} program;
typedef struct
{
  bool timer_active;                        // When there is any timer active this flag is True
  uint8_t index;                            // For movin into the arrays
  uint8_t valve[MAX_MANUAL_TIMERS];         // Save the number of the valves
  uint32_t millix[MAX_MANUAL_TIMERS];       // Save the millis time timers are just for manual close the valves
  uint32_t time_to_stop[MAX_MANUAL_TIMERS]; // Save the millis time timers are just for manual close the valves
} manual;
#define MAX_NUM_MESSAGES 8
typedef struct
{                                        // This struct conteis the messages of radio that are going to be sent but the node is sleeping so it has to wait
  bool request_MANUAL[MAX_NUM_MESSAGES]; // the max number of messages are 4
  bool request_TIME[MAX_NUM_MESSAGES];
  bool request_ASSIGNED_VALVES[MAX_NUM_MESSAGES];
  bool request_STOP_ALL[MAX_NUM_MESSAGES];
  bool request_FULL_MESSAGE[MAX_NUM_MESSAGES];
  uint8_t num_message_flags;
  uint8_t valve_info[3][MAX_NUM_MESSAGES];    //This info is for [num oasis,  tº hours , tºminutes]
  uint8_t assigned_info[4][MAX_NUM_MESSAGES]; //This info is for assigned [num oasis, assig0, assig1, assig2, assig3]
} radio_actions;
typedef enum
{
  REQUEST_MANVAL,
  REQUEST_MANUAL,
  REQUEST_TIME,
  REQUEST_ASSIGNED_VALVES,
  REQUEST_STOP_ALL,
  REQUEST_FULL_MESSAGE
} messages_radio;

Jam jam;
sysVar sys;
manual man;
RV1805 rtc;
Sleep lowPower;
SimpleTimer timerA, timerB, timerC, timerD, timerE, timerF, timerCheck;
radio_actions radio_waitting_msg;
SPIFlash flash(CS_M);
program prog[TOTAL_PROG];
SoftwareSerial softSerial(PG_RXD, PG_TXD);

RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);

uint8_t UUID_1[] = {'A', '1'};

// Don't put this on the stack:
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
bool rf_flag = false;

String pg;
int timer_A, timer_B, timer_C, timer_D, timer_E, timer_F;
int timer_check;
char pgData[PG_MAX_LEN];
uint8_t i, j, rfId, cmd;
volatile uint8_t oldPort = 0x00;
volatile bool intButton = false, intRtc = false;
uint8_t valveDef[MAX_CHILD], progDef[TOTAL_PROG];
bool comError[MAX_CHILD];
char asignacion[] = {1, 2, 15, random(1, 127)};

bool oasis_actions;
uint32_t millix;
uint8_t index_prog_A, index_prog_B, index_prog_C, index_prog_D, index_prog_E, index_prog_F;
bool start_programA, start_programB, start_programC, start_programD, start_programE, start_programF;
bool start_programA_ones, start_programB_ones, start_programC_ones, start_programD_ones, start_programE_ones, start_programF_ones;

uint32_t start = 0;
uint8_t counter_syn = 0;
uint8_t counter_time_sms = 0;
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
  manager.init();
  manager.setRetries(1);
  manager.setTimeout(175);
  driver.setTxPower(20, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  // rtc.setAlarmMode(0);
  //rtc.setAlarm(0, 30, 0, 0, 0);
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  //rtc.setToCompilerTime();
  rtc.setAlarmMode(6);
  rtc.setAlarm(58, 0, 0, 0, 0);

  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  Serial.println(rtc.stringTime());
  Serial.println(rtc.stringDate());
  jam.ledBlink(LED_SETUP, 1000);
  timer_check = timerCheck.setInterval(20000, check_time);
  for (i = 0; i < MAX_CHILD; i++)
    comError[i] = false;
  radio_waitting_msg.num_message_flags = 0;
  for (int i = 0; i < sizeof(data); i++)
    data[i] = 'z';
  for (int msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  {
    radio_waitting_msg.request_MANUAL[msg] = false; // the max n
    radio_waitting_msg.request_TIME[msg] = false;
    radio_waitting_msg.request_ASSIGNED_VALVES[msg] = false;
    radio_waitting_msg.request_STOP_ALL[msg] = false;
    radio_waitting_msg.request_FULL_MESSAGE[msg] = false;
  }
}

void loop()
{
  /*
  Every 30 seconds I test if the irrigation time is the same as the current RTC time. The function that check this is: check_time();
  When this happend I start the program (for example A) and set a timer on: timer A.
  This timer can be overlap so we can start diferent programs at the same time, timer A, B, C, D ,E ,F.
  */
  if (Serial.available())
  {
    uint8_t a = Serial.read();
    uint8_t nodo_envio[16];
    if (a == 97)
    {
      Serial.println(millis());
      counter_syn = 5;
      prepare_message(); // This function spends 400ms to compleat
    }
  }
  /*
  When the Serial Port of the PG is set I read what happend there ant act. If there is an inmidiate action I just write an struct 
  and when is the time to send I do it
  */
  listening_pg();

  if (!digitalRead(PCINT_PIN))
  {
    Serial.println("BUTTON PRESSED");
    getAllFromPG();
  }
  if (intRtc) // I wake up at 58 seconds just for 10 seconds
  {
    intRtc = false;
    rtc.updateTime();
    Serial.println(rtc.stringTime());
    oasis_actions = true;
    millix = millis();
  }
  if (oasis_actions) // When something
  {
    // Always the first message have to be sync
    start = millis();
    while (counter_syn <= 10) // I try to send the message for 10 times, if I fail print kill me.
    {
      if (millis() - start >= 800)
      {
        prepare_message(); // This function spends 400ms to compleat
        counter_syn++;

        start = millis();
      }
      listening_pg();

      if (listen_nodo()) // When a nodo response then I am able to stop sendding because it has received correctly
      {
        for (uint8_t x = 0; x < MAX_NUM_MESSAGES; x++) // I clear all the flags of the messages beacuse I have sent it properly
        {
          radio_waitting_msg.request_MANUAL[x] = false; // the max number of messages are 4
          radio_waitting_msg.request_TIME[x] = false;
          radio_waitting_msg.request_ASSIGNED_VALVES[x] = false;
          radio_waitting_msg.request_STOP_ALL[x] = false;
          radio_waitting_msg.request_FULL_MESSAGE[x] = false;
        }
        break;
      }
    }
    counter_syn = 0;
    oasis_actions = false;
  }
  /*
  The ejecution of the programs in order. THIS PART IS A SAME AND I DO NOT CONSIDER RESPONSIBLE FOR THIS SHIT
  */
  if (start_programA)
  {
    if (prog[0].irrigTime[index_prog_A] != 255 && prog[0].irrigTime[index_prog_A] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_A + 1);
      timerA.deleteTimer(timer_A);
      timer_A = timerA.setInterval(prog[0].irrigTime[index_prog_A] * 60000, waitValveCloseA);
      uint8_t hours_temp, min_temp;
      if (prog[0].irrigTime[index_prog_A] >= 60)
      {
        hours_temp = prog[0].irrigTime[index_prog_A] / 60;
        Serial.println(hours_temp);
        min_temp = prog[0].irrigTime[index_prog_A] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[0].irrigTime[index_prog_A];
      }
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_A + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = min_temp;
      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_A + 1, hours_temp, min_temp, asignacion);
      start_programA = false;
      Serial.println("El message es: ");
      Serial.println(radio_waitting_msg.num_message_flags);
    }
    else
      index_prog_A++;
    if (index_prog_A > 127)
    {
      index_prog_A = 0;
      timerA.deleteTimer(timer_A);
      start_programA_ones = false;
      start_programA = false;
      Serial.println("TODO HA ACABADO YA - A");
    }
  }
  if (start_programB)
  {
    if (prog[1].irrigTime[index_prog_B] != 255 && prog[1].irrigTime[index_prog_B] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_B + 1);
      timerB.deleteTimer(timer_B);
      timer_B = timerB.setInterval(prog[1].irrigTime[index_prog_B] * 60000, waitValveCloseB);
      uint8_t hours_temp, min_temp;
      if (prog[1].irrigTime[index_prog_B] >= 60)
      {
        hours_temp = prog[1].irrigTime[index_prog_B] / 60;
        Serial.println(hours_temp);
        min_temp = prog[1].irrigTime[index_prog_B] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[1].irrigTime[index_prog_B];
      }
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_B + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = min_temp;

      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_B + 1, hours_temp, min_temp, asignacion);
      start_programB = false;
    }
    else
      index_prog_B++;
    if (index_prog_B > 127)
    {
      index_prog_B = 0;
      timerB.deleteTimer(timer_B);
      start_programB = false;
      start_programB_ones = false;

      Serial.println("TODO HA ACABADO YA - B");
    }
  }
  if (start_programC)
  {
    if (prog[2].irrigTime[index_prog_C] != 255 && prog[2].irrigTime[index_prog_C] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_C + 1);
      timerC.deleteTimer(timer_C);
      timer_C = timerC.setInterval(prog[2].irrigTime[index_prog_C] * 60000, waitValveCloseC);
      uint8_t hours_temp, min_temp;
      if (prog[2].irrigTime[index_prog_C] >= 60)
      {
        hours_temp = prog[2].irrigTime[index_prog_C] / 60;
        Serial.println(hours_temp);
        min_temp = prog[2].irrigTime[index_prog_C] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[2].irrigTime[index_prog_C];
      }
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_C + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = min_temp;

      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_C + 1, hours_temp, min_temp, asignacion);
      start_programC = false;
    }
    else
      index_prog_C++;
    if (index_prog_C > 127)
    {
      index_prog_C = 0;
      timerC.deleteTimer(timer_C);
      start_programC = false;
      start_programC_ones = false;

      Serial.println("TODO HA ACABADO YA - C");
    }
  }
  if (start_programD)
  {
    if (prog[3].irrigTime[index_prog_D] != 255 && prog[3].irrigTime[index_prog_D] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_D + 1);
      timerD.deleteTimer(timer_D);
      timer_D = timerD.setInterval(prog[3].irrigTime[index_prog_D] * 60000, waitValveCloseD);
      uint8_t hours_temp, min_temp;
      if (prog[3].irrigTime[index_prog_D] >= 60)
      {
        hours_temp = prog[3].irrigTime[index_prog_D] / 60;
        Serial.println(hours_temp);
        min_temp = prog[3].irrigTime[index_prog_D] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[3].irrigTime[index_prog_D];
      }
      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_D + 1, hours_temp, min_temp, asignacion);
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_D + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags] = min_temp;

      start_programD = false;
    }
    else
      index_prog_D++;
    if (index_prog_D > 127)
    {
      index_prog_D = 0;
      timerD.deleteTimer(timer_D);
      start_programD = false;
      start_programD_ones = false;

      Serial.println("TODO HA ACABADO YA - D");
    }
  }
  if (start_programE)
  {
    if (prog[4].irrigTime[index_prog_E] != 255 && prog[4].irrigTime[index_prog_E] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_E + 1);
      timerE.deleteTimer(timer_E);
      timer_E = timerE.setInterval(prog[4].irrigTime[index_prog_E] * 60000, waitValveCloseE);
      uint8_t hours_temp, min_temp;
      if (prog[4].irrigTime[index_prog_E] >= 60)
      {
        hours_temp = prog[4].irrigTime[index_prog_E] / 60;
        Serial.println(hours_temp);
        min_temp = prog[4].irrigTime[index_prog_E] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[4].irrigTime[index_prog_E];
      }
      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_E + 1, hours_temp, min_temp, asignacion);
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_E + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = min_temp;

      start_programE = false;
    }
    else
      index_prog_E++;
    if (index_prog_E > 127)
    {
      index_prog_E = 0;
      timerE.deleteTimer(timer_E);
      start_programE = false;
      start_programE_ones = false;

      Serial.println("TODO HA ACABADO YA - E");
    }
  }
  if (start_programF)
  {
    if (prog[5].irrigTime[index_prog_F] != 255 && prog[5].irrigTime[index_prog_F] != 0)
    {
      Serial.print("OPEN V");
      Serial.println(index_prog_F + 1);
      timerF.deleteTimer(timer_F);
      timer_F = timerF.setInterval(prog[5].irrigTime[index_prog_F] * 60000, waitValveCloseF);
      uint8_t hours_temp, min_temp;
      if (prog[5].irrigTime[index_prog_F] >= 60)
      {
        hours_temp = prog[5].irrigTime[index_prog_F] / 60;
        Serial.println(hours_temp);
        min_temp = prog[5].irrigTime[index_prog_F] - hours_temp * 60;
        Serial.println(min_temp);
      }
      else
      {
        hours_temp = 0;
        min_temp = prog[5].irrigTime[index_prog_F];
      }
      //send_nodo(1, UUID_1, REQUEST_MANVAL, index_prog_F + 1, hours_temp, min_temp, asignacion);
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_F + 1;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = hours_temp;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = min_temp;

      start_programF = false;
    }
    else
      index_prog_F++;
    if (index_prog_F > 127)
    {
      index_prog_F = 0;
      timerF.deleteTimer(timer_F);
      start_programF_ones = false;
      start_programF = false;
      Serial.println("TODO HA ACABADO YA - F");
    }
  }
  if (man.timer_active) // This timer start only when we have start a manual valve action and we have to stop
  {
    for (uint8_t index_manual = 0; index_manual < MAX_MANUAL_TIMERS; index_manual++)
    {
      // The first element is the time of the valve and the second element is
      if (man.millix[index_manual] != 0 && millis() - man.millix[index_manual] >= man.time_to_stop[index_manual])
      {
        Serial.print("APAGO LA VALVULA: ");
        Serial.println(man.valve[index_manual]);
        //I set a flag, when the time of waking up starts all the messages
        radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
        radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = man.valve[index_manual];
        radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = 0;
        radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = 0;
        man.index--;
        if (man.index < 1)
          man.timer_active = false;
      }
    }
  }

  /*
  When the Serial Port of the PG is set I read what happend there ant act.
  */
  timerA.run();
  timerB.run();
  timerC.run();
  timerD.run();
  timerE.run();
  timerF.run();
  timerCheck.run();
}

void prepare_message()
{
  uint8_t long_message[4]; // = "_1581603360_XNX_##STOP#ALL#00_X_##ASIGNED#000#000:000:000:000#00_X_##MANVAL#000#00:00#00_X_##STOP#ALL#00";
  for (int i = 0; i < sizeof(data) / 2; i++)
    data[i] = 'z';

  // First obtein the timestamp and save to DATA radio
  //String timestamp = String(rtc.getTimestamp()); // from string I convert to char[]
  uint16_t index = 0;
  long_message[index++] = '_';
  //for (index = 1; index < timestamp.length() + 1; index++)
  //  long_message[index] = timestamp.charAt(index - 1);
  long_message[index++] = '_';

  //Second implement a system for adding more messages to DATA radio
  //The possibilities are:
  //##ASIGNED#000#000:000:000:000#00
  //##MANVAL#000#00:00#00
  //##STOP#ALL#00

  uint8_t msg_number = 0, msg_position = 0;
  //I just copy the buffer to data radio and send it
  memcpy(data, long_message, sizeof(long_message));
  counter_time_sms++;
  Serial.println(counter_time_sms);
  if (counter_time_sms >= 10)
  {
    counter_time_sms = 0;
    send_nodo(index, UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
  }
  //Serial.println("EL ENVIO EN MILLIS COMIENZA A LAS: ");
  //Serial.println(millis());
  //2.2 introduce the messages:
  for (uint8_t msg = 0; msg < MAX_NUM_MESSAGES; msg++)
  {

    if (radio_waitting_msg.request_MANUAL[msg])
    {
      send_nodo(index, UUID_1, REQUEST_MANUAL, radio_waitting_msg.valve_info[0][msg], radio_waitting_msg.valve_info[1][msg], radio_waitting_msg.valve_info[2][msg], asignacion);
      //radio_waitting_msg.request_MANUAL[msg] = false;
    }
    else if (radio_waitting_msg.request_ASSIGNED_VALVES[msg])
    {
      char temp_assigned[] = {radio_waitting_msg.assigned_info[1][msg], radio_waitting_msg.assigned_info[2][msg], radio_waitting_msg.assigned_info[3][msg], radio_waitting_msg.assigned_info[4][msg]};
      send_nodo(index, UUID_1, REQUEST_ASSIGNED_VALVES, radio_waitting_msg.assigned_info[0][msg], 0, 0, temp_assigned);
      //radio_waitting_msg.request_ASSIGNED_VALVES[msg] = false;
    }
    else if (radio_waitting_msg.request_STOP_ALL[msg])
    {
      send_nodo(index, UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
      //radio_waitting_msg.request_STOP_ALL[msg] = false;
    }
  }
  radio_waitting_msg.num_message_flags = 0;

  manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
void check_time()
{

  //I sleep the device for the first

  //Serial.print("check_time: ");
  rtc.updateTime();
  //Serial.println(rtc.stringTime());

  for (uint8_t index_program = 0; index_program < 6; index_program++)
  {
    for (uint8_t index_time_h = 0; index_time_h < 6; index_time_h++)
    {
      if (prog[index_program].start[index_time_h][0] == rtc.getHours())
      {
        if (prog[index_program].start[index_time_h][1] == rtc.getMinutes())
        {
          Serial.println("ES LA HORA BUENA");
          switch (index_program)
          {
          case 0:
            if (!start_programA_ones)
            {
              Serial.println("Encender programa A");
              start_programA = true;
              start_programA_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa A");
            break;
          case 1:
            if (!start_programB_ones)
            {
              Serial.println("Encender programa B");
              start_programB = true;
              start_programB_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa B");
            break;
          case 2:
            if (!start_programC_ones)
            {
              Serial.println("Encender programa C");
              start_programC = true;
              start_programC_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa C");
            break;
          case 3:
            if (!start_programD_ones)
            {
              Serial.println("Encender programa D");
              start_programD = true;
              start_programD_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa D");
            break;
          case 4:
            if (!start_programE_ones)
            {
              Serial.println("Encender programa E");
              start_programE = true;
              start_programE_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa E");
            break;
          case 5:
            if (!start_programF_ones)
            {
              Serial.println("Encender programa F");
              start_programF = true;
              start_programF_ones = true;
              //timerCheck.deleteTimer(timer_check);
            }
            else
              Serial.println("Ya encendido el programa F");
            break;
          default:
            Serial.println("never");
            break;
          }
        }
      } // check if the hours are fix and we can start a program
    }
  }
}
void waitValveCloseA()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_A + 1);
  radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
  radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = index_prog_A + 1;
  radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = 0;
  radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++] = 0;
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_A + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_A++;
  start_programA = true;
}
void waitValveCloseB()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_B + 1);
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_B + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_B++;
  start_programB = true;
}
void waitValveCloseC()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_C + 1);
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_C + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_C++;
  start_programC = true;
}
void waitValveCloseD()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_D + 1);
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_D + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_D++;
  start_programD = true;
}
void waitValveCloseE()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_E + 1);
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_E + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_E++;
  start_programE = true;
}
void waitValveCloseF()
{
  Serial.print("CLOSE V");
  Serial.println(index_prog_F + 1);
  //send_nodo(UUID_1, REQUEST_MANVAL, index_prog_F + 1, 0, 0, asignacion);
  delay(1000);
  index_prog_F++;
  start_programF = true;
}
void send_nodo(uint16_t &order, uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
{
  //First write the destination of the message:
  bool f_man_valve = false, f_time = false, f_asigned = false, f_stop = false, f_manual = false, f_full = false;
  switch (msg)
  {
  case REQUEST_MANUAL:
  {
    f_manual = true;
    Serial.println("MANUAL OPEN");
    uint8_t str_manual[] = "##MANVAL#000#00:00#00";
    if (valve > 99)
    {
      str_manual[9] = '1';
      str_manual[10] = ((valve - 100) / 10) + 0x30;
      str_manual[11] = ((valve - 100) % 10) + 0x30;
    }
    else
    {
      str_manual[9] = '0';
      str_manual[10] = (valve / 10) + 0x30;
      str_manual[11] = (valve % 10) + 0x30;
    }
    str_manual[13] = (hour / 10) + 0x30;
    str_manual[14] = (hour % 10) + 0x30;
    str_manual[16] = (minutes / 10) + 0x30;
    str_manual[17] = (minutes % 10) + 0x30;
    str_manual[19] = uuid[0];
    str_manual[20] = uuid[1];
    for (int i = 0; i < sizeof(str_manual); i++)
    {
      data[order] = str_manual[i];
      order++;
    }
    break;
  }
  case REQUEST_MANVAL:
  {
    f_man_valve = true;
    Serial.println("REQUEST MANVALVE");
    uint8_t str_manval[] = "##MANVAL#000#00:00#00X";
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
    str_manval[19] = uuid[0];
    str_manval[20] = uuid[1];

    for (int i = 0; i < sizeof(str_manval); i++)
    {
      data[order] = str_manval[i];
      order++;
    }
    f_man_valve = false;
    break;
  }
  case REQUEST_TIME:
  {

    f_time = true;
    Serial.println("REQUEST TIME");
    rtc.updateTime();
    rtc_node((int)rtc.getHours(), (int)rtc.getMinutes(), (int)rtc.getSeconds(), (int)rtc.getDate(), (int)rtc.getMonth(), order);
    f_time = false;
    break;
  }
  case REQUEST_ASSIGNED_VALVES:
  {
    Serial.println("CHANGE ASIGNATION VALVE");
    f_asigned = true;
    //VALVE es el ID que va del 1 al 250
    uint8_t str_assigned[] = "##ASIGNED#000#000:000:000:000#00X";
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
    str_assigned[30] = uuid[0];
    str_assigned[31] = uuid[1];

    for (int j = 0; j < sizeof(str_assigned); j++)
    {
      data[order] = str_assigned[j];
      order++;
    }
    f_asigned = false;
    break;
  }
  case REQUEST_STOP_ALL:
  {
    Serial.println("STOP ALL");
    f_stop = true;
    f_stop = false;
    uint8_t str_stop[] = "##STOP#ALL#00X";
    str_stop[11] = uuid[0];
    str_stop[12] = uuid[1];
    for (int j = 0; j < sizeof(str_stop); j++)
    {
      data[order] = str_stop[j];
      order++;
    }
    break;
  }
  }
  //manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
void rtc_node(int hour, int minute, int second, int day, int month, uint16_t &order)
{
  uint8_t send_time[] = "##TIME:H:XX/M:XX/S:XX/D:XX/M:XX/";
  send_time[7 + 2] = (hour / 10) + 0x30;
  send_time[8 + 2] = (hour % 10) + 0x30;
  send_time[12 + 2] = (minute / 10) + 0x30;
  send_time[13 + 2] = (minute % 10) + 0x30;
  send_time[17 + 2] = (second / 10) + 0x30;
  send_time[18 + 2] = (second % 10) + 0x30;
  send_time[22 + 2] = (day / 10) + 0x30;
  send_time[23 + 2] = (day % 10) + 0x30;
  send_time[27 + 2] = (month / 10) + 0x30;
  send_time[28 + 2] = (month % 10) + 0x30;
  for (int i = 0; i < sizeof(send_time); i++)
    data[order++] = send_time[i];
  //for (int i = 0; i < sizeof(send); i++)
  //  Serial.write(send[i]);
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
bool listen_nodo() // if the oasis send something i listen
{
  if (manager.available()) // Detect radio activity
  {
    uint8_t len = sizeof(buf);
    manager.recvfromAck(buf, &len);
    Serial.println("NODE_RECEIVED");
    //for (uint8_t i = 0; i < 70; i++) //loop from the buffer looking for the end of message
    //  Serial.write(buf[i]);
    return true;
  }
  return false;
}
void rtcInt() //this callback funtion is called when rtc interrupt is triggered
{
  intRtc = true; //set flag to indicate that rtc interrupt was triggered
}
void memmoryHandler(uint8_t pos, bool sendChange) //this function read memmory in a given range of address - It is not changed
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
void getAllFromPG() //this function get all data from PG
{
  bool defined;
  uint8_t i, m, n, v;
  int index, addrInt;
  String aux, addr = "200";
  uint8_t bookEnd = ceil((double)(MAX_CHILD * 4) / 64);

  pgCommand(cmd_write_flag, sizeof(cmd_write_flag)); //clear memmory flag
  //getPGTime(true);                                   //read date and time
  memmoryHandler(3, false); //read arranque
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
bool sendCommand(uint8_t data[], uint8_t len, uint8_t id) //this function get PG date and time
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
void emiter_to_sleep()
{
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
      Serial.print(valve_number);
      Serial.print(" valvula - ");
      Serial.print(valve_time_hour);
      Serial.print(" horas - ");
      Serial.print(valve_time_min);
      Serial.println(" minutos. ");
      //start_programA = true;

      //I set a flag, when the time of waking up starts all the messages
      radio_waitting_msg.request_MANUAL[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags] = valve_number;
      radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags] = valve_time_hour;
      radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags] = valve_time_min;

      //Serial.println("El valor de la radio waitting ha cambiado");
      //Serial.println(radio_waitting_msg.valve_info[0][radio_waitting_msg.num_message_flags]);
      //Serial.println(radio_waitting_msg.valve_info[1][radio_waitting_msg.num_message_flags]);
      //Serial.println(radio_waitting_msg.valve_info[2][radio_waitting_msg.num_message_flags++]);

      man.timer_active = true;
      man.millix[man.index] = millis();
      man.time_to_stop[man.index] = 1000 * ((uint32_t)valve_time_hour * 3600 + (uint32_t)valve_time_min * 60);
      man.valve[man.index] = valve_number;
      Serial.print("El valor en ms que se maneja por aquí es de: ");
      Serial.println(man.time_to_stop[man.index]);
      Serial.print("En la válvula: ");
      Serial.println(man.valve[man.index]);
      man.index++;

      //send_nodo(1, UUID_1, REQUEST_MANUAL, valve_number, valve_time_hour, valve_time_min, asignacion);
    }
    else if (pg.indexOf("MANPRG START#") > 0)
    {
      uint8_t prog_name = (pgData[pg.indexOf("MANPRG START#") + 13] - 65);
      Serial.print("Programa:");
      Serial.println(prog_name);
      switch (prog_name)
      {
      case 0:
        start_programA = true;
        break;
      case 1:
        start_programB = true;
        break;
      case 2:
        start_programC = true;
        break;
      case 3:
        start_programD = true;
        break;
      case 4:
        start_programE = true;
        break;
      case 5:
        start_programF = true;
        break;
      default:
        break;
      }
    }
    else if (pg.indexOf("STOP ALL") > 0)
    {
      Serial.println("Paro TODO");
      start_programA = false;
      start_programA_ones = false;
      start_programB = false;
      start_programB_ones = false;
      start_programC = false;
      start_programC_ones = false;
      start_programD = false;
      start_programD_ones = false;
      radio_waitting_msg.request_STOP_ALL[radio_waitting_msg.num_message_flags++] = true;
      //send_nodo(1, UUID_1, REQUEST_STOP_ALL, 0, 0, 0, asignacion);
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

      //send_nodo(1, UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
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
        //Serial.println(oasis_valves[k]);
      }
      temp_valve[0] = (char)oasis_valves[0];
      temp_valve[1] = (char)oasis_valves[1];
      temp_valve[2] = (char)oasis_valves[2];
      temp_valve[3] = (char)oasis_valves[3];

      //I set the flag for sendding the messages:

      radio_waitting_msg.request_ASSIGNED_VALVES[radio_waitting_msg.num_message_flags] = true;
      radio_waitting_msg.assigned_info[0][radio_waitting_msg.num_message_flags] = (char)valve_number_true;
      radio_waitting_msg.assigned_info[1][radio_waitting_msg.num_message_flags] = temp_valve[0];
      radio_waitting_msg.assigned_info[2][radio_waitting_msg.num_message_flags] = temp_valve[1];
      radio_waitting_msg.assigned_info[3][radio_waitting_msg.num_message_flags] = temp_valve[2];
      radio_waitting_msg.assigned_info[4][radio_waitting_msg.num_message_flags] = temp_valve[3];
      for (uint8_t k = 0; k < 5; k++)
        Serial.println(radio_waitting_msg.assigned_info[k][radio_waitting_msg.num_message_flags]);
      radio_waitting_msg.num_message_flags++;
      //send_nodo(1, UUID_1, REQUEST_ASSIGNED_VALVES, valve_number_true + 1, 0, 0, temp_valve);
    }
    else if (pg.indexOf("SELECTOR#00") > 0)
    {
      Serial.println("AUTO");
      //getAllFromPG();
    }
  }
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
