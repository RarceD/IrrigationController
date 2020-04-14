#include "Oasis_RarceD.h"
// #include <SimpleTimer.h>

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

#define TX_PWR 20
#define CLIENT_ADDRESS 4
#define SERVER_ADDRESS 3

#define DEAD_TIME_COUNTER 5  //if I lose 20 packets I am dead and I close all the valves I have
#define AWAKE_TIME_COUNTER 2 //if I do not receive 3 packets I awake 1 minute compleat just one time
#define AWAKE_TIME_PER_MIN 2000

#define FLASH_SYS_DIR 0x040400

typedef enum
{
  MANVAL_MSG = 'M',
  TIME_MSG = 'T',
  ASSIGNED_MSG = 'A',
  STOP_MSG = 'S'
} msg_receive;

typedef struct
{
  uint8_t id;                 // This is the unique ID, there are 250 units so we can fix this number for identify the net
  uint8_t assigned_output[4]; // There are 4 output valves
  uint8_t master_id[2];
  uint8_t ack_msg[8];
} sysVar;
typedef struct
{
  bool valves_on[4];
  bool secure_close; //If I do not receive msg from the emiter in 5 minutes I switch off all the valves
  bool just_one_time_awake_1_min;
  bool send_ack; //When I lost connection for 20 minutes I do not send ack more
  uint8_t counter_secure_close;
} valve_status;
typedef enum
{
  MODE_FIRST_SYN,
  MODE_SLEEP,
  MODE_AWAKE,
  MODE_ACK
} STATE_MACHINE;
Jam jam;
typedef enum
{
  ACK,
  FAULT,
  SENSORS
} msg_send;
sysVar sys;
valve_status v;
msg_send msg_sendd;
STATE_MACHINE state_machine;

RV1805 rtc;
Sleep lowPower;

SPIFlash flash(CS_M);
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

volatile bool intButton, intRtc, Global_Flag_int;
uint16_t Set_Vshot = 600;
uint8_t iOpen = 0, valveOpened[4];
uint8_t data[20];

/******************************************************************* setup section ************************************************************************************/

bool first_start_syn = true;
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
  // I have to change the flash info for each devise:
  /*
  sys.id = 2;
  sys.master_id[0] = 'A';
  sys.master_id[1] = '3';
  sys.assigned_output[0] = 1;
  sys.assigned_output[1] = 2;
  sys.assigned_output[2] = 3;
  sys.assigned_output[3] = 4;
  char ack[] = "##OK02##";
  for (int i = 0; i < sizeof(ack); i++)
    sys.ack_msg[i] = ack[i]; 
  flash.eraseSector(FLASH_SYS_DIR);
  flash.writeAnything(FLASH_SYS_DIR, sys);
  */
  flash.readAnything(FLASH_SYS_DIR, sys);
  manager.init();
  driver.setPreambleLength(8);
  driver.setTxPower(TX_PWR, false);
  manager.setRetries(8); // I change this value but I don`t now what is the best
  // This value should be random for not overlapping
  uint8_t time_retries = 120;
  manager.setTimeout(time_retries);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  rtc.setAlarmMode(6);
  rtc.setAlarm(0, 0, 0, 0, 0);
  // rtc.setToCompilerTime();
  // For disable the interrupt : //rtc.setAlarmMode(0);
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  attachPCINT(digitalPinToPCINT(SW_SETUP), buttonInt, FALLING);
  chargeCapacitor();
  ledBlink(LED_SETUP, 1000);
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  print_flash();
  //First of all close all the valves just in case:
  for (uint8_t index = 0; index < 4; index++)
  {
    v.valves_on[index] = false;
    // valveAction(index + 1, v.valves_on[index]); // Turn On or OFF a valve
    delay(1000);
  }
  v.counter_secure_close = DEAD_TIME_COUNTER;
  v.just_one_time_awake_1_min = true;
  v.send_ack = true;
  DPRINTLN(freeRam());
  state_machine = MODE_FIRST_SYN;
}
/******************************************************************* main program  ************************************************************************************/
uint8_t millix;
bool to_sleep;
void loop()
{
  /*
  The first time you plug we have to wait received the time to sincronize  
  
 
    if (MODE_AWAKE)
    {
      if (manager.available()) // Detect radio activity
      {
        uint8_t len = sizeof(buf);
        manager.recvfromAck(buf, &len);
        listen_master(); //When activity is detected listen the master
        to_sleep = true;
        //for (int i = 0; i < sizeof(buf); i++)
        //  Serial.write(buf[i]);
        v.just_one_time_awake_1_min = true;
        v.counter_secure_close = DEAD_TIME_COUNTER;
        delay(10);
      }
      if (millis() - millix >= AWAKE_TIME_PER_MIN || to_sleep) // It is awake for 2 seconds
      {
        DPRINTLN(" A dormir");
        //The oasis has lost the radio wave and the sync and I close just in case I flood something
        if (v.counter_secure_close >= 1)
        {
          //If I lost connection for 3 min I awakae for 1 min and then everithing the sme
          if (v.counter_secure_close == DEAD_TIME_COUNTER - AWAKE_TIME_COUNTER && v.just_one_time_awake_1_min)
          {
            DPRINTLN("I AM LOST AND I AWAKE FOR 1 MIN");
            uint64_t millix_awake_for_1_min = millis();
            bool connected = false;
            while (millis() - millix_awake_for_1_min <= 60000) //In case I have not receive a packet in 3 min
            {
              if (manager.available()) // Detect radio activity and set a timer for waking up at 00
              {
                uint8_t len = sizeof(buf);
                manager.recvfromAck(buf, &len);
                listen_master(); //When activity is detected listen the master
                v.counter_secure_close = DEAD_TIME_COUNTER;
                v.send_ack = true;
                connected = true;
                delay(500);
                break;
              }
              jam.ledBlink(LED_SETUP, 100);
            }
            DPRINTLN("1 min done, I am lost");
            if (connected)
              v.just_one_time_awake_1_min = true;
            else
              v.just_one_time_awake_1_min = false; //THIS TRY JUST ONE TIME TO KEEP THE BATTERY ON
            MODE_AWAKE = false;                    //I go to sleep
            intRtc = false;
          }
          v.counter_secure_close--;
        }
        else
        {
          //When the counter == 0;
          //If I have not connected in 20 times I just close all the valves that are open
          DPRINTLN("I CLOSE ALL THE VALVES BECAUSE I'VE NOT RECEIVED IN 20 MIN");
          v.counter_secure_close = DEAD_TIME_COUNTER;
          for (uint8_t index = 0; index < 4; index++)
            if (v.valves_on[index])
              valveAction(index + 1, false); // Turn On or OFF a valve
          v.send_ack = false;
        }
        to_sleep = false;
        MODE_AWAKE = false;
        SEND_ACK = true;
        delay(10);
      }



    }
  }
*/
  switch (state_machine)
  {
  case MODE_FIRST_SYN:
    if (manager.available()) // Detect radio activity and set a timer for waking up at 00
    {
      DPRINTLN("MODE_FIRST_SYNC");
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (manager.recvfromAck(buf, &len))
        listen_master(); //This function change the state machine to SLEEP
    }
    ledBlink(LED_SETUP, 100);
    break;
  case MODE_SLEEP:
    driver.sleep();
    lowPower.sleep_delay(100);
    break;
  case MODE_AWAKE:
    if (manager.available()) // Detect radio activity and set a timer for waking up at 00
    {
      DPRINTLN("MODE_AWAKE");
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (manager.recvfromAck(buf, &len))
        listen_master(); //This function change the state machine to SLEEP
      to_sleep = true;
    }
    if (millis() - millix >= AWAKE_TIME_PER_MIN || to_sleep) // It is awake for 2 seconds
    {
      Serial.println("to_sleep_after_awake");
      rtc.setAlarmMode(6);
      rtc.setAlarm(30 + sys.id - 1, 0, 0, 0, 0);
      state_machine = MODE_SLEEP;
      delay(1);
    }
    break;
  case MODE_ACK:
    Serial.println("MODE_ACK");
    rtc.setAlarmMode(6);
    rtc.setAlarm(0, 0, 0, 0, 0);
    send_master(ACK);
    state_machine = MODE_SLEEP;
    delay(1);
    break;
  default:
    Serial.println("MODE DEAD");
    break;
  }
  if (intRtc) //The rtc interrupt is triggered and there are 2 options
  {
    intRtc = false;
    rtc.updateTime();
    DPRINTLN(rtc.stringTime());
    //I decide which state is the one:
    if (rtc.getSeconds() < 10)
    {
      state_machine = MODE_AWAKE;
      millix = millis(); // I have to be in AWAKE_MODE for 2 seconds
    }
    else
      state_machine = MODE_ACK;
  }
}

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
uint8_t batLevel() // return the battery level
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
void valveAction(uint8_t Valve, boolean Dir) // Turn On or OFF a valve
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
    if (i == Valve - 1)
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
      DPRINT(F("Salida: "));
      DPRINTLN(i + 1);
      DPRINT(F("Time Capacitor (ms):"));
      DPRINTLN(a * 30 + 64);
      DPRINT(F("Voltage:"));
      DPRINTLN(Vshot * 0.0107);
      nAction++;
    }
  }
  for (i = 0; i < 4; i++)
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
  if (Dir && !defined)
  {
    valveOpened[iOpen] = aux;
    iOpen++;
  }
  delay(5);
}
void listen_master() // Listen and actuate in consideration
{
  DPRINTLN("He recibido del master: ");
  uint8_t start_msg_letter[] = "AAAA"; //The max number of messages in buffer is 4 because why not?
  uint8_t index_start_msg = 0;
  // I first find the number of msg and the position of the first letter of them
  // I save them on start_msg_letter[]
  for (int i = 0; i < sizeof(buf); i++)
  {
    if (buf[i] == '#')
      if (buf[i + 1] == '#')
        start_msg_letter[index_start_msg++] = i + 2; // I find the number of messages in the buffer and also the position of start
    if (buf[i] == 'z')
      if (buf[i + 1] == 'z')
        break;
  }
  bool is_for_me = false;
  uint8_t buffer_index = 0; // This variable is for offset what I am doing
  // I execute all the actions saved in the buffer start_msg_letter[]
  while (index_start_msg > 0)
  {
    switch (buf[start_msg_letter[--index_start_msg]])
    {
    case MANVAL_MSG:
    {
      DPRINTLN("VALVE ACTION");
      //##MANVAL#002#00:10#A1#MAN
      //--012345678901234567890123456
      DPRINTLN("");

      if (buf[start_msg_letter[index_start_msg] + 17] == sys.master_id[0] && buf[start_msg_letter[index_start_msg] + 18] == sys.master_id[1])
      {
        buffer_index = start_msg_letter[index_start_msg] + 7;
        uint8_t valve_action = (buf[buffer_index] - '0') * 100 + (buf[buffer_index + 1] - '0') * 10 + (buf[buffer_index + 2] - '0');
        uint8_t valve_time_hours = (buf[buffer_index + 4] - '0') * 10 + (buf[buffer_index + 5] - '0');
        uint8_t valve_time_minutes = (buf[buffer_index + 7] - '0') * 10 + (buf[buffer_index + 8] - '0');
        DPRINT("Valve action: ");
        DPRINT(valve_action);
        DPRINT(" time: ");
        DPRINT(valve_time_hours);
        DPRINT(":");
        DPRINTLN(valve_time_minutes);

        for (int i = 0; i < 4; i++) // I test if the message is for me and I open, or close the valve.
          if (sys.assigned_output[i] == valve_action)
            if (valve_time_hours == 0 && valve_time_minutes == 0)
            { //I close the valve
              if (v.valves_on[i])
              {
                v.valves_on[i] = false;
                valveAction(i + 1, false);
              }
            }
            else
            { //I open the valve
              if (!v.valves_on[i])
              {
                valveAction(i + 1, true);
                v.valves_on[i] = true;
              }
            }
      }
      break;
    }
    case TIME_MSG:
    {
      //uint8_t send[] = "##TIME:H:XX  /M:XX/S: XX/D:XX /M:XX/";
      //uint8_t sead[] = "01234567890  12345678 9012345 67890";
      buffer_index = start_msg_letter[index_start_msg] + 8;
      int hours, minutes, day, month, year, seconds;
      if (buf[buffer_index - 1] == '0')
        hours = buf[buffer_index] - '0';
      else
        hours = (buf[buffer_index - 1] - '0') * 10 + (buf[buffer_index] - '0');
      if (buf[buffer_index + 4] == '0')
        minutes = buf[buffer_index + 5] - '0';
      else
        minutes = (buf[buffer_index + 4] - '0') * 10 + (buf[buffer_index + 5] - '0');
      if (buf[buffer_index + 9] == '0')
        seconds = buf[buffer_index + 10] - '0';
      else
        seconds = (buf[buffer_index + 9] - '0') * 10 + (buf[buffer_index + 10] - '0');
      if (buf[buffer_index + 14] == '0')
        day = buf[buffer_index + 15] - '0';
      else
        day = (buf[buffer_index + 14] - '0') * 10 + (buf[buffer_index + 15] - '0');
      if (buf[buffer_index + 19] == '0')
        month = buf[buffer_index + 20] - '0';
      else
        month = (buf[buffer_index + 19] - '0') * 10 + (buf[buffer_index + 20] - '0');
      if (seconds < 53 && seconds > 1)
        seconds -= 1;
      change_time(hours, minutes, day, month, seconds, 2020);
      state_machine = MODE_SLEEP; //I change the state of the machine
      ledBlink(LED_SETUP, 500);   //A led ON to realize that I it es continously receiving
      break;
    }
    case ASSIGNED_MSG:
    {
      DPRINTLN("ASSIGNED IN PROGRESS");

      //##ASIGNED#720#045:099:004:035#00
      buffer_index = start_msg_letter[index_start_msg] + 8;
      Serial.write(buf[buffer_index - 1]);
      Serial.write(buf[buffer_index + 21]);
      Serial.write(buf[buffer_index + 20]);

      //DPRINTLN("");
      uint8_t id_msg = (buf[buffer_index] - '0') * 100 + (buf[buffer_index + 1] - '0') * 10 + (buf[buffer_index + 2] - '0') + 1;
      //  I test if the message is for me checking the unique ID
      // This only is used when the assignation message is sent
      if (sys.id == id_msg && buf[buffer_index + 20] == sys.master_id[0] && buf[buffer_index + 21] == sys.master_id[1])
        is_for_me = true;
      else
        is_for_me = false;
      DPRINT("Assigned done in id: ");
      DPRINT(id_msg);
      DPRINT(" outputs: ");
      uint8_t offset_msg = 0;
      uint8_t out[4];
      for (uint8_t msg_index = 0; msg_index < 4; msg_index++, offset_msg += 4)
      {
        out[msg_index] = (buf[buffer_index + 4 + offset_msg] - '0') * 100 + (buf[buffer_index + 5 + offset_msg] - '0') * 10 + (buf[buffer_index + 6 + offset_msg] - '0');
        DPRINT(out[msg_index] + 1);
        DPRINT(" ");
      }
      // If the node ID is the same as saved then execute
      if (is_for_me)
      {
        DPRINTLN("Es para mi, hago la asignación");
        sys.assigned_output[0] = out[0] + 1;
        sys.assigned_output[1] = out[1] + 1;
        sys.assigned_output[2] = out[2] + 1;
        sys.assigned_output[3] = out[3] + 1;

        digitalWrite(CS_RF, HIGH); //unselect rf
        flash.eraseSector(FLASH_SYS_DIR);
        flash.writeAnything(FLASH_SYS_DIR, sys);
        digitalWrite(CS_RF, LOW);
      }
      break;
    }
    case STOP_MSG:
    {
      //##STOP#ALL#00
      buffer_index = start_msg_letter[index_start_msg];
      if (buf[buffer_index + 9] == sys.master_id[0] && buf[buffer_index + 10] == sys.master_id[1])
      {
        for (int i = 0; i < 4; i++) // I test if the message is for me and I open, or close the valve.
          if (v.valves_on[i])
          {
            v.valves_on[i] = false;
            valveAction(i + 1, false);
          }
      }
      else
      {
        DPRINTLN("Esto no es de mi master, es para el: ");
        Serial.write(buf[buffer_index + 9]);
        Serial.write(buf[buffer_index + 10]);
      }
      break;
    }
    default:
      DPRINTLN("NO TIENE QUE SALIR");
    }
  }
}
void send_master(uint8_t msg) // I just have to send the flash info for getting the ack
{
  if (msg == ACK)
  {
    // DPRINTLN("I send ack to master");
    for (int i = 0; i < sizeof(sys.ack_msg); i++)
      data[i] = sys.ack_msg[i];
  }
  uint8_t batery_level = batLevel();
  data[9] = (batery_level / 10) + '0';
  data[10] = (batery_level % 10) + '0';
  manager.sendtoWait(data, 15, SERVER_ADDRESS);
}
void change_time(int hours, int minutes, int day, int month, int seconds, int year)
{
  rtc.set24Hour();
  uint8_t currentTime[8];
  currentTime[0] = rtc.DECtoBCD(0);
  currentTime[1] = rtc.DECtoBCD(seconds);
  currentTime[2] = rtc.DECtoBCD(minutes);
  currentTime[3] = rtc.DECtoBCD(hours);
  currentTime[4] = rtc.DECtoBCD(day);
  currentTime[5] = rtc.DECtoBCD(month);
  currentTime[6] = rtc.DECtoBCD(year - 2000);
  currentTime[7] = rtc.DECtoBCD(0);
  rtc.setTime(currentTime, TIME_ARRAY_LENGTH);
  DPRINTLN("TIME CHANGE");
}
void rtcInt()
{
  DPRINTLN("RTC_INT");
  intRtc = true;
}
void buttonInt()
{
  DPRINTLN("BUTTON PRESSED");
  ledBlink(LED_SETUP, 1000);
  first_start_syn = true;
  // I set the routine to start and syncronize
}
void print_flash()
{
  DPRINT("The ID value is: ");
  DPRINTLN(sys.id);
  DPRINT("The assignated valves are: ");
  DPRINT(sys.assigned_output[0]);
  DPRINT(", ");
  DPRINT(sys.assigned_output[1]);
  DPRINT(", ");
  DPRINT(sys.assigned_output[2]);
  DPRINT(", ");
  DPRINT(sys.assigned_output[3]);
  DPRINTLN(" ");
  DPRINT("The master UUID is: ");
  Serial.write(sys.master_id[0]);
  DPRINT(" ");
  Serial.write(sys.master_id[1]);
  DPRINTLN(" ");
  DPRINT("The msg of ack is: ");
  for (int i = 0; i < sizeof(sys.ack_msg); i++)
    Serial.write(sys.ack_msg[i]);
  DPRINTLN(" ");
}
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}