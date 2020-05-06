#include "Oasis_RarceD.h"

/******************************************************************* debug ********************************************************************************************/
#define DEBUG_ON
#ifdef DEBUG_ON
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

/******************************************************************* declarations  ************************************************************************************/
typedef enum
{
  MANVAL_MSG = 'M',
  TIME_MSG = 'T',
  ASSIGNED_MSG = 'A',
  STOP_MSG = 'S'
} msg_receive;
typedef enum
{
  MODE_FIRST_SYN,
  MODE_SLEEP,
  MODE_AWAKE,
  MODE_ACK
} STATE_MACHINE;
typedef enum
{
  ACK,
  FAULT,
  SENSORS
} msg_send;
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
  bool secure_awake;
  bool send_ack; //When I lost connection for 20 minutes I do not send ack more
  bool receive_time;
  uint8_t counter_secure_close;
} valve_status;

sysVar sys;
valve_status v;
uint8_t state_machine;

RV1805 rtc;
Sleep lowPower;
SPIFlash flash(CS_M);
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

volatile bool intButton, intRtc, Global_Flag_int;

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
  sys.id = 1;
  sys.master_id[0] = 'A';
  sys.master_id[1] = '2';
  sys.assigned_output[0] = 1;
  sys.assigned_output[1] = 2;
  sys.assigned_output[2] = 3;
  sys.assigned_output[3] = 4;
  char ack[] = "##OK01##";
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
  uint8_t time_retries = 100;
  manager.setTimeout(time_retries);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  // rtc.setAlarmMode(6);
  // rtc.setAlarm(0, 0, 0, 0, 0);
  // rtc.setToCompilerTime();
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
    v.valves_on[index] = false;

  DPRINTLN(freeRam());
  state_machine = MODE_FIRST_SYN;
}
/******************************************************************* main program  ************************************************************************************/
uint32_t millix;
bool to_sleep;
void loop()
{
  switch (state_machine)
  {
  case MODE_FIRST_SYN:
  {
    if (driver.available()) // Detect radio activity and set a timer for waking up at 00
    {
      DPRINTLN("MODE_FIRST_SYNC");
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      v.receive_time = true;      //I only receive the time ones
      if (driver.recv(buf, &len)) //Just one time at start
      {
        // listen_master(buf);       //This function change the state machine to SLEEP
        uint8_t uuid_master[] = "A1";
        uint8_t index_msg = 0;
        //First I check if this msg is in my net
        if (from_my_network(uuid_master, buf, index_msg))
          for (; index_msg < 150; index_msg++)
          {
            if (buf[index_msg] == '#')
            {
              if (buf[index_msg + 1] == '#') //I have found a msg:
              {
                uint8_t buffer_index = 0; // This variable is for offset what I am doing
                switch (buf[index_msg + 2])
                {
                case TIME_MSG:
                {
                  //uint8_t send[] = "##TIME:H:XX  /M:XX/S: XX/D:XX /M:XX/";
                  //uint8_t sead[] = "01234567890  12345678 9012345 67890";
                  buffer_index = index_msg + 10;
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
                  change_time(hours, minutes, 1, 2, seconds, 20);
                  state_machine = MODE_SLEEP;
                  DPRINTLN("TIEMPO CAMBIADO A: ");
                  DPRINTLN(rtc.stringTime());
                  break;
                }
                default:
                  DPRINT("");
                }
              }
            }
            // if (buf[index_msg] == '_')
            // break;
          }
        //for (int i = 0; i < 180; i++)
        //  Serial.write(buf[i]);
      }
      intRtc = 0;
      v.counter_secure_close = DEAD_TIME_COUNTER;
      v.send_ack = true;      //Continue sending ack to emiter
      v.receive_time = false; //No more time change
      v.secure_close = true;  //I can close all the valves ones
      v.secure_awake = true;
      to_sleep = false;
    }
    ledBlink(LED_SETUP, 100);
    delay(100);
    break;
  }
  case MODE_SLEEP:
    driver.sleep();
    lowPower.sleep_delay(100);
    break;
  case MODE_AWAKE:
    if (driver.available()) // Detect radio activity and set a timer for waking up at 00
    {
      DPRINTLN("Received");
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (driver.recv(buf, &len))
      {
        // listen_master(buf);     //This function change the state machine to SLEEP
        uint8_t uuid_master[] = "A1";
        uint8_t index_msg = 0;
        //First I check if this msg is in my net
        if (from_my_network(uuid_master, buf, index_msg))
        {
          for (; index_msg < 150; index_msg++)
          {
            if (buf[index_msg] == '#')
            {
              if (buf[index_msg + 1] == '#') //I have found a msg:
              {
                uint8_t buffer_index = 0; // This variable is for offset what I am doing
                switch (buf[index_msg + 2])
                {
                case TIME_MSG:
                {
                  //uint8_t send[] = "##TIME:H:XX  /M:XX/S: XX/D:XX /M:XX/";
                  //uint8_t sead[] = "01234567890  12345678 9012345 67890";
                  buffer_index = index_msg + 10;
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
                  rtc.updateTime();
                  // int my_time = rtc.getSeconds();
                  // if (my_time != seconds)
                  // {
                  //if (seconds < 60)
                  //  seconds++;
                  //I received the time and change
                  rtc.set24Hour();
                  uint8_t currentTime[8];
                  currentTime[0] = rtc.DECtoBCD(0);
                  currentTime[1] = rtc.DECtoBCD(seconds);
                  currentTime[2] = rtc.DECtoBCD(minutes);
                  currentTime[3] = rtc.DECtoBCD(hours);
                  currentTime[4] = rtc.DECtoBCD(1);
                  currentTime[5] = rtc.DECtoBCD(1);
                  currentTime[6] = rtc.DECtoBCD(20);
                  currentTime[7] = rtc.DECtoBCD(0);
                  rtc.setTime(currentTime, TIME_ARRAY_LENGTH);
                  DPRINTLN("TIME CHANGE");
                  DPRINTLN(rtc.stringTime());
                  //rtc.setAlarmMode(6);
                  //rtc.setAlarm(30, 0, 0, 0, 0);
                  // state_machine = MODE_SLEEP;
                  // }
                  break;
                }
                case ASSIGNED_MSG:
                {
                  //##ASIGNED#720#045:099:004:035#00
                  buffer_index = index_msg + 10;
                  uint8_t id_msg = (buf[buffer_index] - '0') * 100 + (buf[buffer_index + 1] - '0') * 10 + (buf[buffer_index + 2] - '0') + 1;
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
                  DPRINTLN(" ");
                  break;
                }
                case MANVAL_MSG:
                {
                  //##MANVAL#002#00:10#A1#MAN
                  //--012345678901234567890123456
                  buffer_index = index_msg + 9;
                  uint8_t valve_action = (buf[buffer_index] - '0') * 100 + (buf[buffer_index + 1] - '0') * 10 + (buf[buffer_index + 2] - '0');
                  uint8_t valve_time_hours = (buf[buffer_index + 4] - '0') * 10 + (buf[buffer_index + 5] - '0');
                  uint8_t valve_time_minutes = (buf[buffer_index + 7] - '0') * 10 + (buf[buffer_index + 8] - '0');
                  DPRINT("Valve action: ");
                  DPRINT(valve_action);
                  DPRINT(" time: ");
                  DPRINT(valve_time_hours);
                  DPRINT(":");
                  DPRINTLN(valve_time_minutes);
                  break;
                }
                case STOP_MSG:
                {
                  //##STOP#ALL#00
                  for (int i = 0; i < 4; i++) // I test if the message is for me and I open, or close the valve.
                    if (v.valves_on[i])
                    {
                      v.valves_on[i] = false;
                      valveAction(i + 1, false);
                    }
                  break;
                }
                default:
                  DPRINTLN(" ");
                }
              }
            }
          }
        }
      }
      //for (int i = 0; i < 180; i++)
      //  Serial.write(buf[i]);

      v.receive_time = false;   //I only receive the time ones
      ledBlink(LED_SETUP, 500); //A led ON to realize that I it es continously receiving
      to_sleep = true;
      v.send_ack = true;     //Continue sending ack to emiter
      v.secure_close = true; //I can close all the valves ones
      v.counter_secure_close = DEAD_TIME_COUNTER;
      v.secure_awake = true;
    }
    if (millis() - millix >= AWAKE_TIME_PER_MIN || to_sleep) // It is awake for 2 seconds
    {
      DPRINTLN("to_sleep");
      DPRINTLN("");
      rtc.setAlarmMode(6);
      rtc.setAlarm(30 + sys.id - 1, 0, 0, 0, 0);
      state_machine = MODE_SLEEP;
      if (!to_sleep)
      {
        DPRINTLN("Lost sync");
        DPRINTLN(" ");
        v.receive_time = true; //I only receive the time ones
        if (v.counter_secure_close == DEAD_TIME_COUNTER - AWAKE_TIME_COUNTER && v.secure_awake)
        {
          v.secure_awake = false;
          v.send_ack = false; //not sendding sending ack to emiter
          uint32_t time_awake = millis();
          DPRINTLN("1 min awake");
          while (millis() - time_awake < 60000)
          {
            if (driver.available())
            { // Detect radio activity and set a timer for waking up at 00
              uint8_t buf[150];
              uint8_t len = sizeof(buf);
              v.receive_time = true;      //I only receive the time ones
              if (driver.recv(buf, &len)) //Just one time at start
                listen_master(buf);       //This function change the state machine to SLEEP
              intRtc = 0;
              v.counter_secure_close = DEAD_TIME_COUNTER;
              v.send_ack = true;      //Continue sending ack to emiter
              v.receive_time = false; //No more time change
              v.secure_close = true;  //I can close all the valves ones
              v.secure_awake = true;
              break;
            }
            ledBlink(LED_SETUP, 100);
          }
        }
        if (v.counter_secure_close > 0)
          v.counter_secure_close--;
        else
        {
          v.counter_secure_close = DEAD_TIME_COUNTER;
          if (v.secure_close)
          {
            v.secure_close = false;
            DPRINTLN("I close all the valves");
            for (uint8_t times = 0; times < 4; times++)
              if (v.valves_on[times])
              {
                valveAction(times, false);
                delay(1000);
              }
          }
        }
      }
      to_sleep = false;
      delay(10);
    }
    break;
  case MODE_ACK:
    rtc.setAlarmMode(6);
    rtc.setAlarm(0, 0, 0, 0, 0);
    if (v.send_ack)
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
      DPRINTLN("MODE_AWAKE");
      millix = millis(); // I have to be in AWAKE_MODE for 2 seconds
      state_machine = MODE_AWAKE;
    }
    else
    {
      DPRINTLN("MODE_ACK");
      state_machine = MODE_ACK;
    }
  }
}

void chargeCapacitor()
{
  uint16_t Set_Vshot = 600;
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
  uint8_t iOpen = 0, valveOpened[4];
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
void listen_master(uint8_t buf[]) // Listen and actuate in consideration
{
  // DPRINTLN("He recibido del master: ");
  uint8_t start_msg_letter[] = "AAAA"; //The max number of messages in buffer is 4 because why not?
  uint8_t index_start_msg = 0;
  // I first find the number of msg and the position of the first letter of them
  // I save them on start_msg_letter[]
  for (int i = 0; i < 150; i++)
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
}
void send_master(uint8_t msg) // I just have to send the flash info for getting the ack
{
  uint8_t data[20];
  if (msg == ACK)
  {
    // DPRINTLN("I send ack to master");
    for (int i = 0; i < sizeof(sys.ack_msg); i++)
      data[i] = sys.ack_msg[i];
  }
  uint8_t batery_level = batLevel();
  if (batery_level >= 100)
    batery_level == 99;
  data[8] = (batery_level / 10) + '0';
  data[9] = (batery_level % 10) + '0';
  // manager.sendtoWait(data, 15, SERVER_ADDRESS);
  uint8_t counter = 0;
  do
  {
    driver.send((const uint8_t *)data, sizeof(data));
    driver.waitPacketSent();
    delay(10);
    counter++;
  } while (counter < 2);
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
  intRtc = true;
}
void buttonInt()
{
  DPRINTLN("BUTTON PRESSED");
  state_machine = MODE_FIRST_SYN;
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
void ledBlink(uint8_t pin, long milli)
{
  digitalWrite(pin, 1);
  delay(milli);
  digitalWrite(pin, 0);
}
bool from_my_network(uint8_t *master_uuid, uint8_t *received_msg, uint8_t &index_msg)
{
  //I just find the uuid in the msg, if it is I return true:
  //00XXA1XX I want to check the A1 elements
  for (uint8_t element = 0; element < 10; element++)
    if (received_msg[element] == 88)
      if (received_msg[element + 1] == 88)
      {
        index_msg = element + 2; //I modified the index to point to the start of the msg
        if (received_msg[index_msg] == master_uuid[0] && received_msg[++index_msg] == master_uuid[1])
        {
          index_msg++;
          return true;
        }
        else
          return false;
        break; //If I find something and do not check I break
      }
  return false;
}