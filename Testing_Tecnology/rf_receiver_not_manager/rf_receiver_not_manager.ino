
#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_RV1805.h>
#include <PinChangeInterrupt.h>

#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))
#define DPRINT(...) Serial.print(__VA_ARGS__)

#define INT_RF 2
#define INT_RTC 14
#define CS_M 22
#define CS_RF 23
// Singleton instance of the radio driver
RH_RF95 rf95(CS_RF, INT_RF);
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
RV1805 rtc; // All the classes inicializated
uint8_t state_machine;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  // rtc.setToCompilerTime();

  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  rtc.updateTime();
  DPRINT(rtc.stringTime());
  DPRINT(" ");
  DPRINTLN(rtc.stringDate());
  // rtc.setAlarmMode(0);
  // rtc.setAlarm(0, 0, 0, 0, 0);
  state_machine = MODE_FIRST_SYN;
}
volatile bool intRTC;
bool awake_mode;
uint64_t millix;
bool first_syn = true;
bool debug_sleep = false;
void loop()
{
  /*
  if (intRTC)
  {
    intRTC = false;
    rtc.updateTime();
    DPRINTLN(rtc.stringTime());
    if (state_machine != MODE_FIRST_SYN)
      state_machine = MODE_AWAKE;
    millix = millis();
    debug_sleep = true;
  }
  if (state_machine == MODE_FIRST_SYN)
  {
    //When I receive a msg I set the time and start a timer for waking up at :00
    if (rf95.available())
    {
      DPRINT("fIRST SYNC");
      uint8_t buf[200];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len))
      {
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
                  rtc.updateTime();
                  // int my_time = rtc.getSeconds();
                  // if (my_time != seconds)
                  // {
                  if (seconds < 60)
                    seconds++;
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
                  rtc.setAlarmMode(6);
                  rtc.setAlarm(30, 0, 0, 0, 0);
                  state_machine = MODE_SLEEP;
                  // }
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
    }
  }
  if (state_machine == MODE_AWAKE)
  {
    if (rf95.available())
    {
      uint8_t buf[200];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len))
      {
        uint8_t uuid_master[] = "A1";
        uint8_t index_msg = 0;
        //First I check if this msg is in my net and then if is valid I process and then stop to sleep
        if (from_my_network(uuid_master, buf, index_msg))
        {

          for (; index_msg < 150; index_msg++)
          {
            if (buf[index_msg] == '#')
            {
              if (buf[index_msg + 1] == '#') //I have found a msg:
              {
                uint8_t buffer_index = 0; // This variable is for offset what I am doing
                if (buf[index_msg + 2] == TIME_MSG)
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
                  int my_time = rtc.getSeconds();
                  if (my_time != seconds)
                  {
                    if (seconds < 60)
                      seconds++;
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
                    debug_sleep = true;
                  }
                }
              }
            }
            if (buf[index_msg] == '_')
              break;
          }
          state_machine = MODE_SLEEP;
        }
        //for (int i = 0; i < 180; i++)
        //  Serial.write(buf[i]);
      }
    }
    if (millis() - millix >= 4000)
    {
      DPRINTLN("LOST SYNC");
      state_machine = MODE_SLEEP;
    }
  }
  if (state_machine == MODE_SLEEP)
  {
    if (debug_sleep)
    {
      debug_sleep = false;
      DPRINTLN("SLEEP");
    }
  }
*/
  if (rf95.available())
  {
    uint8_t buf[150];
    memset(buf, '0', sizeof(buf));
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      for (uint8_t t = 0; t < 150; t++)
      {
        Serial.write(buf[t]);
      }
      uint8_t uuid_master[] = "A1";
      uint8_t index_msg = 0;
      //First I check if this msg is in my net
      if (!from_my_network(uuid_master, buf, index_msg))
        for (; index_msg < 150; index_msg++)
        {
          if (buf[index_msg] == '#')
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
                int my_time = rtc.getSeconds();
                if (my_time != seconds)
                {
                  if (seconds < 60)
                    seconds++;
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
                }
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
              default:
                DPRINTLN("NO TIENE QUE SALIR");
                if (buf[index_msg] == '_')
                  break;
              }
              for (int i = 0; i < 180; i++)
                Serial.write(buf[i]);
            }
        }
    }
        Serial.println(" ");
 
  }
}

void rtcInt()
{
  intRTC = true;
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