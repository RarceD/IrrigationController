
#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_RV1805.h>
#include <PinChangeInterrupt.h>

#define INT_RF 2
#define INT_RTC 14
#define CS_M 22
#define CS_RF 23
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))
#define DPRINT(...) Serial.print(__VA_ARGS__)
// Singleton instance of the radio driver
RH_RF95 rf95(CS_RF, INT_RF);
RV1805 rtc; // All the classes inicializated
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
char data[200];
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);
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
  rtc.setAlarmMode(6);
  rtc.setAlarm(28, 0, 0, 0, 0);
  for (int i = 0; i < 100; i++)
    data[i] = 'z';
}
volatile bool intRTC;
bool awake_mode;
void loop()
{
  if (intRTC)
  {
    intRTC = false;
    awake_mode = true;
    // DPRINTLN(millis());
    uint8_t data[150];
    uint8_t preamble[] = "00XXA1##TIME:H:XX/M:XX/S:XX##MANVAL#012#01:03##ASIGNED#021#045:099:004:035##STOP#ALL________";
    rtc.updateTime();
    Serial.println(rtc.stringTime());
#define offset 8
    uint8_t h = rtc.getHours();
    uint8_t m = rtc.getMinutes();
    uint8_t s = rtc.getSeconds();
    preamble[offset + 7] = (h / 10) + '0';
    preamble[offset + 8] = (h % 10) + '0';
    preamble[offset + 12] = (m / 10) + '0';
    preamble[offset + 13] = (m % 10) + '0';
    preamble[offset + 17] = (s / 10) + '0';
    preamble[offset + 18] = (s % 10) + '0';
    //TODO: Integrate the UUID in the msg
    memcpy(data, preamble, sizeof(preamble));
    uint8_t times = 6; //6 times better
    while (times-- > 0)
    {
      rf95.send((const uint8_t *)data, sizeof(data));
      rf95.waitPacketSent();
    }
    //for (int i = 0; i < 100; i++)
    //  Serial.write(data[i]);
    // Serial.println(" ");
    // DPRINTLN(millis());
    awake_mode = false;
  }

  if (Serial.available())
  {
    int a = Serial.read();
    if (a == 97)
    {
      DPRINTLN(millis());
      uint8_t data[150];
      uint8_t preamble[] = "00XXA1##TIME:H:XX/M:XX/S:XX##MANVAL#012#01:03##ASIGNED#021#045:099:004:035##STOP#ALL_________";
      // uint8_t offset = 8;
      rtc.updateTime();
      preamble[offset + 7] = (rtc.getHours() / 10) + '0';
      preamble[offset + 8] = (rtc.getHours() % 10) + '0';
      preamble[offset + 12] = (rtc.getMinutes() / 10) + '0';
      preamble[offset + 13] = (rtc.getMinutes() % 10) + '0';
      preamble[offset + 17] = (rtc.getSeconds() / 10) + '0';
      preamble[offset + 18] = (rtc.getSeconds() % 10) + '0';
      //TODO: Integrate the UUID in the msg
      memcpy(data, preamble, sizeof(preamble));
      uint8_t times = 2; //6 times better
      while (times-- > 0)
      {
        rf95.send((const uint8_t *)data, sizeof(data));
        rf95.waitPacketSent();
      }
      for (int i = 0; i < 100; i++)
        Serial.write(data[i]);
      Serial.println(" ");
      DPRINTLN(millis());
      awake_mode = false;
    }
    if (a == 98)
    {
      DPRINTLN("Send different: ");
      uint8_t data[150];
      memset(data, '0', sizeof(data));
      uint8_t preamble[] = "KILLME_PLEASE_";
      memcpy(data, preamble, sizeof(preamble));
      uint8_t times = 2; //6 times better
      while (times-- > 0)
      {
        rf95.send((const uint8_t *)data, sizeof(data));
        rf95.waitPacketSent();
      }
    }
  }
}

void rtcInt()
{
  intRTC = true;
}

void rtc_node(int hour, int minute, int second, int day, int month)
{
  uint8_t send_time[] = "##TIME:H:XX/M:XX/S:XX/D:XX/M:XX/";
  send_time[7 + 2] = (hour / 10) + '0';
  send_time[8 + 2] = (hour % 10) + '0';
  send_time[12 + 2] = (minute / 10) + '0';
  send_time[13 + 2] = (minute % 10) + '0';
  send_time[17 + 2] = (second / 10) + '0';
  send_time[18 + 2] = (second % 10) + '0';
  send_time[22 + 2] = (day / 10) + '0';
  send_time[23 + 2] = (day % 10) + '0';
  send_time[27 + 2] = (month / 10) + '0';
  send_time[28 + 2] = (month % 10) + '0';
  for (int i = 0; i < sizeof(send_time); i++)
    data[i] = send_time[i];
  //for (int i = 0; i < sizeof(send); i++)
  //  Serial.write(send[i]);
  //manager.sendtoWait(data, sizeof(send), CLIENT_ADDRESS);
}
/*
void send_nodo(uint16_t &order, uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
{
  //First write the destination of the message:
  bool f_man_valve = false, f_time = false, f_asigned = false, f_stop = false, f_manual = false, f_full = false;
  switch (msg)
  {
  case REQUEST_MANUAL:
  {
    f_manual = true;
    // DPRINTLN("MANUAL OPEN");
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
    // DPRINTLN("REQUEST MANVALVE");
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
    // DPRINTLN("REQUEST TIME");
    rtc.updateTime();
    rtc_node((int)rtc.getHours(), (int)rtc.getMinutes(), (int)rtc.getSeconds(), (int)rtc.getDate(), (int)rtc.getMonth(), order);
    f_time = false;
    break;
  }
  case REQUEST_ASSIGNED_VALVES:
  {
    // DPRINTLN("CHANGE ASIGNATION VALVE");
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
      Serial.write(data[order]);
      order++;
    }
    f_asigned = false;
    break;
  }
  case REQUEST_STOP_ALL:
  {
    // DPRINTLN("STOP ALL");
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
*/
