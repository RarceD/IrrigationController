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

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1
#define TIME_RESPONSE_NODE 1000

typedef enum
{
  MANVAL_MSG = 'M',
  TIME_MSG = 'T',
  ASSIGNED_MSG = 'A',
  STOP_MSG = 'S'
} msg_receive;
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
  uint8_t master_id;
} sysVar;

Jam jam;
sysVar sys;
RV1805 rtc;
Sleep lowPower;

SPIFlash flash(CS_M);
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

volatile bool intButton = false, intRtc = false, Global_Flag_int = false;
uint16_t Set_Vshot = 600;
uint8_t iOpen = 0, valveOpened[4];
uint8_t i, j, cmd, msgPending, minAlarm, hourAlarm, secAlarm, dataSize;
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
uint8_t data_size, buf[RH_RF95_MAX_MESSAGE_LEN];
bool valve_flag, time_flag, assigned_flag, stop_flag, rf_flag;

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
  //sys.id = 11;
  //sys.assigned_output[0] = 1;
  //flash.eraseSector(SYS_VAR_ADDR);
  //flash.writeAnything(SYS_VAR_ADDR, sys);
  flash.readAnything(SYS_VAR_ADDR, sys);
  manager.init();
  driver.setPreambleLength(8);
  driver.setTxPower(TX_PWR, false);
  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);

  //rtc.setAlarmMode(4);
  //  bool RV1805::setAlarm(uint8_t sec, uint8_t min, uint8_t hour, uint8_t date, uint8_t month)

  //rtc.setAlarm(10, 0, 0, 0, 0);

  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);
  attachPCINT(digitalPinToPCINT(SW_SETUP), buttonInt, FALLING);
  chargeCapacitor();
  jam.ledBlink(LED_SETUP, 1000);
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  print_flash();

  //sys.id = 11
}
volatile bool a;
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

  if (manager.available()) // Detect radio activity
  {
    uint8_t len = sizeof(buf);
    manager.recvfromAck(buf, &len);
    rf_flag = true;
  }
  if (rf_flag)
  {
    listen_master(); //When activity is detected then listen the master
    rf_flag = false;
    // driver.sleep();
    // lowPower.sleep_delay(5555);
  }
  if (a)
  {
    a = false;
    delay(5000);
    Serial.println("alarma dada");
    rtc.setAlarmMode(4); // one every day if 4 is selected, if 6 is selected then every day
    rtc.setAlarm(1, 1, 0, 0, 0);
  }
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
void listen_master() // Listen and actuate in consideration
{
  Serial.println("He recibido del master: ");
  uint8_t start_msg;
  bool is_for_me = false;

  for (int i = 0; i < sizeof(buf); i++)
  {
    Serial.write(buf[i]);
    if (buf[i] == '#')
      if (buf[i + 1] == '#')
        start_msg = i + 2;
  }
  Serial.println(" ");

  //I clasified the message received:
  switch (buf[start_msg])
  {
  case MANVAL_MSG:
    Serial.println("VALVE ACTION");
    valve_flag = true;
    break;
  case TIME_MSG:
    Serial.println("TIME MODE");
    send_master(ACK);
    time_flag = true;
    break;
  case ASSIGNED_MSG:
    Serial.println("ASSIGNED MODE");
    send_master(ACK);
    assigned_flag = true;
    break;
  case STOP_MSG:
    Serial.println("STOP MODE");
    stop_flag = true;
    break;
  default:
    Serial.println("NO TIENE QUE SALIR");
  }
  if (time_flag)
  {
    time_flag = false;
    //uint8_t send[] = "##TIME:H:XX  /M:XX/S: XX/D:XX /M:XX/ ";
    //uint8_t sead[] = "01234567890  12345678 9012345 67890";
    int hours, minutes, day, month, year;
    if (buf[9] == '0')
      hours = buf[10] - '0';
    else
      hours = (buf[9] - '0') * 10 + (buf[10] - '0');

    if (buf[14] == '0')
      minutes = buf[15] - '0';
    else
      minutes = (buf[14] - '0') * 10 + (buf[15] - '0');

    if (buf[24] == '0')
      day = buf[25] - '0';
    else
      day = (buf[24] - '0') * 10 + (buf[25] - '0');

    if (buf[29] == '0')
      month = buf[30] - '0';
    else
      month = (buf[29] - '0') * 10 + (buf[30] - '0');
    change_time(hours, minutes, day, month, 2019);
  }
  else if (valve_flag)
  {
    //##MANVAL#002#08:04#
    valve_flag = false;
    uint8_t valve_action = (buf[9] - '0') * 100 + (buf[10] - '0') * 10 + (buf[11] - '0');
    uint8_t valve_time_hours = (buf[13] - '0') * 10 + (buf[14] - '0');
    uint8_t valve_time_minutes = (buf[13 + 3] - '0') * 10 + (buf[14 + 3] - '0');
    Serial.print("Valve action: ");
    Serial.print(valve_action);
    Serial.print(" time: ");
    Serial.print(valve_time_hours);
    Serial.print(":");
    Serial.print(valve_time_minutes);
    for (int i = 0; i < 4; i++) // I test if the message is for me and I open, or close the valve.
    {
      if (sys.assigned_output[i] == valve_action)
      {
        Serial.println("Is for me: ");
        if (valve_time_hours == 0 && valve_time_minutes == 0)
          valveAction(i + 1, false);
        else
          valveAction(i + 1, true);
      }
    }
  }
  else if (assigned_flag)
  {
    //##ASIGNED#720#045:099:004:035#
    assigned_flag = false;
    Serial.println("");
    Serial.println("");
    uint8_t id_msg = (buf[10] - '0') * 100 + (buf[11] - '0') * 10 + (buf[12] - '0');
    uint8_t out[4];
    uint8_t offset_msg = 0;
    //  I test if the message is for me checking the unique ID
    // This only is used when the assignation message is sent
    if (sys.id == id_msg)
      is_for_me = true;
    else
      is_for_me = false;
    Serial.print("Assigned done in id: ");
    Serial.print(id_msg);
    Serial.print(" outputs: ");
    for (uint8_t msg_index = 0; msg_index < 4; msg_index++, offset_msg += 4)
    {
      out[msg_index] = (buf[start_msg + 8 + 4 + offset_msg] - '0') * 100 + (buf[start_msg + 9 + 4 + offset_msg] - '0') * 10 + (buf[start_msg + 10 + 4 + offset_msg] - '0');
      Serial.print(out[msg_index] + 1);
      Serial.print(" ");
    }
    if (is_for_me)
    {
      Serial.println("Es para mi, hago la asignación");
      sys.assigned_output[0] = out[0] + 1;
      sys.assigned_output[1] = out[1] + 1;
      sys.assigned_output[2] = out[2] + 1;
      sys.assigned_output[3] = out[3] + 1;

      digitalWrite(CS_RF, HIGH); //unselect rf
      flash.eraseSector(SYS_VAR_ADDR);
      flash.writeAnything(SYS_VAR_ADDR, sys);
      digitalWrite(CS_RF, LOW);
    }
  }
  else if (stop_flag)
  {
    stop_flag = false;
    //##STOP#ALL#
    valveAction(1, false);
    delay(1200);
    valveAction(2, false);
    delay(1200);
    valveAction(3, false);
    delay(1200);
    valveAction(4, false);
  }
  else
  {
    Serial.println("no tiene que salir");
  }
}
void send_master(uint8_t msg)
{
  if (msg == ACK)
  {
    Serial.println("##OK");
    char ack[] = "##OK";
    for (int i = 0; i < sizeof(ack); i++)
      data[i] = ack[i];
  }
  else if (msg == FAULT)
  {
    Serial.println("##FAULT");
    char fault[] = "##OK";
    for (int i = 0; i < sizeof(fault); i++)
      data[i] = fault[i];
  }
  manager.sendtoWait(data, 70, SERVER_ADDRESS);
}
void change_time(int hours, int minutes, int day, int month, int year)
{

  rtc.set24Hour();
  uint8_t currentTime[8];
  currentTime[0] = rtc.DECtoBCD(0);
  currentTime[1] = rtc.DECtoBCD(30);
  currentTime[2] = rtc.DECtoBCD(minutes);
  currentTime[3] = rtc.DECtoBCD(hours);
  currentTime[4] = rtc.DECtoBCD(day);
  currentTime[5] = rtc.DECtoBCD(month);
  currentTime[6] = rtc.DECtoBCD(year - 2000);
  currentTime[7] = rtc.DECtoBCD(0);
  rtc.setTime(currentTime, TIME_ARRAY_LENGTH);
  rtc.updateTime();
  Serial.print(rtc.stringDate());
  Serial.print(" ");
  Serial.println(rtc.stringTime());
  Serial.print("DayOfWeek: ");
  Serial.println(rtc.dayOfWeek());

  /***** timestamp ******/
  rtc.updateTime();
  Serial.print("timestamp: ");
  Serial.println(rtc.getTimestamp());
}
void rtcInt()
{
  Serial.println("RTC INTERRUPTION");
  // driver.sleep();
  // lowPower.sleep_delay(5555);
}
void buttonInt()
{
  Serial.println("BUTTON PRESSED");
  a = true;
}
void print_flash()
{
  Serial.println(" ");
  Serial.print("El valor del ID es: ");
  Serial.println(sys.id);
  Serial.print("El valor de la asignación es: ");
  Serial.print(sys.assigned_output[0]);
  Serial.print(", ");
  Serial.print(sys.assigned_output[1]);
  Serial.print(", ");
  Serial.print(sys.assigned_output[2]);
  Serial.print(", ");
  Serial.print(sys.assigned_output[3]);
  Serial.println(", ");
}
