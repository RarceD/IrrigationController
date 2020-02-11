#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SparkFun_RV1805.h>
#include <SPIFlash.h>

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1
#define TIME_RESPONSE_NODE 1000

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

#define FLASH_SYS_DIR 0x040400

typedef struct
{
  uint8_t id;                 // This is the unique ID, there are 250 units so we can fix this number for identify the net
  uint8_t assigned_output[4]; // There are 4 output valves
} sysVar;

typedef enum
{
  MANVAL_MSG = 'M',
  TIME_MSG = 'T',
  ASSIGNED_MSG = 'A'
} msg_receive;

typedef enum
{
  ACK,
  FAULT,
  SENSORS
} msg_send;

// RF driver stuff:
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);
// RTC stuff:
RV1805 rtc;
SPIFlash flash(CS_M);

// Structs:
sysVar sys;

// Dont put this on the stack: VARIAS PUTAS HORAS POR NO HACER CASO A ESTO
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
uint8_t data_size, buf[RH_RF95_MAX_MESSAGE_LEN];
bool valve_flag, time_flag, assigned_flag, rf_flag;
unsigned long millix;
void setup()
{
  Serial.begin(115200);
  delay(250);
  pinMode(CS_RF, OUTPUT);
  digitalWrite(CS_RF, HIGH);
  pinMode(RF_RST, OUTPUT);
  digitalWrite(RF_RST, HIGH);
  pinMode(SIM_PWR, OUTPUT);
  pinMode(SIM_AWK, OUTPUT);
  pinMode(LED_SETUP, OUTPUT);
  digitalWrite(SIM_PWR, LOW);
  digitalWrite(SIM_AWK, HIGH);
  while (!Serial)
    ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  else
    Serial.println("Inicializado correctamente");
  driver.setTxPower(20, false);
  SWire.begin();
  rtc.begin();
  delay(1);
  rtc.updateTime();
  // Inicialice the variables just for first run
  Serial.print(rtc.stringDate());
  Serial.print(" ");
  Serial.println(rtc.stringTime());
  flash.powerUp();
  flash.begin();

  sys.id = 11;
  sys.assigned_output[0] = 1;
  flash.eraseSector(FLASH_SYS_DIR);
  flash.writeAnything(FLASH_SYS_DIR, sys);
  flash.readAnything(FLASH_SYS_DIR, sys); //read system variables
  //Una vez leido de la memoria solo me queda leerlo y comprobar que se ha guardado:
  delay(500);
  print_flash();
}
void loop()
{
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
  }
}
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
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
  default:
    Serial.println("NO TIENE QUE SALIR");
  }
  //send_master(ACK);
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
          Serial.println("I CLOSE");
        else
          Serial.println("I OPEN");
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
      Serial.print(out[msg_index]);
      Serial.print(" ");
    }
    if (is_for_me)
    {
      Serial.println("Es para mi, hago la asignación");
      sys.assigned_output[0] = out[0];
      sys.assigned_output[1] = out[1];
      sys.assigned_output[2] = out[2];
      sys.assigned_output[3] = out[3];
      flash.eraseSector(FLASH_SYS_DIR);
      flash.writeAnything(FLASH_SYS_DIR, sys);
    }
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
  Serial.println(" ");
}

