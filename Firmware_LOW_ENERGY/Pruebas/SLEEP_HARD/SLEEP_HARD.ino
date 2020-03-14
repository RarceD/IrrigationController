#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SparkFun_RV1805.h>
#include <SPIFlash.h>
#include <JamSleep.h>
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#include <TrueRandom.h>

#include <SPIFlash.h>
#include <SPI.h>

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

Sleep lowPower;

// Structs:
sysVar sys;
bool a = false;

// Dont put this on the stack: VARIAS PUTAS HORAS POR NO HACER CASO A ESTO
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
uint8_t data_size, buf[RH_RF95_MAX_MESSAGE_LEN];
bool valve_flag, time_flag, assigned_flag, rf_flag, Global_Flag_int;
uint8_t dataSize, msgPending;

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
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  rtc.setAlarmMode(6);
  rtc.setAlarm(10, 0, 0, 0, 0);
  flash.powerUp();
  flash.begin();

  sys.id = 11;
  sys.assigned_output[0] = 1;
  //flash.eraseSector(FLASH_SYS_DIR);
  //flash.writeAnything(FLASH_SYS_DIR, sys);
  flash.readAnything(FLASH_SYS_DIR, sys); //read system variables
  //Una vez leido de la memoria solo me queda leerlo y comprobar que se ha guardado:
  //print_flash();
  attachPCINT(digitalPinToPCINT(INT_RTC), rtcInt, FALLING);

  delay(500);
  //change_time(16, 13, 6, 2, 20);
}
void rtcInt()
{
  Serial.println("INT");
  Global_Flag_int = true;
}
void loop()
{

  if (Serial.available())
  {
    int input = Serial.read();
    if (input == 97)
    {
      Serial.println(input);
      rtc.updateTime();
      Serial.println(rtc.stringTime());
      if (rtc.getSeconds() >= 30)
      {
        Serial.println("A dormir que son más de las 30");
        lowPower.sleep_delay(28000);
      }
      else
      {
        Serial.println("A dormir que son más de las 30");
      }
    }
  }
  if (Global_Flag_int)
  {
    Global_Flag_int = false;
    driver.sleep();
    lowPower.sleep_delay(30000);
  }
  else
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
  for (int i = 0; i < sizeof(buf); i++)
    Serial.write(buf[i]);
  Serial.println(" ");
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
