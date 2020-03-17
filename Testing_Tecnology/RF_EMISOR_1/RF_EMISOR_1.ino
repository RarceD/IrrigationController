#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <PinChangeInterrupt.h>
#include <SPIFlash.h>
#include <SpiRam_Extended.h>
#include <SimpleTimer.h>
#include <SparkFun_RV1805.h>

#define char uint8_t

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1
#define MAX_NODE_NUMBER 7
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000

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

#define SYS_VAR_ADDR 0x040000
#define SECTOR_SIZE 4096
#define PAYLOAD_INDEX 3

#define FLASH_SYS_DIR 0x020000
#define FLASH_SENSOR_DIR 0x025000

#define SRAM_ESP_ADDR 0x000100
#define SRAM_ATM_ADDR 0x000500
#define SRAM_BUFFER_SIZE 1024

typedef struct
{
  uint8_t master_id;
  uint8_t UUID[UUID_LENGTH];
  uint8_t nodes_uuid[UUID_LENGTH][MAX_NODE_NUMBER];
  uint8_t currentFwVer[10];
} sysVar;

uint8_t UUID_1[] = {29, 126, 254, 123, 181, 94, 75, 217, 175, 233, 194, 218, 54, 62, 115, 110};
//uint8_t UUID_2[] = {215, 183, 122, 1, 255, 134, 73, 95, 188, 173, 6, 124, 72, 106, 181, 240};
//uint8_t UUID_3[] = {40, 57, 116, 35, 217, 163, 77, 162, 191, 87, 99, 102, 244, 64, 117, 243};
//uint8_t UUID_4[] = {3, 131, 131, 96, 26, 90, 68, 199, 130, 11, 252, 253, 195, 50, 118, 112};
//uint8_t UUID_5[] = {68, 114, 1, 67, 227, 4, 64, 217, 180, 152, 221, 28, 123, 67, 224, 191};
//uint8_t UUID_6[] = {243, 230, 25, 47, 203, 141, 77, 173, 170, 219, 35, 112, 89, 235, 42, 59};
//uint8_t UUID_7[] = {5, 109, 127, 225, 253, 170, 75, 16, 174, 89, 169, 201, 145, 168, 51, 123};
typedef enum
{
  REQUEST_MANVAL,
  REQUEST_TIME,
  REQUEST_ASSIGNED_VALVES
} messages_radio;

// Radio Driver
RH_RF95 driver(CS_RF, INT_RF);
RHReliableDatagram manager(driver, SERVER_ADDRESS);
RV1805 rtc;
SPIFlash flash(CS_M);

// Dont put this on the stack:
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
// THE HUGE BUFFER for SRAM:
uint8_t buf[70];
bool rf_flag = false;

sysVar sys;
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
  //sram_index = 0;
  SWire.begin();
  rtc.begin();
  delay(1);
  rtc.updateTime();
  Serial.print(rtc.stringDate());
  Serial.print(" ");
  Serial.println(rtc.stringTime());
  while (!Serial)
    ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  else
    Serial.println("Inicializado correctamente");
  driver.setTxPower(20, false);
  flash.powerUp();
  flash.begin();
  //uint8_t original_UUID[] = {29, 126, 254, 123, 181, 94, 75, 217, 175, 233, 194, 218, 54, 62, 115, 110};
  //for (int i = 0; i<sizeof(original_UUID);i++)
  //sys.UUID[i] = original_UUID[i];
  //sys.id = 1;
  //flash.eraseSector(FLASH_SYS_DIR);
  //flash.writeAnything(FLASH_SYS_DIR,sys);
  flash.readAnything(FLASH_SYS_DIR, sys); //read system variables
  //Una vez leido de la memoria solo me queda leerlo y comprobar que se ha guardado:

  //print_flash();
  millix = millis();
  //write_flash();
}
#define char uint8_t
char asignacion[] = {random(1, 128), random(1, 128), random(1, 128), random(1, 128)};
void loop()
{
  /* 
      I use the serial port to send commands just for debugging
  */
  if (Serial.available())
  {
    uint8_t a = Serial.read();
    uint8_t nodo_envio[16];
    if (a == 97) // if I send letter a I request nodos
      send_nodo(UUID_1, REQUEST_MANVAL, 1, random(0, 12), random(1, 59), asignacion);
    if (a == 98) // if I send letter b I request just 1 NODES !
      send_nodo(UUID_1, REQUEST_TIME, 0, 0, 0, asignacion);
    if (a == 99)
      send_nodo(UUID_1, REQUEST_ASSIGNED_VALVES, 11, 0, 0, asignacion);
    if (a == 100) // CLOSE VALVE if d
      send_nodo(UUID_1, REQUEST_MANVAL, 1, 0, 0, asignacion);
  }
  if (millis() - millix > 4000)
  {
    millix = millis();
    send_nodo(UUID_1, REQUEST_MANVAL, random(1, 127), random(0, 12), random(1, 59), asignacion);
  }
  /* 
    If I detect radio activity I just listen the remote nodes
  */
  if (manager.available())
  {
    uint8_t len = sizeof(buf);
    manager.recvfromAck(buf, &len);
    rf_flag = true;
  }
  if (rf_flag)
  {
    listen_nodo();
    rf_flag = false;
  }
}
void send_nodo(uint8_t uuid[], uint8_t msg, char valve, char hour, char minutes, char assigned[])
{
  //First write the destination of the message:
  bool f_man_valve = false, f_time = false, f_asigned = false;
  switch (msg)
  {
  case REQUEST_MANVAL:
    f_man_valve = true;
    Serial.println("REQUEST MANVALVE");
    break;
  case REQUEST_TIME:
    f_time = true;
    Serial.println("REQUEST TIME");
    break;
  case REQUEST_ASSIGNED_VALVES:
    Serial.println("CHANGE ASIGNATION VALVE");
    f_asigned = true;
    break;
  default:
    Serial.println("JAMAS SALE");
  }
  if (f_man_valve)
  {
    uint8_t str_manval[] = "##MANVAL#000#00:00#";
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

    for (int i = 0; i < sizeof(str_manval); i++)
      data[i] = str_manval[i];
    f_man_valve = false;
  }
  else if (f_time)
  {
    rtc.updateTime();
    rtc_node((int)rtc.getHours(), (int)rtc.getMinutes(), (int)rtc.getSeconds(), (int)rtc.getDate(), (int)rtc.getMonth());
    f_time = false;
  }
  else if (f_asigned)
  {
    //VALVE es el ID que va del 1 al 250
    uint8_t str_assigned[] = "##ASIGNED#000#000:000:000:000#";
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

    for (int j = 0; j < sizeof(str_assigned); j++)
      data[j] = str_assigned[j];
    f_asigned = false;
  }
  manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
}
void rtc_node(int hour, int minute, int second, int day, int month)
{
  uint8_t send[] = "##TIME:H:XX/M:XX/S:XX/D:XX/M:XX/ ";
  send[7 + 2] = (hour / 10) + 0x30;
  send[8 + 2] = (hour % 10) + 0x30;
  send[12 + 2] = (minute / 10) + 0x30;
  send[13 + 2] = (minute % 10) + 0x30;
  send[17 + 2] = (second / 10) + 0x30;
  send[18 + 2] = (second % 10) + 0x30;
  send[22 + 2] = (day / 10) + 0x30;
  send[23 + 2] = (day % 10) + 0x30;
  send[27 + 2] = (month / 10) + 0x30;
  send[28 + 2] = (month % 10) + 0x30;
  for (int i = 0; i < sizeof(send); i++)
    data[i] = send[i];
  for (int i = 0; i < sizeof(send); i++)
    Serial.write(send[i]);
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
void listen_nodo()
{
  Serial.println("");
  Serial.println("He recibido de nodos:");
  //uint8_t print_len = (buf[0] == 'S') ? 8 : 3;
  //print_len = (buf[0] == 'R') ? 14 : 70;
  for (uint8_t i = 0; i < 70; i++) //loop from the buffer looking for the end of message
    Serial.write(buf[i]);
}