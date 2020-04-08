#include <JamAtm-Vyrsa.h>
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
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

#define TX_PWR 20
#define CLIENT_ADDRESS 4
#define SERVER_ADDRESS 3

#define FLASH_SYS_DIR 0x040400

typedef struct
{
  uint8_t id;                 // This is the unique ID, there are 250 units so we can fix this number for identify the net
  uint8_t assigned_output[4]; // There are 4 output valves
  uint8_t master_id[2];
  uint8_t ack_msg[8];
} sysVar;

sysVar sys;
RV1805 rtc;

SPIFlash flash(CS_M);

void setup()
{
  Serial.begin(115200);
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
  
  sys.id = 6;
  sys.master_id[0] = 'A';
  sys.master_id[1] = '3';

  sys.assigned_output[0] = 1;
  sys.assigned_output[1] = 2;
  sys.assigned_output[2] = 3;
  sys.assigned_output[3] = 4;
  char ack[] = "##OK00##";
  ack[5] = sys.id + '0';
  for (int i = 0; i < sizeof(ack); i++)
    sys.ack_msg[i] = ack[i]; 
  flash.eraseSector(FLASH_SYS_DIR);
  flash.writeAnything(FLASH_SYS_DIR, sys);
  
  flash.readAnything(FLASH_SYS_DIR, sys);

  SWire.begin();
  rtc.begin();
  rtc.set24Hour();
  rtc.enableInterrupt(INTERRUPT_AIE);
  rtc.enableTrickleCharge(DIODE_0_3V, ROUT_3K);
  rtc.setAlarmMode(6);
  rtc.setAlarm(0, 0, 0, 0, 0);
  // rtc.setToCompilerTime();
  // For disable the interrupt : //rtc.setAlarmMode(0);
  rtc.updateTime();
  DPRINT(rtc.stringDate());
  DPRINT(F(" "));
  DPRINTLN(rtc.stringTime());
  print_flash();
}
/******************************************************************* main program  ************************************************************************************/

void loop()
{
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
