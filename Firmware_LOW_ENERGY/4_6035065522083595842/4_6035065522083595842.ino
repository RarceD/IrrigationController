#include <SoftwareSerial.h>
#define SSerialRX 16
#define SSerialTX 17
SoftwareSerial softSerial(SSerialRX, SSerialTX);

//uint8_t buf[] = {0x02,0xfe,'I','N','I','T',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','D','E','V','I','C','E',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','D','A','T','E',0x23,'1','A','0',0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','L','I','N','E',0x23,'1','A','0',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','B','O','O','K',0x23,'1','B','8',0x23,0x03,0,0};
uint8_t buf[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'D', 'A', 'T', 'A', 0x23, '4', '0', '0', 0x23, '0', 'A', 0x23, 0x03, 0, 0};
//uint8_t buf[] = {0x02,0xfe,'W','R','I','T','E',' ','B','O','O','K',0x23,'1','A','0',0x23,'1','1','A','A','A','A','A','A','A','A','F',' ','0','0',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                  'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                  'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','S','T','A','T','U','S',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','P','R','G',0x23,'A',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D', ' ','P','A','G','E',0x23,'0','A','0',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','S','E','T',' ', 'U','N','I','T',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','T','I','M','E',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'W','R','I','T','E',' ','F','L','A','G','S',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','F','L','A','G','S',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'S','E','T',' ','O','A','S','I','S',0x23,'0','0',0x23,'O','K',0x23,0,0};
//uint8_t buf[] = {0x02,0xfe,'S','E','T',' ','O','A','S','I','S',0x23,' ',' ',0x23,'N','O','K',0x23,0,0};
uint16_t i, j;
int calcrc(char ptr[], int length);
/******************************************************************* setup section ************************************************************************************/
void setup()
{
  uint32_t millix;
  bool intPg = false;
  uint8_t i = 0; //initialize index
  char pgData[100];
  Serial.begin(115200);
  softSerial.begin(9600);
  calcrc((char *)buf, sizeof(buf) - 2);
  softSerial.write(buf, sizeof(buf));
  if (softSerial.available())
  {
    char a = softSerial.read();
    Serial.write(a);
  }
  /*
  millix = millis(); //start timer
  while ((unsigned long)(millis() - millix) < 100)
  { //while timer doesn't reach 100 ms
    while (softSerial.available())
    {                                  //while there is data to read
      pgData[i++] = softSerial.read(); //read byte and save it
      millix = millis();
    }
  }
  pgData[i] = '\0';
  String pg = String(pgData); //convert message received into string
 Serial.print("PG: ");
  Serial.println(pg);
  Serial.print("Command send: ");
  for (i = 0; i < sizeof(buf); i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Read response: ");
  if (softSerial.available())
  {
    Serial.println(softSerial.read());
  }
  //uint8_t buff[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'D', 'A', 'T', 'A', 0x23, '4', '0', '1', 0x23, '0', 'A', 0x23, 0x03, 0, 0};
  //calcrc((char*)buff, sizeof(buff) - 2);
  //softSerial.write(buf, sizeof(buff));
  //Serial.print("Command send: ");
  //for (i = 0; i < sizeof(buff); i++) {
  //  Serial.print(buff[i], HEX);
  //  Serial.print(" ");
  //}
  //buff[15]='8';
  //delay(1500);
  //calcrc((char*)buff, sizeof(buff) - 2);
  //softSerial.write(buf, sizeof(buff));
  //Serial.print("Command send: ");
  //for (i = 0; i < sizeof(buff); i++) {
  //  Serial.print(buff[i], HEX);
  //  Serial.print(" ");
  //}*/
}
void loop()
{
}

int calcrc(char ptr[], int length)
{

  char i;
  int crc = 0;

  while (--length >= 0)
  {
    crc = crc ^ (int)*ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    } while (--i);
  }
  *ptr = crc & 0xff;
  *(++ptr) = crc >> 8;
  return (crc);
}
