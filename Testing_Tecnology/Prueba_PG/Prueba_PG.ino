#include <SoftwareSerial.h>
#define SSerialRX 16
#define SSerialTX 17
SoftwareSerial softSerial(SSerialRX, SSerialTX);

//uint8_t buf[] = {0x02,0xfe,'I','N','I','T',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','D','E','V','I','C','E',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','D','A','T','E',0x23,'0','5','A',0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','L','I','N','E',0x23,'1','C','0',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','B','O','O','K',0x23,'2','0','0',0x23,0x03,0,0};
uint8_t buf[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, 'A', 0x23, 0x03, 0, 0};
//uint8_t buf[] = {0x02,0xfe,'S','T','A','R','T',' ','M','A','N','V','A','L','V',0x23,'1','4',0x23,'0','0','0','1',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'W','R','I','T','E',' ','D','A','T','A',0x23,'1','A','1',0x23,'1','E',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'W','R','I','T','E',' ','B','O','O','K',0x23,'1','A','0',0x23, 'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                        'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                      'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F', ' ',
//                                                                                    'F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',' ','F','F',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'S','E','T',' ','T','I','M','E',0x23,'0','8','5','0','0','0',0x23,'0','5',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','P','R','G',0x23,'F',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D', ' ','P','A','G','E',0x23,'0','A','0',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','S','E','T',' ', 'U','N','I','T',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','T','I','M','E',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'W','R','I','T','E',' ','F','L','A','G','S',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'R','E','A','D',' ','F','L','A','G','S',0x23,0x03,0,0};
//uint8_t buf[] = {0x02,0xfe,'S','E','T',' ','O','A','S','I','S',0x23,'0','0',0x23,'O','K',0x23,0,0};
//uint8_t buf[] = {0x02,0xfe,'S','E','T',' ','O','A','S','I','S',0x23,' ',' ',0x23,'N','O','K',0x23,0,0};
static uint8_t cmd_start_manvalv[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, ' ', ' ', 0x23, ' ', ' ', ' ', ' ', 0x23, 0x03, 0, 0};

uint16_t i, j;
int calcrc(char ptr[], int length);
/******************************************************************* setup section ************************************************************************************/
void setup()
{

  Serial.begin(115200);
  softSerial.begin(9600);
  calcrc((char *)buf, sizeof(buf) - 2);
  softSerial.write(buf, sizeof(buf));
  Serial.print("Command send: ");
  for (i = 0; i < sizeof(buf); i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Read response: ");
  delay(2000);
}

/******************************************************************* main program  ************************************************************************************/
void loop()
{

  while (softSerial.available())
  {
    Serial.print((char)softSerial.read());
  }
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
