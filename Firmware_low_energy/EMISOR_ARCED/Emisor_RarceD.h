
#include <stdint.h>
#include <JamSleep.h>
#include <PinChangeInterrupt.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SparkFun_RV1805.h>
// #include <JamAtm-Vyrsa.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

#define TX_PWR 20
#define RF_TIMEOUT 500
#define MAX_TEMP 15000
#define FACTORY_TIMEOUT 3000
#define LED_E_PIN 19
#define MAX_CHILD 16
#define SECTOR_SIZE 4096
#define CMD_INDEX 0
#define PAYLOAD_INDEX 6
#define PG_MAX_LEN 256
#define PG_TIMEOUT 2000
#define ACK_SIZE 6
#define ETX 0x03
#define BOOK_LEN 192
#define ADDR_INDEX 12
#define TOTAL_VALVE 100
#define TOTAL_START 6
#define TOTAL_PROG 6
#define MEMMORY 3
#define UUID_LEN 37


#define APORTE_AGUA_LEN 47
#define INTERV_INIT_LEN 47
#define WATERING_LEN 17
#define ARRANQUE_LEN 35
#define IRRIG_TIME_LEN 192
#define BOOK_LEN 192
#define RESP_TIMEOUT 5000
#define MAX_JSON_SIZE 512

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

#define TX_PWR 20
#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

#define FLASH_SYS_DIR 0x044000

#define MAX_NODE_NUMBER 4
#define MAX_MANUAL_TIMERS 20
#define MAX_NUM_MESSAGES 15
#define UUID_LENGTH 16
#define TIME_RESPOSE 50000

void ledBlinkk(uint8_t pin, uint64_t milli);
int freeRam();
uint8_t hex2int(char ch); // For converting the manual valve action
String getValue(String data, char separator, int index);
int calcrc(char ptr[], int length);



/********* MEMORY MAP ***********/
#define SYS_VAR_ADDR 0x040000
#define PROG_VAR_ADDR 0x041000
#define IRRIG_ADDR 0x100000
/*****************************/


static uint8_t cmd_read_date[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'D', 'A', 'T', 'E', 0x23, 0x03, 0, 0};
static uint8_t cmd_read_time[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'T', 'I', 'M', 'E', 0x23, 0x03, 0, 0};
static uint8_t cmd_read_flag[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'F', 'L', 'A', 'G', 'S', 0x23, 0x03, 0, 0};
static uint8_t cmd_write_flag[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'F', 'L', 'A', 'G', 'S', 0x23, 0x03, 0, 0};
static uint8_t cmd_ok[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'O', 'K', 0x23, 0, 0};
static uint8_t cmd_read_line[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'L', 'I', 'N', 'E', 0x23, ' ', ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_read_page[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'P', 'A', 'G', 'E', 0x23, ' ', ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_read_book[] = {0x02, 0xfe, 'R', 'E', 'A', 'D', ' ', 'B', 'O', 'O', 'K', 0x23, ' ', ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_nok[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'N', 'O', 'K', 0x23, 0, 0};
static uint8_t cmd_com_error[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'O', 'A', 'S', 'I', 'S', 0x23, ' ', ' ', 0x23, 'C', 'O', 'M', 0x23, 0, 0};
static uint8_t cmd_stop_manprg[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manprg[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'P', 'R', 'G', 0x23, ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_write_data[] = {0x02, 0xfe, 'W', 'R', 'I', 'T', 'E', ' ', 'D', 'A', 'T', 'A', 0x23, ' ', ' ', ' ', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_start_manvalv[] = {0x02, 0xfe, 'S', 'T', 'A', 'R', 'T', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, ' ', ' ', 0x23, ' ', ' ', ' ', ' ', 0x23, 0x03, 0, 0};
static uint8_t cmd_stop_manvalv[] = {0x02, 0xfe, 'S', 'T', 'O', 'P', ' ', 'M', 'A', 'N', 'V', 'A', 'L', 'V', 0x23, ' ', ' ', 0x23, 0x03, 0, 0};

static uint8_t cmd_set_time[] = {0x02, 0xfe, 'S', 'E', 'T', ' ', 'T', 'I', 'M', 'E', 0x23, '0','9','0','5','0','0', 0x23, '0', '3', 0x23,0x03, 0, 0};

