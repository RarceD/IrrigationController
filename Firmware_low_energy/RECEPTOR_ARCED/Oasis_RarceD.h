
#include <PinChangeInterrupt.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SparkFun_RV1805.h>
#include <JamSleep.h>

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


#define PRG_INDEX 11
#define ADDR_INDEX 12
#define WR_ADDR_INDEX 13
#define WR_DATA_INDEX 17
#define NEW_CHILD 0xfe
#define TIME_NDEF 87000
#define TOTAL_VALVE 128
#define TOTAL_PROG 6
#define TOTAL_START 6
#define TOTAL_OUTPUT 4
#define MAX_JSON_SIZE 512
#define SYNC_TIME 5
#define UUID_LEN 37
#define APORTE_AGUA_LEN 47
#define INTERV_INIT_LEN 47
#define WATERING_LEN 17
#define ARRANQUE_LEN 35
#define IRRIG_TIME_LEN 192
#define BOOK_LEN 192
#define RESP_TIMEOUT 5000
#define MAX_JSON_SIZE 512
/*****************************/

/********* MEMORY MAP ***********/
#define SYS_VAR_ADDR 0x040000
#define PROG_VAR_ADDR 0x041000
#define IRRIG_ADDR 0x100000
/*****************************/


void softReset();
void ledBlink(uint8_t pin, long milli);