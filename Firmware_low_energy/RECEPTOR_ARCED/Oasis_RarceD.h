
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


/*********** PIN OUT **********/
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
/********* MEMORY MAP ***********/
#define SYS_VAR_ADDR 0x040000
#define PROG_VAR_ADDR 0x041000
#define FLASH_SYS_DIR 0x040400

/************ RF info ***********/
#define TX_PWR 20
#define CLIENT_ADDRESS 4
#define SERVER_ADDRESS 3

#define DEAD_TIME_COUNTER 5  //if I lose 20 packets I am dead and I close all the valves I have
#define AWAKE_TIME_COUNTER 2 //if I do not receive 3 packets I awake 1 minute compleat just one time
#define AWAKE_TIME_PER_MIN 2000

void ledBlinkk(uint8_t pin, uint64_t milli);
int freeRam();
