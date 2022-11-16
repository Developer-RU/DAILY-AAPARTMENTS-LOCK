#ifndef MAIN_H_
#define MAIN_H_


#include "app_config.hpp"
#include "Arduino.h"
#include "SPI.h"
#include "BLEPeripheral.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"

#include "ble_hci.h"
#include "nrf51_bitfields.h"

#include "BLEStringCharacteristic.hpp"

#include "stdlib.h"
#include "stdint.h"
#include "string.h"




#define uchar unsigned char
#define uint unsigned int


/**
 * @brief 
 * 
 */
//void SWI2_IRQHandler(void);

/**
 * @brief 
 * 
 */
//void SWI2_IRQWakeUp(void);


/**
 * @brief 
 * 
 */
//void TIMER1_IRQHandler(void);

/**
 * @brief 
 * 
 */
void start_timer_1(void);


/**
 * @brief 
 * 
 */
//void TIMER2_IRQHandler(void);

/**
 * @brief 
 * 
 */
void start_timer_2(void);


/**
 * @brief 
 * 
 */
//void ADC_IRQHandler(void);

/**
 * @brief 
 * 
 */
void adc_init(void);


/**
 * @brief 
 * 
 * @param error_code 
 * @param line_num 
 * @param p_file_name 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);

/**
 * @brief 
 * 
 */
static void gap_params_init(void);


/**
 * @brief 
 * 
 */
void gpio_output_init();

/**
 * @brief Get the Voltage object
 * 
 * @return String 
 */
String getVoltage();

/**
 * @brief Get the cpiID object
 * 
 */
void get_cpuID();

/**
 * @brief 
 * 
 */
void services_init();

/**
 * @brief 
 * 
 */
void advertising_init(void);

/**
 * @brief 
 * 
 * @param central 
 * @param characteristic 
 */
void characteristicWrittenCallback(BLECentral& central, BLECharacteristic& characteristic);

/**
 * @brief 
 * 
 * @param central 
 */
void blePeripheralConnectHandler(BLECentral& central);

/**
 * @brief 
 * 
 * @param central 
 */
void blePeripheralDisconnectHandler(BLECentral& central);

/**
 * @brief 
 * 
 * @param new_time 
 */
void updateAdvertisingScanData(unsigned long new_time);


/**
 * @brief 
 * 
 */
void setup();

/**
 * @brief 
 * 
 */
void loop();


#endif