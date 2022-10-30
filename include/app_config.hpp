#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "Arduino.h"


//// Gap service ////
/**********************************************************************************************/
#define GENERIC_ACCESS_SERVICE_UUID             "1800"
#define GENERIC_ACCESS_DEVICE_NAME              "2A00"      // VALUE: device name String, Prop: Read | Notify
#define GENERIC_ACCESS_CONN_PARAM               "2A04"      //VALUE: connParameter HEX, Prop: Read

//// Gatt ////
/**********************************************************************************************/
#define GENERIC_ATTR_SERVICE_UUID               "1801"
#define GENERIC_ACCESS_APPERANCE                "2A01"      // VALUE: appearance HEX, Prop: Read

//// Device information service ////
/**********************************************************************************************/
#define DEVICE_INFO_SERVICE_UUID                "180A"
#define DEVICE_INFO_MOD_NAME                    "2A24"      // VALUE: Model Number String, Prop: Read
#define DEVICE_INFO_SERIAL_N                    "2A25"      // VALUE: Serial Number String, Prop: Read
#define DEVICE_INFO_FIRM_REV                    "2A26"      // VALUE: Firmware Revision String, Prop: Read
#define DEVICE_INFO_HARD_REV                    "2A27"      // VALUE: Hardware Revision, String, Prop: Read
#define DEVICE_INFO_SOFT_REV                    "2A28"      // VALUE: Software Revision, String, Prop: Read
#define DEVICE_INFO_MAN_NAME                    "2A29"      // VALUE: Manufacturer Number String, Prop: Read

//// Led service ////
/**********************************************************************************************/
#define LED_STATE_SERVICE_UUID                  "1214"
#define LED_STATE_SWITH                         "1214"      // VALUE: Sate Led Bool, Prop: Read | Write

//// Temp information service ////
/**********************************************************************************************/
#define TEMP_SERVICE_UUID                       "181A"
#define TEMP_LEVEL                              "2A1F"      // VALUE: last_temp Float, Prop: Read | Notify

//// Battery information service ////
/**********************************************************************************************/
#define BATTERY_SERVICE_UUID                    "180F"
#define BATT_LEVEL                              "2A19"      // VALUE: batVal Int, Prop: Read | Notify

//// Time information service ////
/**********************************************************************************************/
#define TIME_SERVICE_UUID                       "1805"
#define TIME_CURRENT                            "2A2B"      // VALUE: batVal Int, Prop: Read | Write
#define TIME_LOCAL                              "2A0F"      // VALUE: batVal Int, Prop: Read | Write

 /*
        UUID_DATE_TIME_CHAR                     = 0x2A08,
        UUID_CURRENT_TIME_CHAR                  = 0x2A2B,
*/

#define BLE_ADV_DIRECTED_ENABLED                true
#define ADVERTISING_INTERVAL                    5000                                /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_TIMEOUT_IN_SECONDS              180                                 /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL                       MSEC_TO_UNITS(20, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL                       MSEC_TO_UNITS(75, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define TX_POWER                                0

#define DEVICE_NAME                             "SKID"                              /**< Name of device. Will be included in the advertising data. */
#define LOCALNAME                               DEVICE_NAME

#define FIRMWARE                                "1.0.0"
#define REVISION                                "1.0"

#define UART_TX_BUF_SIZE                        256                                 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                        256                                 /**< UART RX buffer size. */

#define GI_BAT_PIN                              A4                                  // Board pin p0.30 Batt analog
#define GI_TMP_PIN                              5                                   // Board pin p0.30 Temp analog

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS           3300
#define ADC_PRE_SCALING_COMPENSATION            3 

#define DEBUG


#ifdef DEBUG
#define DEBUG_PRINT 1
#else
#define DEBUG_PRINT 0
#endif
#define DEBUG_PRINTF(...) do{ if (DEBUG_PRINT) { Serial.print(__VA_ARGS__); } } while(0)
#define DEBUG_PRINTLN(...) do{ if (DEBUG_PRINT) { Serial.print(__VA_ARGS__);  Serial.print("\r\n"); } } while(0)


#endif