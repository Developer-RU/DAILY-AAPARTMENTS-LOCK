#include "main.hpp"



#ifdef __cplusplus

extern "C" {
    #endif

        void SWI2_IRQWakeUp(void);
        void SWI2_IRQHandler(void);

        void TIMER1_IRQHandler(void);
        void TIMER2_IRQHandler(void);
        void ADC_IRQHandler(void);

    #ifdef __cplusplus
}

#endif

// BLEBoolCharacteristic
// BLECharCharacteristic
// BLEUnsignedCharCharacteristic
// BLEShortCharacteristic
// BLEUnsignedShortCharacteristic
// BLEIntCharacteristic
// BLEUnsignedIntCharacteristic
// BLELongCharacteristic
// BLEUnsignedLongCharacteristic
// BLEFloatCharacteristic
// BLEDoubleCharacteristic


BLEPeripheral blePeripheral = BLEPeripheral();

//// Gap service ////
BLEService GenericAccess_PS_H = BLEService(GENERIC_ACCESS_SERVICE_UUID);
BLECharCharacteristic GenericAccess_DeviceName_DP_H = BLECharCharacteristic(GENERIC_ACCESS_DEVICE_NAME, BLERead | BLENotify);
BLECharCharacteristic CONN_PARAM_DP_H = BLECharCharacteristic(GENERIC_ACCESS_CONN_PARAM, BLERead | BLENotify);

//// Gatt ////
BLEService GenericAttribute_PS_H = BLEService(GENERIC_ATTR_SERVICE_UUID);
BLECharCharacteristic GenericAccess_Appearance_DP_H = BLECharCharacteristic(GENERIC_ACCESS_APPERANCE, BLERead | BLENotify);

//// Device information service ////
BLEService DeviceInformation_PS_H = BLEService(DEVICE_INFO_SERVICE_UUID);
//BLEStringCharacteristic DeviceInformation_ModName_DP_H = BLEStringCharacteristic(DEVICE_INFO_MOD_NAME, BLERead, sizeof((String)DEVICE_INFO_MOD_NAME));
BLEStringCharacteristic DeviceInformation_SerialN_DP_H = BLEStringCharacteristic(DEVICE_INFO_SERIAL_N, BLERead, sizeof((String)DEVICE_INFO_SERIAL_N));
BLEStringCharacteristic DeviceInformation_FirmRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_FIRM_REV, BLERead, sizeof((String)DEVICE_INFO_FIRM_REV));
//BLEStringCharacteristic DeviceInformation_HardRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_HARD_REV, BLERead, sizeof((String)DEVICE_INFO_HARD_REV));
//BLEStringCharacteristic DeviceInformation_SoftRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_SOFT_REV, BLERead, sizeof((String)DEVICE_INFO_SOFT_REV));
//BLEStringCharacteristic DeviceInformation_ManName_DP_H = BLEStringCharacteristic(DEVICE_INFO_MAN_NAME, BLERead, sizeof((String)DEVICE_INFO_MAN_NAME));

//// Led service ////
// BLEService LedService = BLEService(LED_STATE_SERVICE_UUID);
// BLECharCharacteristic StateLedCharacteristic = BLECharCharacteristic(LED_STATE_SWITH, BLERead | BLEWrite);

//// Temp information service ////
//BLEService TEMP_PS_H = BLEService(TEMP_SERVICE_UUID);
//BLEStringCharacteristic TEMP_LEVEL_INPUT_DP_H = BLEStringCharacteristic(TEMP_LEVEL, BLERead | BLENotify, sizeof((String)TEMP_LEVEL));

//// Battery information service ////
//BLEService BATT_PS_H = BLEService(BATTERY_SERVICE_UUID);
//BLEStringCharacteristic BATT_LEVEL_INPUT_DP_H = BLEStringCharacteristic(BATT_LEVEL, BLERead | BLENotify, sizeof((String)BATT_LEVEL));

//// UART service ////
BLEService UART_SERVICE_H = BLEService(UART_SERVICE_UUID);
BLEStringCharacteristic UART_SERVICE_RX_H = BLEStringCharacteristic(UART_SERVICE_RX, BLEWrite, sizeof((String)UART_SERVICE_RX));
BLEStringCharacteristic UART_SERVICE_TX_H = BLEStringCharacteristic(UART_SERVICE_TX, BLERead | BLENotify, sizeof((String)UART_SERVICE_TX));
BLEDescriptor UART_SERVICE_DESCRIPTOR = BLEDescriptor("2902", "BLE UART");

String cpuID = "";

unsigned long time_connect = 0;

unsigned long local_time = 0;
unsigned long current_time = 0;

// bool flagUpdate = false;
// unsigned long counterTick = 0;


bool new_command = 0;       // 0 - open, 1 - close
bool command = 0;           // 0 - open, 1 - close

bool state = 0;             // 0 - open, 1 - close
bool new_state = 0;         // 0 - open, 1 - close


/**
 * @brief 
 * 
 */
/*
void TIMER1_IRQHandler(void)
{
    // ?????????? ??????????, ???????????????????????????????? ???? ?????????????????? ??????????
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;

    //NRF_TIMER1->TASKS_CLEAR = 1;  // clear the task first to be usable for later

    // ???????????????? ???????????????? ????????????????
	//NRF_ADC->TASKS_STOP = 1;
	NRF_ADC->TASKS_START = 1;
}

void start_timer_1(void)
{
	NRF_TIMER1->POWER = 1;
    
    // ?????????? ??????????????
	NRF_TIMER1->MODE  = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    // ???????????????? ?????????????? ?????????? ???????????????????? ????????????????????. ????????????????: ???????????????? ???????????????? ???????????? (16 MHz) ???? 2 ?? ?????????????? 9 ?? ?????????????????? 31250 "??????????" 
	NRF_TIMER1->PRESCALER = 9; 
	NRF_TIMER1->CC[0] = 31250;
    // ?????????????????? ????????????????????
	NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    // ???????????????????? ???????????????????? ?? NVIC ?? ?????????????????? ????????????????????		
	NVIC_SetPriority(TIMER1_IRQn, 1);
	NVIC_EnableIRQ(TIMER1_IRQn);
    // ???????????? ?????????????? ???? ????????
	NRF_TIMER1->TASKS_START = 1;
}
*/

/**
 * @brief 
 * 
 */
/* ?????????????? ???????????????????? ????????????????????
void ADC_IRQHandler(void)
{
    // ?????????? ??????????, ???????????????????????????????? ???? ?????????????????? ????????????????????????????
	NRF_ADC->EVENTS_END = 0;
    // ???????????????? ???????????????? ???????????? ????????????????????????????
	switch(NRF_ADC->CONFIG >> ADC_CONFIG_PSEL_Pos)
    {
        case (ADC_CONFIG_PSEL_AnalogInput2):
                adc_value = NRF_ADC->RESULT;
                adc_value_in_mV = ((adc_value * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 1024) * ADC_PRE_SCALING_COMPENSATION;
                NRF_ADC->TASKS_STOP = 1;
            break;
        default:
            
            // ?????????? ?????????? ?????????????????????? ?????????????????? ???????????? ?? ?????????????? ????????????
	        break;	
	}
}
*/

/**
 * @brief 
 * 
 */
/*
void adc_init(void)
{
    // ?????????????? ?????????????????????????? ?????? 
    // ?????? - 10 ??????, ???????????????? ???????????????????? - 1/3, ?????????? ???????????????????????????? - 2 
    NRF_ADC->CONFIG  |=  (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) | (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos);

    // ?????????????????? ???????????????????? ???? ?????????????????? ???????????????????????????? 
    NRF_ADC->INTENSET |= ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos;

    // ?????????????????? ??????
    NRF_ADC->ENABLE |= ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos;

    // ???????????????????? ???????????????????? ?? NVIC ?? ?????????????????? ????????????????????	
    NVIC_SetPriority(ADC_IRQn, 0);
    NVIC_EnableIRQ(ADC_IRQn);

    // ?????????????????? ?????????????????? ????????????????????????????	
    // NRF_ADC->TASKS_START = 1;					
}
*/

/**
 * @brief 
 * 
 */
/*
void TIMER2_IRQHandler(void)
{
    if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
    {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;                  //Clear compare register 0 event
        digitalWrite(PIN_LED1, HIGH);                       //Set LED
    }
    if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        NRF_TIMER2->EVENTS_COMPARE[1] = 0;                  //Clear compare register 1 event
        digitalWrite(PIN_LED1, LOW);                        //Clear LED
    }
}
*/

/**
 * @brief 
 * 
 */
/*
void start_timer_2(void)
{
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;               // Set the timer in Counter Mode
    NRF_TIMER2->TASKS_CLEAR = 1;                            // clear the task first to be usable for later
    NRF_TIMER2->PRESCALER = 8;                              //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;      //Set counter to 16 bit resolution
    NRF_TIMER2->CC[0] = 25000;                              //Set value for TIMER2 compare register 0
    NRF_TIMER2->CC[1] = 25;                                  //Set value for TIMER2 compare register 1
    // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) | (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    NVIC_EnableIRQ(TIMER2_IRQn);
    NRF_TIMER2->TASKS_START = 1;               // Start TIMER2
}
*/

/**
 * @brief 
 * 
 */
void SWI2_IRQHandler(void) 
{
    // NOOP
    // sd_power_system_off();
}

/**
 * @brief 
 * 
 */
void SWI2_IRQWakeUp(void)
{
    //Serial.println("New state registered");

    if(digitalRead(PIN_BUTTON1) == LOW && state == 1) 
    {
        new_state = 1;
        state = 0;
    }
    else if(digitalRead(PIN_BUTTON1) == HIGH && state == 0) 
    {
        new_state = 1;
        state = 1;
    }
}

/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    sd_ble_gap_ppcp_set(&gap_conn_params);
}

/**
 * @brief 
 * 
 * @param new_time data
 */
void updateAdvertisingScanData(unsigned long new_time)
{
    unsigned char srData[32];
    unsigned char srDataLen = 0;
    
    int scanDataSize = 3;

    BLEEirData scanData[scanDataSize];

    // - Local name
    scanData[0].length = strlen(DEVICE_NAME);
    scanData[0].type = 0x09;
    memcpy(scanData[0].data, DEVICE_NAME, scanData[0].length);

    // - Tx Power
    scanData[1].length = 1;
    scanData[1].type = 0x0A;
    scanData[1].data[0] = TX_POWER;

    // - Manufacturer Data
    scanData[2].length = 4;
    scanData[2].type = 0xFF;
    
    // Manufacturer ID
    scanData[2].data[0] = 0xFF;
    scanData[2].data[1] = 0xFF;

    // Manufacturer data content
    scanData[2].data[2] = new_time & 0xFF;
    scanData[2].data[3] = (new_time >> 8) & 0xFF;
    scanData[2].data[6] = (new_time >> 16) & 0xFF;
    scanData[2].data[7] = (new_time >> 24) & 0xFF;

    if (scanDataSize && scanData)
    {
        for (int i = 0; i < scanDataSize; i++)
        {
            srData[srDataLen + 0] = scanData[i].length + 1;
            srData[srDataLen + 1] = scanData[i].type;
            srDataLen += 2;

            memcpy(&srData[srDataLen], scanData[i].data, scanData[i].length);

            srDataLen += scanData[i].length;
        }
    }

    // - Sets only avertising scan data
    sd_ble_gap_adv_data_set(srData, srDataLen, srData, srDataLen);
}

/**
 * @brief 
 * 
 * @param central 
 * @param characteristic 
 */
void characteristicWrittenCallback(BLECentral& central, BLECharacteristic& characteristic) 
{
    // central wrote new value to characteristic
    if (UART_SERVICE_RX_H.value()) 
    {
        String message =  UART_SERVICE_RX_H.value();

        //Serial.println("*********");
        //Serial.print("Received Value: ");

        //Serial.print(message);

        //Serial.println();
        //Serial.println("*********");

        if(message.indexOf("open") > -1) { command = 1; new_command = 1; }
        
        if(message.indexOf("close") > -1) { command = 0; new_command = 1; }

        if(message.indexOf("speed") > -1) { blePeripheral.setAdvertisingInterval(1000); }

        if(message.indexOf("norm") > -1) { blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL * 1000); }

    }

    // central wrote new value to characteristic, set state (LED)
    //if (StateLedCharacteristic.value()) 
    //{
    //    // digitalWrite(PIN_LED2, HIGH);
    //}
    //else 
    //{
    //    // digitalWrite(PIN_LED2, LOW);
    //}
}

/**
 * @brief 
 * 
 * @param central 
 */
void blePeripheralConnectHandler(BLECentral& central) 
{
    // DEBUG_PRINTLN(F("Connected event, central: "));
    // DEBUG_PRINTLN(central.address());
}

/**
 * @brief 
 * 
 * @param central 
 */
void blePeripheralDisconnectHandler(BLECentral& central) 
{
    // DEBUG_PRINTLN(F("Disconnected event, central: "));
    // DEBUG_PRINTLN(central.address());
}

/**
 * @brief 
 * 
 */
void services_init()
{
    //// Gap service ////
    /* 
        blePeripheral.setAdvertisedServiceUuid(GenericAccess_PS_H.uuid());
        blePeripheral.addAttribute(GenericAccess_PS_H);
        blePeripheral.addAttribute(GenericAccess_DeviceName_DP_H);
        blePeripheral.addAttribute(CONN_PARAM_DP_H);
    */

    //// Gatt ////
    /* 
        blePeripheral.setAdvertisedServiceUuid(GenericAttribute_PS_H.uuid());
        blePeripheral.addAttribute(GenericAttribute_PS_H);
        blePeripheral.addAttribute(GenericAccess_Appearance_DP_H);
    */

    //// Device information service ////
    /*blePeripheral.setAdvertisedServiceUuid(DeviceInformation_PS_H.uuid());
    blePeripheral.addAttribute(DeviceInformation_PS_H);
    blePeripheral.addAttribute(DeviceInformation_SerialN_DP_H); 
    blePeripheral.addAttribute(DeviceInformation_FirmRev_DP_H); */
    //blePeripheral.addAttribute(DeviceInformation_HardRev_DP_H); 

    //// Led service ////
    // blePeripheral.setAdvertisedServiceUuid(LedService.uuid());
    // blePeripheral.addAttribute(LedService);
    // blePeripheral.addAttribute(StateLedCharacteristic);

    //// Temp information service ////
    //blePeripheral.setAdvertisedServiceUuid(TEMP_PS_H.uuid());
    //blePeripheral.addAttribute(TEMP_PS_H);
    //blePeripheral.addAttribute(TEMP_LEVEL_INPUT_DP_H);

    //// Battery information service ////
    //blePeripheral.setAdvertisedServiceUuid(BATT_PS_H.uuid());
    //blePeripheral.addAttribute(BATT_PS_H);
    //blePeripheral.addAttribute(BATT_LEVEL_INPUT_DP_H);  

    //// UART service ////
    blePeripheral.setAdvertisedServiceUuid(UART_SERVICE_H.uuid());
    blePeripheral.addAttribute(UART_SERVICE_H);
    blePeripheral.addAttribute(UART_SERVICE_TX_H);    
    blePeripheral.addAttribute(UART_SERVICE_RX_H);
    blePeripheral.addAttribute(UART_SERVICE_DESCRIPTOR);

       
}

/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init()
{
    // blePeripheral.setConnectionInterval(200, 5000);
    blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL * 1000);    
}

/**
 * @brief Get the cpiID object
 * 
 */
void get_cpuID()
{
    // DEBUG_PRINTLN("SAADC Low Power Example; built on " __DATE__ " at " __TIME__ " for " BOARD_STR );	
    // cpuID += NRF_FICR->DEVICEADDR[1];	
    cpuID += NRF_FICR->DEVICEADDR[0];	
}

/**
 * @brief 
 * 
 */
void gpio_output_init()
{
    //pinMode(PIN_LED1, OUTPUT); 
    pinMode(PIN_LED2, OUTPUT); digitalWrite(PIN_LED2, LOW);
    pinMode(PIN_LED3, OUTPUT); digitalWrite(PIN_LED3, LOW);
    //pinMode(PIN_LED4, OUTPUT);    
    //pinMode(PIN_LED5, OUTPUT);    
}

/**
 * @brief 
 * 
 */
void setup()
{
    //Serial.begin(115200);
    
    // pinMode(GI_TMP_PIN, INPUT);
    // pinMode(GI_BAT_PIN, INPUT);

    // analogReadResolution(12);

    pinMode(PIN_BUTTON1, INPUT);
    // digitalWrite(PIN_BUTTON1, HIGH);

    gpio_output_init();

    get_cpuID();

    blePeripheral.setLocalName(LOCALNAME);
    blePeripheral.setDeviceName(DEVICE_NAME);
    // blePeripheral.setAppearance(0x0080);

    //gap_params_init();
    services_init();
    advertising_init();

    // blePeripheral.setConnectionInterval(200, 5000);
    // blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL * 1000);   

    blePeripheral.setTxPower(TX_POWER);

    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    UART_SERVICE_RX_H.setEventHandler(BLEWritten, characteristicWrittenCallback);


    blePeripheral.begin();

    DeviceInformation_SerialN_DP_H.writeValue((String)NRF_FICR->DEVICEADDR[0]);
    DeviceInformation_FirmRev_DP_H.writeValue((String)FIRMWARE);
    //DeviceInformation_HardRev_DP_H.writeValue((String)REVISION);

    updateAdvertisingScanData(0);

    // adc_init();
    // start_timer_1();
    // start_timer_2();

    // NRF_ADC->TASKS_STOP = 1;					
    // NRF_TIMER1->TASKS_STOP = 1; // Start TIMER1
    // NRF_TIMER2->TASKS_STOP = 1; // Start TIMER2

    // DEBUG_PRINTLN("");
    // DEBUG_PRINTLN("=============");
    // DEBUG_PRINTLN("Device booted");
    // DEBUG_PRINTLN("=============");
    
    // DEBUG_PRINTLN(cpuID);	
    // DEBUG_PRINTLN("=============");
    // DEBUG_PRINTLN("NRF_POWER_MODE_LOWPWR");	
    // DEBUG_PRINTLN("=============");

    attachInterrupt(PIN_BUTTON1, SWI2_IRQWakeUp, RISING);

    // enable low power mode without interrupt
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR); //     sd_power_mode_set(SD_POWER_SYSTEM_OFF);

}

/**
 * @brief 
 * 
 */
/*
String getVoltage()
{
    uint16_t v = analogRead(GI_BAT_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (ADC_REF_VOLTAGE_IN_MILLIVOLTS / 1000.0);
    return String(battery_voltage) + "V";
}
*/

/**
 * @brief 
 * 
 */
void loop()
{
    // Clear IRQ flag to be able to go to sleep if nothing happens in between
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);
    
    if(new_state == 1)
    {
        if(state == 0) 
        {
            updateAdvertisingScanData(0); 
            //digitalWrite(PIN_LED5, LOW);
        }
        else
        {
            updateAdvertisingScanData(1);
           //digitalWrite(PIN_LED5, HIGH);
        } 

        new_state = 0;
        delay(10);
    }

    // Clear IRQ flag to be able to go to sleep if nothing happens in between
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);

    // Enter Low power mode
    // sd_app_evt_wait();

    // Exit Low power mode

    // digitalWrite(PIN_LED1, HIGH);
    // delay(200); 
    // digitalWrite(PIN_LED1, LOW);
    // delay(100);

    // Clear IRQ flag to be able to go to sleep if nothing happens in between
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);
    
    blePeripheral.poll();
    BLECentral central = blePeripheral.central();

    // If Connected client BLE
    if (central)
    {

        //Serial.println("Connected");
        
        time_connect = millis();

        // If Connected client BLE
        while (central.connected())
        {
            if(new_state == 1)
            {
                if(state == 0) 
                {
                    updateAdvertisingScanData(0); 
                    //digitalWrite(PIN_LED5, LOW);
                }
                else
                {
                    updateAdvertisingScanData(1);
                    //digitalWrite(PIN_LED5, HIGH);
                } 

                new_state = 0;
            }

            if(new_command == 1)
            {
                new_command = 0;

                if(command == 0)
                {
                    digitalWrite(PIN_LED2, HIGH);
                    delay(500); 
                    digitalWrite(PIN_LED2, LOW);

                    time_connect = millis() + APP_ADV_TIMEOUT_IN_SECONDS * 1000;
                }

                if(command == 1)
                {           
                    digitalWrite(PIN_LED3, HIGH);
                    delay(500); 
                    digitalWrite(PIN_LED3, LOW);
    
                    time_connect = millis() + APP_ADV_TIMEOUT_IN_SECONDS * 1000;
                }
            }

            if(millis() > time_connect + APP_ADV_TIMEOUT_IN_SECONDS * 1000) { central.disconnect(); blePeripheral.disconnect(); }

            delay(10); 
            sd_nvic_ClearPendingIRQ(SWI2_IRQn);
        }
            
        //Serial.println("Disconnected");
    }

    // central.disconnect();
    // blePeripheral.disconnect();

    sd_app_evt_wait();		

    /*
    lastTime = millis();
    TIME_LOCAL_DP_H.setValue(lastTime);
    int32_t temperature = 0;   
    sd_temp_get(&temperature);
    float t = temperature / 4.0;
    TEMP_LEVEL_INPUT_DP_H.writeValue((String)t);
    int level = analogRead(GI_Bat_Pin); // sample ADC to get battery voltage level
    // float bat_value_in = (127.0f / 100.0f) * 3.30f * ((float)level) / 4095.0f; // convert to the LiPo battery voltage
    int bat_value = map(level, 0, 4095, 0, 100); // map battery level from 0 - 100 %
    BATT_LEVEL_INPUT_DP_H.setValue(bat_value);
    */

    //char charBufID[20];
    //cpuID.toCharArray(charBufID, 20);
    //deviceIdCharacteristic.setValue(millis());//*charBufID);
    // (unsigned char*)&value, sizeof(T)

    // Clear IRQ flag to be able to go to sleep if nothing happens in between
    //sd_nvic_ClearPendingIRQ(SWI2_IRQn);

    // updateAdvertisingScanData(local_time);

    //blePeripheral.poll();

    //BLECentral central = blePeripheral.central();

   /* if (central)
    {
        time_connect = millis();

        while (central.connected())
        {
            if(millis() > time_connect + APP_ADV_TIMEOUT_IN_SECONDS * 1000) 
            {
                central.disconnect();
                break;
            }

            local_time = millis();
         /////   TIME_LOCAL_DP_H.setValue(local_time); 

            int32_t temperature = 0;   
            sd_temp_get(&temperature);
            float t = temperature / 4.0;
            TEMP_LEVEL_INPUT_DP_H.writeValue((String)t);

            int bat_level = analogRead(GI_BAT_PIN);
            float battery_voltage = ((float)bat_level / 2800.0) * 3.3 * (3300.0 / 2800.0);
            BATT_LEVEL_INPUT_DP_H.writeValue((String)battery_voltage);

            sd_nvic_ClearPendingIRQ(SWI2_IRQn);

            if (UART_SERVICE_RX_H.written())
            {
                unsigned long new_time = UART_SERVICE_RX_H.value();
                updateAdvertisingScanData(new_time);

            }

            delay(1000);
        }
    } 
    */
}
