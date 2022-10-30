#include "main.hpp"


#ifdef __cplusplus
extern "C" {
#endif
// void SWI2_IRQn(void);
void SWI2_IRQHandler(void);
//void TIMER1_IRQHandler(void);
//void TIMER2_IRQHandler(void);
//void ADC_IRQHandler(void);
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
BLEStringCharacteristic DeviceInformation_ModName_DP_H = BLEStringCharacteristic(DEVICE_INFO_MOD_NAME, BLERead, sizeof((String)DEVICE_INFO_MOD_NAME));
BLEStringCharacteristic DeviceInformation_SerialN_DP_H = BLEStringCharacteristic(DEVICE_INFO_SERIAL_N, BLERead, sizeof((String)DEVICE_INFO_SERIAL_N));
BLEStringCharacteristic DeviceInformation_FirmRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_FIRM_REV, BLERead, sizeof((String)DEVICE_INFO_FIRM_REV));
BLEStringCharacteristic DeviceInformation_HardRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_HARD_REV, BLERead, sizeof((String)DEVICE_INFO_HARD_REV));
BLEStringCharacteristic DeviceInformation_SoftRev_DP_H = BLEStringCharacteristic(DEVICE_INFO_SOFT_REV, BLERead, sizeof((String)DEVICE_INFO_SOFT_REV));
BLEStringCharacteristic DeviceInformation_ManName_DP_H = BLEStringCharacteristic(DEVICE_INFO_MAN_NAME, BLERead, sizeof((String)DEVICE_INFO_MAN_NAME));

//// Led service ////
// BLEService LedService = BLEService(LED_STATE_SERVICE_UUID);
// BLECharCharacteristic StateLedCharacteristic = BLECharCharacteristic(LED_STATE_SWITH, BLERead | BLEWrite);

//// Temp information service ////
BLEService TEMP_PS_H = BLEService(TEMP_SERVICE_UUID);
BLEStringCharacteristic TEMP_LEVEL_INPUT_DP_H = BLEStringCharacteristic(TEMP_LEVEL, BLERead | BLENotify, sizeof((String)TEMP_LEVEL));

//// Battery information service ////
BLEService BATT_PS_H = BLEService(BATTERY_SERVICE_UUID);
BLEStringCharacteristic BATT_LEVEL_INPUT_DP_H = BLEStringCharacteristic(BATT_LEVEL, BLERead | BLENotify, sizeof((String)BATT_LEVEL));

//// Time information service ////
BLEService TIME_PS_H = BLEService(TIME_SERVICE_UUID);
BLEUnsignedLongCharacteristic TIME_LOCAL_DP_H = BLEUnsignedLongCharacteristic(TIME_LOCAL, BLERead | BLENotify);
BLEUnsignedLongCharacteristic TIME_CURRENT_DP_H = BLEUnsignedLongCharacteristic(TIME_CURRENT, BLERead | BLEWrite);


String cpuID = "";

unsigned long time_connect = 0;

unsigned long local_time = 0;
unsigned long current_time = 0;

// bool flagUpdate = false;
// unsigned long counterTick = 0;


/**
 * @brief 
 * 
 */
/*
void TIMER1_IRQHandler(void)
{
    // Сброс флага, сигнализирующего об окончании счета
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    //NRF_TIMER1->TASKS_CLEAR = 1;  // clear the task first to be usable for later
    // Очищение счетного регистра
	//NRF_ADC->TASKS_STOP = 1;
	NRF_ADC->TASKS_START = 1;
}
void start_timer_1(void)
{
	NRF_TIMER1->POWER = 1;
    
    // Режим таймера
	NRF_TIMER1->MODE  = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    // Получить секунду можно различными вариантами. Например: поделить тактовый сигнал (16 MHz) на 2 в степени 9 и отсчитать 31250 "тиков" 
	NRF_TIMER1->PRESCALER = 9; 
	NRF_TIMER1->CC[0] = 31250;
    // Активация прерывания
	NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    // Разрешение прерываний в NVIC и установка приоритета		
	NVIC_SetPriority(TIMER1_IRQn, 1);
	NVIC_EnableIRQ(TIMER1_IRQn);
    // Запуск таймера на счет
	NRF_TIMER1->TASKS_START = 1;
}
*/

/**
 * @brief 
 * 
 */
/* Функция обработчик прерывания
void ADC_IRQHandler(void)
{
    // Сброс флага, сигнализирующего об окончании преобразования
	NRF_ADC->EVENTS_END = 0;
    // Проверка текущего канала преобразования
	switch(NRF_ADC->CONFIG >> ADC_CONFIG_PSEL_Pos)
    {
        case (ADC_CONFIG_PSEL_AnalogInput2):
                adc_value = NRF_ADC->RESULT;
                adc_value_in_mV = ((adc_value * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 1024) * ADC_PRE_SCALING_COMPENSATION;
                NRF_ADC->TASKS_STOP = 1;
            break;
        default:
            
            // здесь можно реализовать обработку данных с нужного канала
	        break;	
	}
}
*/

/**
 * @brief 
 * 
 */
void adc_init(void)
{
    /* Функция инициализации АЦП */
    /* АЦП - 10 бит, Делитель напряжения - 1/3, Канал преобразования - 2 */
    NRF_ADC->CONFIG  |=  (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) | (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos);

    /* Активация прерывания по окончанию преобразования */
    NRF_ADC->INTENSET |= ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos;

    /* Включение АЦП */
    NRF_ADC->ENABLE |= ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos;

    /* Разрешение прерываний в NVIC и установка приоритета */	
    NVIC_SetPriority(ADC_IRQn, 0);
    NVIC_EnableIRQ(ADC_IRQn);

    /* Запустить одиночное преобразование */	
    // NRF_ADC->TASKS_START = 1;					
}

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
    
        counterTick++; if(counterTick == 100) { counterTick = 0; flagUpdate = true; }
    }
}

/**
 * @brief 
 * 
 */
/*
void start_timer_2(void)
{
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;               // Set the timer in Counter Mode
    NRF_TIMER2->TASKS_CLEAR = 1;                            // clear the task first to be usable for later
    NRF_TIMER2->PRESCALER = 6;                              //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
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
 * @param new_time 
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
    // central wrote new value to characteristic, set state (LED)
    //if (StateLedCharacteristic.value()) 
    //{
    //    // digitalWrite(PIN_LED2, HIGH);
    //}
    //else 
    //{
    //    // digitalWrite(PIN_LED2, LOW);
    //}

    // central wrote new value to characteristic, get temp (TEMP_LEVEL_INPUT_DP_H)
    //if (TEMP_LEVEL_INPUT_DP_H.value()) 
    //{
    //    // digitalWrite(PIN_LED3, HIGH);
    //}
    //else 
    //{
    //    // digitalWrite(PIN_LED3, LOW);
    //}

    // central wrote new value to characteristic, get power (BATT_LEVEL_INPUT_DP_H)
    //if (BATT_LEVEL_INPUT_DP_H.value()) 
    //{
    //    // digitalWrite(PIN_LED4, HIGH);
    //}
    //else 
    //{
    //    // digitalWrite(PIN_LED4, LOW);
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
    blePeripheral.setAdvertisedServiceUuid(GenericAccess_PS_H.uuid());
    blePeripheral.addAttribute(GenericAccess_PS_H);
    blePeripheral.addAttribute(GenericAccess_DeviceName_DP_H);
    blePeripheral.addAttribute(CONN_PARAM_DP_H);
    
    //// Gatt ////
    blePeripheral.setAdvertisedServiceUuid(GenericAttribute_PS_H.uuid());
    blePeripheral.addAttribute(GenericAttribute_PS_H);
    blePeripheral.addAttribute(GenericAccess_Appearance_DP_H);

    //// Device information service ////
    blePeripheral.setAdvertisedServiceUuid(DeviceInformation_PS_H.uuid());
    blePeripheral.addAttribute(DeviceInformation_PS_H);
    blePeripheral.addAttribute(DeviceInformation_SerialN_DP_H); 
    blePeripheral.addAttribute(DeviceInformation_FirmRev_DP_H); 
    blePeripheral.addAttribute(DeviceInformation_HardRev_DP_H); 

    //// Led service ////
    // blePeripheral.setAdvertisedServiceUuid(LedService.uuid());
    // blePeripheral.addAttribute(LedService);
    // blePeripheral.addAttribute(StateLedCharacteristic);

    //// Temp information service ////
    blePeripheral.setAdvertisedServiceUuid(TEMP_PS_H.uuid());
    blePeripheral.addAttribute(TEMP_PS_H);
    blePeripheral.addAttribute(TEMP_LEVEL_INPUT_DP_H);

    //// Battery information service ////
    blePeripheral.setAdvertisedServiceUuid(BATT_PS_H.uuid());
    blePeripheral.addAttribute(BATT_PS_H);
    blePeripheral.addAttribute(BATT_LEVEL_INPUT_DP_H);  

    //// Time information service ////
    blePeripheral.setAdvertisedServiceUuid(TIME_PS_H.uuid());
    blePeripheral.addAttribute(TIME_PS_H);
    blePeripheral.addAttribute(TIME_LOCAL_DP_H);    
    blePeripheral.addAttribute(TIME_CURRENT_DP_H);    
}

/**
 * @brief Get the cpiID object
 * 
 */
void get_cpiID()
{
    // DEBUG_PRINTLN("SAADC Low Power Example; built on " __DATE__ " at " __TIME__ " for " BOARD_STR );	
    // cpuID += NRF_FICR->DEVICEADDR[1];	
    cpuID += NRF_FICR->DEVICEADDR[0];	
}

/**
 * @brief 
 * 
 */
void leds_init()
{
    pinMode(PIN_LED1, OUTPUT);

    // pinMode(PIN_LED2, OUTPUT);
    // pinMode(PIN_LED3, OUTPUT);
    // pinMode(PIN_LED4, OUTPUT);    
}

/**
 * @brief 
 * 
 */
void setup()
{
    // Serial.begin(115200);
    
    // pinMode(GI_TMP_PIN, INPUT);
    pinMode(GI_BAT_PIN, INPUT);

    analogReadResolution(12);

    leds_init();
    get_cpiID();

    blePeripheral.setLocalName(LOCALNAME);
    blePeripheral.setDeviceName(DEVICE_NAME);
    // blePeripheral.setAppearance(0x0080);

    services_init();

    blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);    
    //blePeripheral.setConnectionInterval(1000, 5000);
    blePeripheral.setTxPower(TX_POWER);


    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // StateLedCharacteristic.setEventHandler(BLEWritten, characteristicWrittenCallback);

    blePeripheral.begin();

    DeviceInformation_SerialN_DP_H.writeValue((String)NRF_FICR->DEVICEADDR[0]);
    DeviceInformation_FirmRev_DP_H.writeValue((String)FIRMWARE);
    DeviceInformation_HardRev_DP_H.writeValue((String)REVISION);

    updateAdvertisingScanData(0);

    //adc_init();
    //start_timer_1();
    //start_timer_2();

    //NRF_ADC->TASKS_STOP = 1;					
    //NRF_TIMER1->TASKS_STOP = 1; // Start TIMER1
    //NRF_TIMER2->TASKS_STOP = 1; // Start TIMER2

    // DEBUG_PRINTLN("");
    // DEBUG_PRINTLN("=============");
    // DEBUG_PRINTLN("Device booted");
    // DEBUG_PRINTLN("=============");
    
    // DEBUG_PRINTLN(cpuID);	
    // DEBUG_PRINTLN("=============");
    // DEBUG_PRINTLN("NRF_POWER_MODE_LOWPWR");	
    // DEBUG_PRINTLN("=============");


    // enable low power mode without interrupt
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
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
    
    // Enter Low power mode
    sd_app_evt_wait();

    // Exit Low power mode

    // Clear IRQ flag to be able to go to sleep if nothing happens in between
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);


    //digitalWrite(PIN_LED1, HIGH);
    //delay(500); 
    //digitalWrite(PIN_LED1, LOW);

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
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);

    // updateAdvertisingScanData(local_time);

    blePeripheral.poll();

    BLECentral central = blePeripheral.central();

    if (central)
    {
        time_connect = millis();

        while (central.connected())
        {
            if(millis() > time_connect + 60000) 
            {
                central.disconnect();
                break;
            }

            local_time = millis();
            TIME_LOCAL_DP_H.setValue(local_time); 

            float temperature = 36.6;//sensors.getTempCByIndex(0);
            TEMP_LEVEL_INPUT_DP_H.writeValue((String)temperature);

            int bat_level = analogRead(GI_BAT_PIN);
            float battery_voltage = ((float)bat_level / 2800.0) * 3.3 * (3300.0 / 2800.0);
            BATT_LEVEL_INPUT_DP_H.writeValue((String)battery_voltage);

            sd_nvic_ClearPendingIRQ(SWI2_IRQn);

            if (TIME_CURRENT_DP_H.written())
            {
                unsigned long new_time = TIME_CURRENT_DP_H.value();
                updateAdvertisingScanData(new_time);

            }

            delay(1000);
        }
    } 

    sd_app_evt_wait();		
}
