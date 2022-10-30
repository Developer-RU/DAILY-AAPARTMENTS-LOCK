#ifndef _BLE_STRING_CHARACTERISTIC_H_
#define _BLE_STRING_CHARACTERISTIC_H_


#include "Arduino.h"
#include "BLECharacteristic.h"


/**
 * @brief 
 * 
 */
class BLEStringCharacteristic : public BLECharacteristic
{
    /**
     * @brief 
     * 
     */
    public:

    BLEStringCharacteristic(const char* uuid, unsigned char properties, int valueSize);

    int writeValue(const String& value);
    int setValue(const String& value) { return setValue(value); }
    String value(void);

    private:

    /**
     * @brief 
     * 
     */

    /*
    *
    *
    */
};


#endif