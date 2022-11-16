#include "BLEStringCharacteristic.hpp"


/**
 * @brief Construct a new BLEStringCharacteristic::BLEStringCharacteristic object
 * 
 * @param uuid 
 * @param properties 
 * @param valueSize 
 */
BLEStringCharacteristic::BLEStringCharacteristic(const char* uuid, unsigned char properties, int valueSize) :
    BLECharacteristic(uuid, properties, valueSize) {
}

/**
 * @brief 
 * 
 * @param value 
 * @return int 
 */
int BLEStringCharacteristic::writeValue(const String& value)
{
    return BLECharacteristic::setValue(value.c_str());
}

/**
 * @brief 
 * 
 * @return String 
 */
String BLEStringCharacteristic::value(void)
{
    String str;
    int length = BLECharacteristic::valueLength();
    const uint8_t* val = BLECharacteristic::value();

    str.reserve(length);

    for (int i = 0; i < length; i++) 
    {
        str += (char)val[i];
    }

    return str;
}
