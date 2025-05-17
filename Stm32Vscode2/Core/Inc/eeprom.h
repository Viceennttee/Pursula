#ifndef EEPROM_H
#define EEPROM_H
#define BMP280_ADDR (0x76 << 1)  // Dirección I2C del BMP280 (0x76 si SDO=GND, 0x77 si SDO=VCC)
#define BMP280_REG_ID 0xD0       // Registro del ID (debe devolver 0x58)


#define EEPROM_ADDRESS 0xA0 //es la dirección 0x50 << 1, porque todas las A van a 0
#define TIMEOUT 100

#endif
