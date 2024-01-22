#include <bridge/persistent_config.h>

using namespace persistent_config;
using namespace bridge;

void PersistentConfig::begin()
{
    // Initialize EEPROM
    if (!EEPROM.begin(MAX_EEPROM_SIZE))
    {
        while (true)
        {
            Serial.println("failed to initialize EEPROM");
            delay(1000);
        }
    }
    delay(100);
}

bool PersistentConfig::is_set()
{
    uint8_t is_config;
    EEPROM.get(CONFIG_SET_ADDRESS, is_config);
    return is_config == 0x01;
}

void PersistentConfig::unset()
{
    EEPROM.put(CONFIG_SET_ADDRESS, 0x00);
    if (EEPROM.commit())
    {
        Serial.println("Config unset");
    }
    else
    {
        Serial.println("Failed to unset config");
    }
}

void PersistentConfig::write(config_info_p config)
{
    EEPROM.put(CONFIG_SET_ADDRESS, 0x01);
    EEPROM.put(CONFIG_SET_ADDRESS + 1, *config);
    if (EEPROM.commit())
    {
        Serial.println("Config written to EEPROM");
    }
    else
    {
        Serial.println("Failed to write config to EEPROM");
    }
}

void PersistentConfig::read(config_info_p config)
{
    config_info_t read_config;
    EEPROM.get(CONFIG_SET_ADDRESS + 1, read_config);
    *config = read_config;
}

void PersistentConfig::check_clear()
{
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        if (command.equals("clear"))
        {
            unset();
            ESP.restart();
        }
    }
}