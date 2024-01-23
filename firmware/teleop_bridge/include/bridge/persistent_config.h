#include <Arduino.h>
#include <EEPROM.h>
#include <bridge/structs.h>

#ifndef __BRIDGE_PERSISTENT_CONFIG_H__
#define __BRIDGE_PERSISTENT_CONFIG_H__
namespace persistent_config
{
    class PersistentConfig
    {
    public:
        const int CONFIG_SET_ADDRESS = 1; // EEPROM address to store whether config is set
        const int MAX_EEPROM_SIZE = 512;  // Maximum size of the EEPROM

        static PersistentConfig *get_instance()
        {
            static PersistentConfig instance;
            return &instance;
        }

        /**
         * @brief Initialize the EEPROM. Hangs if the EEPROM fails to initialize.
         */
        void begin();

        /**
         * @brief Check if the config is set in EEPROM
         *
         * @return True if the config is set, false otherwise
         */
        bool is_set();

        /**
         * @brief Unset the config in EEPROM
         */
        void unset();

        /**
         * @brief Write the config to EEPROM
         *
         * @param config The config struct pointer to write
         */
        void write(bridge::config_info_p config);

        /**
         * @brief Read the config from EEPROM
         *
         * @param config The config struct pointer to read into
         */
        void read(bridge::config_info_p config);

        /**
         * @brief Check if the serial port has a "clear" command
         *
         * If the serial port has a "clear" command, unset the config and restart the
         * device.
         */
        void check_clear();

    private:
        PersistentConfig(){};
    };
} // namespace persistent_config
#endif // __BRIDGE_PERSISTENT_CONFIG_H__