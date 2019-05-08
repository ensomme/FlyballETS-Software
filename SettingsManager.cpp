#include "SettingsManager.h"
#include "Embedis.h"
#include <EEPROM.h>
#include <SPIFFS.h>
#include <StreamString.h>
#include "config.h"

void SettingsManagerClass::loop()
{
   if (_settings_save) {
      ESP_LOGD(TAG, "[SETTINGS] Saving");
      ESP_LOGD(TAG, "Writing settings\r\n");
      EEPROM.commit();
      _settings_save = false;
   }
}

void SettingsManagerClass::init()
{
   EEPROM.begin(EEPROM_SIZE);
   _settings_save = false;

   Embedis::dictionary(F("EEPROM"),
      SPI_FLASH_SEC_SIZE,
      [](size_t pos) -> char { return EEPROM.read(pos); },
      [](size_t pos, char value) { EEPROM.write(pos, value); },
      []() {}
   );
   setDefaultSettings();
}

String SettingsManagerClass::getSetting(const String& key, String defaultValue)
{
   String value;
   if (!Embedis::get(key, value)) value = defaultValue;
   return value;
}

String SettingsManagerClass::getSetting(const String& key)
{
   String strReturnValue = getSetting(key, "");
   return strReturnValue;
}

//template<typename T> bool SettingsManagerClass::setSetting(const String& key, T value)
bool SettingsManagerClass::setSetting(const String& key, String value)
{
   saveSettings();
   return Embedis::set(key, String(value));
}

void SettingsManagerClass::saveSettings()
{
   _settings_save = true;
   ESP_LOGD(TAG, "Saving settings\r\n");
}

bool SettingsManagerClass::hasSetting(const String& key)
{
   return getSetting(key).length() != 0;
}

void SettingsManagerClass::setDefaultSettings()
{
   if (!hasSetting("AdminPass")) {
      setSetting("AdminPass", "FlyballETS.1234");
      saveSettings();
   }

   if (!hasSetting("APName")) {
      setSetting("APName", "FlyballETS");
      saveSettings();
   }

   if (!hasSetting("APPass")) {
      setSetting("APPass", "FlyballETS.1234");
      saveSettings();
   }

   if (!hasSetting("RunDirectionInverted")) {
      setSetting("RunDirectionInverted", "0");
      saveSettings();
   }

   if (!hasSetting("OperationMode")) {
      setSetting("OperationMode", "0");
      saveSettings();
   }
}

SettingsManagerClass SettingsManager;
