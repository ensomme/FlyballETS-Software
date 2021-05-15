#include "Arduino.h"

//Includes
#include <Structs.h>
#include <config.h>

//Public libs
#include <LiquidCrystal.h>
#ifndef WiFiOFF
    #include <WiFi.h>
    #include <ESPmDNS.h>
    #include <ArduinoOTA.h>
    #include <WiFiUdp.h>
    #include <WiFiClientSecure.h>
#endif
#include <EEPROM.h>

//Private libs
#include <GPSHandler.h>
#include <SettingsManager.h>
#ifndef WiFiOFF
    #include <WebHandler.h>
    #include <SlaveHandler.h>
    #include <WifiManager.h>
#endif
#include <RaceHandler.h>
#include <LCDController.h>
#include <LightsController.h>
#include <BatterySensor.h>
#include <SystemManager.h>

//Set simulate to true to enable simulator class (see Simulator.cpp/h)
#if Simulate
#include "Simulator.h"
#endif

#ifdef WS281x
//#include <Adafruit_NeoPixel.h>
#include <NeoPixelBus.h>
#endif // WS281x

//Function prototypes
void Sensor1Wrapper();
void Sensor2Wrapper();
void ResetRace();
void mdnsServerSetup();
void StartStopRace();
void StopRaceMain();
void StartRaceMain();
void serialEvent();
void Core0Loop(void *parameter);
void HandleSerialMessages();
void HandleRemoteControl();
void HandleLCDUpdates();
