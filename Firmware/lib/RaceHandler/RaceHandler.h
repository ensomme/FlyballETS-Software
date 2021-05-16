// RaceHandler.h
#ifndef _RACEHANDLER_h
#define _RACEHANDLER_h

#include <Arduino.h>
#include <config.h>


#define NUM_HISTORIC_RACE_RECORDS 100
#include <Structs.h>

class RaceHandlerClass
{
   friend class SimulatorClass;
   friend class WebHandlerClass;
protected:


public:
	void init(uint8_t iS1Pin, uint8_t iS2Pin);
   enum RaceStates {
      RESET,
      STARTING,
      RUNNING,
      STOPPED,
      SCHEDULED
   };

   RaceStates RaceState = RaceStates::RESET;
   RaceStates PreviousRaceState = RaceStates::RESET;

   uint8_t iCurrentDog;
   uint8_t iPreviousDog;
   uint8_t iNextDog;
   uint8_t iDogRunCounters[4];  //Number of (re-)runs for each dog
   unsigned long long llRaceStartTime;

   void Main();
   void StartTimers();
   void StartRace(unsigned long long StartTime);
   void StartRace();
   void StopRace();
   void StopRace(unsigned long long llStopTime);
   void ResetRace();
   void PrintRaceTriggerRecords();
   void TriggerSensor1();
   void TriggerSensor2();

   enum DogFaults {
      OFF,
      ON,
      TOGGLE
   };
   void SetDogFault(uint8_t iDogNumber, DogFaults State = TOGGLE);

   double GetRaceTime();
   String GetDogTime(uint8_t iDogNumber, int8_t iRunNumber = -1);
   String GetStoredDogTimes(uint8_t iDogNumber, int8_t iRunNumber = -1);
   int8_t SelectRunNumber(uint8_t iDogNumber, int8_t iRunNumber = -1);
   String GetCrossingTime(uint8_t iDogNumber, int8_t iRunNumber = -1);
   String TransformCrossingTime(uint8_t iDogNumber, int8_t iRunNumber);
   String GetRerunInfo(uint8_t iDogNumber);
   double GetNetTime();

   String GetRaceStateString();

   stRaceData GetRaceData();
   stRaceData GetRaceData(uint iRaceId);
   void ToggleRunDirection();
   boolean GetRunDirection();

private:
   unsigned long long _lSchduledRaceStartTime;
   unsigned long long _llRaceEndTime;
   unsigned long long _llRaceTime;
   unsigned long long _llLastDogExitTime;
   unsigned long long _llS2CrossedSafeTime;
   unsigned long long _llS2CrossedUnsafeTime;
   unsigned long long _llRaceElapsedTime;
   unsigned long _lLastTransitionStringUpdate;

   uint8_t _iS1Pin;
   uint8_t _iS2Pin;
   boolean _bRunDirectionInverted = false;
   boolean _bNextDogFound = false;

   struct STriggerRecord
   {
      volatile int iSensorNumber;
      volatile long long llTriggerTime;
      volatile int iSensorState;
   };
   
   #define TRIGGER_QUEUE_LENGTH 60
   STriggerRecord _InputTriggerQueue[TRIGGER_QUEUE_LENGTH];
   STriggerRecord _OutputTriggerQueue[TRIGGER_QUEUE_LENGTH];

   volatile uint8_t _iOutputQueueReadIndex = 0;
   volatile uint8_t _iInputQueueReadIndex = 0;
   volatile uint8_t _iOutputQueueWriteIndex = 0;
   volatile uint8_t _iInputQueueWriteIndex = 0;

   bool _bFault;
   bool _bDogFaults[4];
   bool _bDogManualFaults[4];
   bool _bDogPerfectCross[4][4];
   bool _bDogBigOK[4][4];
   bool _bDogMissedGateGoingin[4][4];
   bool _bDogMissedGateComingback[4][4];
   bool _bRerunBusy;
   bool _bS1isSafe;
   bool _bS1StillSafe;
   bool _bNegativeCrossDetected;
   bool _bPotentialNegativeCrossDetected;
   unsigned long long _llLastDogTimeReturnTimeStamp[4];
   uint8_t _iLastReturnedRunNumber[4];
   unsigned long long _llDogEnterTimes[4];
   unsigned long long _llDogExitTimes[4];
   unsigned long _lDogTimes[4][4];
   long _lCrossingTimes[4][4];

   String _strTransition;
   
   enum _byDogStates {
      GOINGIN,
      COMINGBACK
   };
   _byDogStates _byDogState;
   bool _bGatesClear = true;

   stRaceData _HistoricRaceData[NUM_HISTORIC_RACE_RECORDS];
   uint _iCurrentRaceId;

   void _ChangeRaceState(RaceStates _byNewRaceState);
   void _ChangeDogState(_byDogStates _byNewDogState);
   void _ChangeDogNumber(uint8_t _iNewDogNumber);
   void _QueuePush(STriggerRecord _InterruptTrigger);
   void _QueueFilter();
   STriggerRecord _QueuePop();
   bool _QueueEmpty();
   void _AddToTransitionString(STriggerRecord _InterruptTrigger);
   void _HandleScheduledRace();
};

extern RaceHandlerClass RaceHandler;

#endif
