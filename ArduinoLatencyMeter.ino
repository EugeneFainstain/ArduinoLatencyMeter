/*
Copyright (C) 2023  Eugene Fainstain https://github.com/EugeneFainstain
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License at http://www.gnu.org/licenses .
AUTHOR: Eugene Fainstain
WEBSITE: https://github.com/EugeneFainstain
HISTORY:
1.0 Sept-17-2023 ArduinoLatencyMeter for Arduino Pro Micro
*/

#define VERSION_STRING "v1.0"

//#define PHOTOTRANSISTOR_TEST_MODE
//#define REVERSE_PHOTOTRANSISTOR
//#define SLOW_ANALOG_READ
//#define PRINT_ESTIMATED_LAG_FRAMES

const bool g_bPrint = true;
const bool g_bBlink = true;

#if !defined(SLOW_ANALOG_READ)
#include "avdweb_AnalogReadFast.h"
#endif
#include <Mouse.h>

#ifdef REVERSE_PHOTOTRANSISTOR
const int16_t GND_PIN        =  A6;
const int16_t DATA_PIN       =  A7;
#else
const int16_t GND_PIN        =  A7;
const int16_t DATA_PIN       =  A6;
#endif
const int16_t INIT_LUM_RANGE = 300;
const int16_t MIN_LUM_RANGE  =  16; // Anything less than that makes no sense.
const int16_t TOUCH_THRESHOLD=1006; // Anything above this means we shorted the leads.

const int16_t  MAX_RANDOM_TIME_MS = 60; // Randomize start-of-test time to avoid being locked into a stable cycle...
const int16_t  AVERAGE_NUM        = 16; // How many samples to average together.
const int16_t  LUM_THRESH_FRAC    = 4;  // What fraction of the full range does the luminance need to change to be registered.
const int16_t  WINDOW_SIZE        = 16; // Measurements per line
const int16_t  FLICKER_WINDOW_MS  = 30; // 30ms
const signed char MOUSE_MOVE      = 40; // mouse step 0..127 - don't use "128"!

float RESULTS_WINDOW[WINDOW_SIZE] = {};
int16_t     g_iLastWindowIdx      = 0;
int16_t     g_iLUM_RANGE          = MIN_LUM_RANGE;
signed char g_scMouseMove         = 0;
bool        g_bTouchDetected      = false;

#ifdef SLOW_ANALOG_READ
  inline int16_t ReadLum() { return 1023 - analogRead(DATA_PIN); }
#else
  inline int16_t ReadLum() { return 1023 - analogReadFast(DATA_PIN); }
#endif

inline int16_t  max_i16  (int16_t   iVal1, int16_t   iVal2) { return  iVal1 >  iVal2 ?  iVal1 :  iVal2; }
inline uint32_t max_ui32 (uint32_t uiVal1, uint32_t uiVal2) { return uiVal1 > uiVal2 ? uiVal1 : uiVal2; }
inline float    max_float(float     fVal1, float     fVal2) { return  fVal1 >  fVal2 ?  fVal1 :  fVal2; }

#define analogRead     ERROR // Just in case
#define analogReadFast ERROR // Just in case
#undef  max
#define max            ERROR // Just in case
#undef  min
#define min            ERROR // Just in case

void setup()
{
    // Init com print
    Serial.begin(115200);

    // Wait until Serial is ready
    while (!Serial) delay(1);
   
    // Start mouse HID
    Mouse.begin();
    
    // Disable TX LED
    pinMode(30, INPUT); 

    // Set "Ground"
    pinMode(GND_PIN, OUTPUT); digitalWrite(GND_PIN, LOW);
    
    // Set up the ADC readout pin.
    pinMode(DATA_PIN, INPUT_PULLUP);
}

uint32_t ONE_SECOND_OF_COMPARE_BREAK_LOOP = 0; // This will be used for the actual measurement
uint32_t ONE_SECOND_OF_TOUCH_MAX_LOOP     = 0;

void BenchmarkMeasurementLoops()
{
#ifdef SLOW_ANALOG_READ
   const uint32_t measureCycles = 2000;
#else
   const uint32_t measureCycles = 12000; // Fast analog read is about 6x faster...
#endif   

   delay(1); // This is needed for the timers (used in "micros") to properly wake up...

   //
   // Case 1: Compare-break loop (main measurement)
   //
   {
       int16_t  threshLum = 1024;
       uint32_t timeStartedUS = micros(); 

       for(uint32_t count = measureCycles; count > 0; count--)
          if( ReadLum() > threshLum ) // Do NOT use ReadLumAndCheckForTouch() here!...
              break; // never breaks
          
       uint32_t timeFinishedUS = micros();

       ONE_SECOND_OF_COMPARE_BREAK_LOOP = (uint32_t)( (double)measureCycles * 1000000.0 / (timeFinishedUS - timeStartedUS) );
   }

   //
   // Case 2: Max + Touch detection loop
   //
   {
       int16_t  iMaxVal = 0;
       uint32_t measureCyclesWithTouch = measureCycles; // ReadLumAndCheckForTouch() is about as fast as ReadLum()
       uint32_t timeStartedUS = micros(); 
       
       for(uint32_t count = measureCyclesWithTouch; count > 0; count--)
          iMaxVal = max_i16(iMaxVal, ReadLumAndCheckForTouch());

       uint32_t timeFinishedUS = micros();

       ONE_SECOND_OF_TOUCH_MAX_LOOP = (uint32_t)( (double)measureCyclesWithTouch * 1000000.0 / (timeFinishedUS - timeStartedUS) );
   }
//Serial.println();
//Serial.print(ONE_SECOND_OF_COMPARE_BREAK_LOOP); Serial.print(" , "); Serial.print(" , "); Serial.println(ONE_SECOND_OF_TOUCH_MAX_LOOP);
//Serial.println();
}

uint32_t WaitForBlack(int16_t* pRetWhiteVal, const int16_t iLumThresh)
{
   ///////////////////////////////////////////////////////
   // At this point we assume that the square is WHITE. //
   ///////////////////////////////////////////////////////
   uint32_t timeStartedWaitingForBlackUS = micros();

   //
   // Calculate the maximal white value (in case of a flickering display) - this also includes the LCD settle time.
   //
   int16_t whiteVal = 0;
   for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * FLICKER_WINDOW_MS / 1000; count > 0; count--)
      whiteVal = max_i16(whiteVal, ReadLumAndCheckForTouch());
      
   //
   // Send USB HID command
   //
   Mouse.move(-g_scMouseMove, 0, 0); // Move back (from bright to dark)

   //
   // Make several attempts (each one FLICKER_WINDOW_MS) looking for the maximal value. This will timeout within 1.0 seconds
   //
   const uint16_t numTries = 1000 / FLICKER_WINDOW_MS;
   uint16_t attemptIndex;
   for(attemptIndex = numTries; attemptIndex > 0; attemptIndex-- )
   {
      int16_t localMax = 0;

      // Measure the maximum within 1 flicker window
      for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * FLICKER_WINDOW_MS / 1000; count > 0; count--)
         localMax = max_i16(localMax, ReadLumAndCheckForTouch());
      
      whiteVal = max_i16(whiteVal, localMax); // Update the whiteVal while we're at it...

      const int16_t threshLum = whiteVal - iLumThresh; // This works even if whiteVal is saturated...
      
      if( localMax < threshLum )
         break; // Success! This is indeed considered black.
   }      

   *pRetWhiteVal = whiteVal; // Update return value

   return (attemptIndex == 0) ? 0 : (micros() - timeStartedWaitingForBlackUS); // If we reached 0 - there is no black... :(
}

uint32_t WaitForWhite(int16_t* pRetBlackVal, const int16_t iLumThresh)
{
   ///////////////////////////////////////////////////////
   // At this point we assume that the square is BLACK. //
   ///////////////////////////////////////////////////////

   //
   // Wait a randomized amount of time to spread the samples evenly throughout the frame and to avoid phase-locking.
   //
   //delay(random(MAX_RANDOM_TIME_MS)+10);
   int16_t iMillisecondsToWait = random(MAX_RANDOM_TIME_MS) + 10;
   for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * iMillisecondsToWait / 1000; count > 0; count--)
      ReadLumAndCheckForTouch();
   
   //
   // Measure the value of black
   //
   int16_t blackVal = 0; // starting with lowest
   for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * FLICKER_WINDOW_MS / 1000; count > 0; count--)
      blackVal = max_i16(blackVal, ReadLumAndCheckForTouch());

   // Move forward (from dark to bright)
   Mouse.move(g_scMouseMove, 0, 0);

   // Wait for the brightness to increase, while calculating black at the same time... This will timeout if change is not detected within 1.0 seconds.
   const int16_t threshLum = blackVal + iLumThresh;
   uint32_t startTimeUS, endTimeUS;

   //
   // The main time measurement loop:
   //
   {
       // Write down start time
       startTimeUS = micros(); // has resolution of 4 microseconds
    
       // Light up the LED
       if(g_bBlink) RXLED0; // This lights up the LED after micros()

       // First wait for 200ms
       uint32_t count = ONE_SECOND_OF_COMPARE_BREAK_LOOP / 5; // Tight loop for 200ms
       for(count = count; count > 0; count--)
          if( ReadLum() > threshLum ) // Do NOT use ReadLumAndCheckForTouch() here!...
              break;

       // Second - wait the remaining 800ms
       if(count == 0) // this means we already waited for 200ms with no result, which probably means we are going to timeout.
       {              // So now we wait and check touch at the same time, with a slightly lower (almost the same) temporal resolution.
           count = ONE_SECOND_OF_TOUCH_MAX_LOOP * 4 / 5; // Loop for 800ms
           for(count = count; count > 0; count--)
              if( ReadLumAndCheckForTouch() > threshLum )
                  break;
       }
    
       // Write down end time
       endTimeUS = micros();

       // Turn off the LED after micros() - for symmetry - although it is so fast that it makes no difference...
       if(g_bBlink) RXLED1; // This turns off the LED
   }

   *pRetBlackVal = blackVal; // Update return value

   // Return the time delta
   return endTimeUS - startTimeUS;
}

float GetWindowAverage(int16_t iCount)
{
   int16_t idx = g_iLastWindowIdx;
   
   float fWindowSum = 0;
   for(int16_t i = 0; i < iCount; i++)
   {
      fWindowSum += RESULTS_WINDOW[ idx ];
      idx = (idx + WINDOW_SIZE - 1) % WINDOW_SIZE;
   }

   return fWindowSum / (float)iCount;
}

enum eState
{
    STATE_WELCOME_NOTE = 0,
    STATE_IDLE            ,
    STATE_WAITING_TO_START,
    STATE_CALIBRATING     ,
    STATE_MEASURING       ,
};

eState  g_eState = STATE_WELCOME_NOTE;
int16_t g_iWaitToStartCounter = 0;
bool    g_bResetStatisticsOnFirstRun = true;

bool ParseSerialCommand() // Returns true if state changed
{
    if( g_eState == STATE_WELCOME_NOTE ) return true;
    
    eState eStateBefore = g_eState;

    if( Serial.available() || g_bTouchDetected )
    {
        int16_t val;

        // Emulating serial read...
        if( g_bTouchDetected )
        {
            if (g_eState == STATE_MEASURING)
               val = 0; // Stop measurement...
            else
               val = -1; // Do not wait to start...
        }
        else
          val = Serial.parseInt();

        if( val == 0 )
            g_eState = STATE_WELCOME_NOTE;
        else
        {
            g_eState = STATE_WAITING_TO_START;
            g_iWaitToStartCounter = val;
        }

        while( Serial.available() )
            Serial.read(); // Empty the serial buffer
    }

    if (g_eState != eStateBefore)
    {
        g_bTouchDetected = false;
        return true;      
    }
    return false;
}

bool WelcomeNote()
{
    static bool bOneTimeWelcomeNoteAndBenchmark = true;

    if( g_eState == STATE_WELCOME_NOTE )
    {
        if( g_bPrint && bOneTimeWelcomeNoteAndBenchmark )
        {
            delay(1000); // delay a bit to make sure the welcome message is printed.
            for(int i=0; i<16; i++) Serial.println(); // i.e. "clear screen"
            Serial.println("Benchmarking the CPU......");
        }
        
        // Benchmark the CPU - only once per boot.
        if( bOneTimeWelcomeNoteAndBenchmark )
            BenchmarkMeasurementLoops();

        if( g_bPrint && bOneTimeWelcomeNoteAndBenchmark )
        {
            float fTimeResolutionMS = 1000.f / ONE_SECOND_OF_COMPARE_BREAK_LOOP;
        
            Serial.println();
            Serial.print("Firmware version "); Serial.print(VERSION_STRING);
            Serial.println();
            Serial.print("Temporal resolution is approximately "); Serial.print(fTimeResolutionMS,6); Serial.println(" milliseconds.");
            Serial.print("Equivalent camera framerate is about "); Serial.print(ONE_SECOND_OF_COMPARE_BREAK_LOOP/100*100); Serial.println(" frames per second.");
        }

        bOneTimeWelcomeNoteAndBenchmark = false;

        if( g_bPrint )
        {
            Serial.println();
            Serial.println();
            Serial.println("Ready! Please position the sensor next to a contrast vertical edge.");
            Serial.println("Enter the number of seconds to wait (0 to stop), or press the wire to start/stop the measurement.");
            Serial.println();
        }

        g_bTouchDetected = false;
        g_eState         = STATE_IDLE;
        return true;
    }
    return false;  
}

void TouchDelay(int16_t iMillisecondsToWait)
{
    for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * iMillisecondsToWait / 1000; count > 0; count--)
    {
        ReadLumAndCheckForTouch();
        if( g_bTouchDetected ) // This doesn't have to be a precise wait...
            break;
    }
}

bool InIdleMode()
{
    if( g_eState == STATE_IDLE )
    {
        for(;;)
        {
            TouchDelay(100); //delay(100); // Just wait a bit...
           
            if( ParseSerialCommand() )
               break;
        }
    }
    return false;  
}

bool WaitToStart()
{
    if( g_eState == STATE_WAITING_TO_START )
    {
        if( g_iWaitToStartCounter > 0 )
        {
            Serial.println();
            Serial.print("Waiting : ");
        }
        
        while( g_iWaitToStartCounter > 0 )
        {
            Serial.print(" ");
            Serial.print(g_iWaitToStartCounter);
            TouchDelay(1000); //delay(1000); // Wait about 1 second...
            g_iWaitToStartCounter--;
            if( ParseSerialCommand() ) return true;
        }
        Serial.println();
        g_bResetStatisticsOnFirstRun = true;
        g_eState = STATE_CALIBRATING;
        return true;
    }
    return false;
}

int16_t Calibrate_Measure()
{
    delay(500);
    
    int16_t val = 0;
    uint32_t measureTimeMS = 125; // spend 1/8 of a second for each measurement
    for(uint32_t count = ONE_SECOND_OF_TOUCH_MAX_LOOP * measureTimeMS / 1000; count > 0; count--) // 125ms
       val = max_i16( val, ReadLumAndCheckForTouch() );
       
    return val;
}

bool Calibrate()
{
    if( g_eState == STATE_CALIBRATING )
    {
        Serial.print("Calibrating... ");

        const int16_t SEARCH_RANGE = 1;//2; // -2..+2
        const int16_t SEARCH_TOTAL = SEARCH_RANGE*2 + 1; // 0..5
        const int16_t SEARCH_PAIRS = SEARCH_TOTAL   - 1; // 0..4
        int16_t iCalibrateValues[SEARCH_TOTAL] = {}; // Looking around -2..+2 steps for a good edge

        // Move to the left of the range and wait a bit
        for(int16_t i=0; i<SEARCH_RANGE; i++)
        {
            Mouse.move(-MOUSE_MOVE, 0, 0);
            delay(100);
        }

        // Measure the point16_ts and move the mouse right (except after the last measurement)
        for(int16_t i=0; i<SEARCH_TOTAL; i++)
        {
            iCalibrateValues[i] = Calibrate_Measure();
            if( i < SEARCH_PAIRS ) Mouse.move(MOUSE_MOVE, 0, 0);
        }          

        // Find the most contrast pair
        int16_t iPairIdx = -1; int16_t iMaxContrast = -1;
        for(int16_t i=0; i<SEARCH_PAIRS; i++)
        {
           int16_t iContrast = abs(iCalibrateValues[i] - iCalibrateValues[i+1]);
           if( iContrast > iMaxContrast )
           {
              iMaxContrast = iContrast;
              iPairIdx = i;
           }
        }

        // Update the initial contrast and number of left steps - assuming the bright patch is on the left.
        int16_t iLeftSteps = SEARCH_PAIRS - iPairIdx;
        g_scMouseMove  = -MOUSE_MOVE; // A bit counter-int16_tuitive, yes...
        g_iLUM_RANGE   = iMaxContrast;

        // Update for the case when the bright patch is on the right
        int16_t iLeftVal  = iCalibrateValues[iPairIdx    ];
        int16_t iRightVal = iCalibrateValues[iPairIdx + 1];
        if( iLeftVal < iRightVal )
        {
            g_scMouseMove = -g_scMouseMove;
            iLeftSteps--;
        }
        
        // Move left and wait a bit (if needed)
        if( iLeftSteps )
        {
            for(int16_t i=0; i<iLeftSteps; i++)
            {
                Mouse.move(-MOUSE_MOVE, 0, 0);
                delay(500);
            }
        }

        // Make sure we have enough contrast
        if( iMaxContrast < 20 )
        {
           Serial.print("Insufficient contrast - "); Serial.print(iMaxContrast); Serial.println(" - please reposition the view. Aborting...");
           g_eState = STATE_WELCOME_NOTE;
           return true;
        }

        Serial.print("Success! Mouse step is "); Serial.print(g_scMouseMove);
        Serial.print(", initial contrast is "); Serial.print(g_iLUM_RANGE);
        Serial.println(".");

        // Update state
        g_eState = STATE_MEASURING;
        return true;
    }
    return false;
}

inline int16_t ReadLumAndCheckForTouch()
{
  const int16_t iLum = ReadLum();

  g_bTouchDetected |= (iLum >= TOUCH_THRESHOLD);

  return iLum;
}

void loop()
{
#ifdef PHOTOTRANSISTOR_TEST_MODE
    Serial.println(ReadLum());
    delay(10);
    return;
#endif

    if( WelcomeNote() ) return;
    if( InIdleMode()  ) return;
    if( WaitToStart() ) return;
    if( Calibrate()   ) return;

    static double   dAccTimeSumMS   = 0;
    static double   dAccTimeSqSumMS = 0;
    static uint32_t uiAccSamples    = 0;

    if(g_bResetStatisticsOnFirstRun)
    {
        for(int i = 0; i < WINDOW_SIZE; i++) RESULTS_WINDOW[i] = 0;
        g_iLastWindowIdx = 0; // not necessary
        dAccTimeSumMS    = 0;
        dAccTimeSqSumMS  = 0;
        uiAccSamples     = 0;
    }

    if(g_bPrint)
    {
        Serial.print("av8 = ");
        Serial.print(GetWindowAverage(8),1);
        Serial.print(",\t");
        ////////////////////////////////
        Serial.print("av4 = ");
        Serial.print(GetWindowAverage(4),1);
        Serial.print(",\t");
        ////////////////////////////////
        Serial.print("av2 = ");
        Serial.print(GetWindowAverage(2),1);
        Serial.print(",\t");
        ////////////////////////////////
        Serial.print("av1 = ");
        Serial.print(GetWindowAverage(1),1); // == RESULTS_WINDOW[g_iLastWindowIdx] == "prev" fAverageMS
        Serial.print(",\t");
        ////////////////////////////////
        Serial.print("imm: ");
    }    

    const int iLumThresh = max_i16(g_iLUM_RANGE,MIN_LUM_RANGE) / LUM_THRESH_FRAC; // How much does the luminance need to change to be registered.

    float fTimeSumMS   = 0;
    float fTimeSqSumMS = 0;
    int   sumWhite     = 0;
    int   sumBlack     = 0;

    for( int i = 0; i < AVERAGE_NUM; i++ )
    {
        if( ParseSerialCommand() ) return;
        
        int whiteVal, blackVal;

        float fTimeWaitingForBlackMS = WaitForBlack(&whiteVal, iLumThresh) / 1000.0f;

        bool bSuccess = (fTimeWaitingForBlackMS > 0) ? true : false;
        
        if( bSuccess == false ) if(g_bPrint) Serial.print('#');
    
        float fTimeMS   = WaitForWhite(&blackVal, iLumThresh) / 1000.0f;
        float fTimeSqMS = fTimeMS * fTimeMS;

        if(g_bPrint) { Serial.print(fTimeMS,0); Serial.print(", "); }
    
        fTimeSumMS   += fTimeMS;
        fTimeSqSumMS += fTimeSqMS;

        sumWhite += whiteVal;
        sumBlack += blackVal;

        float ftempAccAverageMS = (float)(dAccTimeSumMS / max_ui32(1,uiAccSamples));

        // This check is intended to safeguard the accumulated reading against invalid values (and far outliers)
        if( bSuccess && (fTimeMS < 500) && (fTimeMS > ftempAccAverageMS / 4) )
        {
            dAccTimeSumMS   += fTimeMS;
            dAccTimeSqSumMS += fTimeSqMS;
            uiAccSamples    += 1;
        }
    }

    int iMeasuredRange = (sumWhite - sumBlack) / AVERAGE_NUM;
    g_iLUM_RANGE = max_i16( MIN_LUM_RANGE, iMeasuredRange );
  
    float fAverageMS = fTimeSumMS / AVERAGE_NUM;

    if(g_bResetStatisticsOnFirstRun)
    {
        g_bResetStatisticsOnFirstRun = false;
        for(int i = 0; i < WINDOW_SIZE; i++) RESULTS_WINDOW[i] = fAverageMS;
    }
    else
    {
        g_iLastWindowIdx = (g_iLastWindowIdx + 1) % WINDOW_SIZE;
        RESULTS_WINDOW[g_iLastWindowIdx] = fAverageMS;
    }

    // Standard variation and estimated framerate - based on the last AVERAGE_NUM samples
    float fStdMS, fEstFPS;
    {
        float fAverageSqMS = fTimeSqSumMS / AVERAGE_NUM;
        float fVarMS2      = max_float(0, fAverageSqMS - fAverageMS * fAverageMS);
              fStdMS       = sqrtf(fVarMS2);
        float fFrameTimeMS = fStdMS * sqrtf(12);
              fEstFPS      = 1000.0f / fmax(1, fFrameTimeMS);
    }

    // Standard variation and estimated framerate - cumulatively based on all data in the current experiment
    // Latency in [frames] (fAccLatFrames) is based on estimated framerate and cumulative average latency in [ms]
    float fAccStdMS, fAccEstFPS, fAccAverageMS, fAccLatFrames;
    {
        float  fAccAverageSqMS = (float)(dAccTimeSqSumMS / max_ui32(1,uiAccSamples));
               fAccAverageMS   = (float)(dAccTimeSumMS   / max_ui32(1,uiAccSamples));
        float  fAccVarMS2      = max_float(0, fAccAverageSqMS - fAccAverageMS * fAccAverageMS);
               fAccStdMS       = sqrtf(fAccVarMS2);
        float  fAccFrameTimeMS = fAccStdMS * sqrtf(12);
               fAccEstFPS      = 1000.0f / fmax(1, fAccFrameTimeMS);

               fAccLatFrames   = fAccAverageMS / max_float(1.0f, fAccFrameTimeMS);
               fAccLatFrames   = fAccLatFrames - 0.5f; // Compensate for the stochastic nature of [ms] latency measurement
    }

    if(g_bPrint)
    {
        ////////////////////////////////
        //Serial.print("\tstd1 = ");
        //Serial.print(fStdMS,1);
        //Serial.print(",");
        ////////////////////////////////
        Serial.print("\tlag = ");
        Serial.print(fAccAverageMS,1);
        Serial.print("ms,");
        ////////////////////////////////
        Serial.print("\tsamples = ");
        Serial.print(uiAccSamples);
        Serial.print(",");
        ////////////////////////////////
        Serial.print("\t~fps = ");
        Serial.print(fAccEstFPS,1);
        Serial.print(",");
        ////////////////////////////////
#ifdef PRINT_ESTIMATED_LAG_FRAMES
        Serial.print("\t~lag[frames] = ");
        Serial.print(fAccLatFrames,2);
        Serial.print(",");
#endif
        ////////////////////////////////
        Serial.print("\tB/"); Serial.print(sumBlack / AVERAGE_NUM);
        Serial.print(", W/"); Serial.print(sumWhite / AVERAGE_NUM);
        Serial.print(", R/"); Serial.print(iMeasuredRange);
        Serial.print(", " );  Serial.print(VERSION_STRING);
        Serial.println();
    }
}
