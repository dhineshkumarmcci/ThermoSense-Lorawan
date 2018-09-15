/*

Module:  WeRadiate-ThermoSense.ino

Function:
        Code for the WeRadiate ThermoSense sensor based on Catena 4612

Copyright notice:
        See LICENSE file accompanying this project

Author:
        Ezra Undag, WeRadiate	September 2018
        Terry Moore, MCCI Corporation	September 2018

*/

#include <Catena.h>
#include <Catena_Led.h>
#include <CatenaStm32L0Rtc.h>

#include <Arduino_LoRaWAN.h>
#include <lmic.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/****************************************************************************\
|
|       Manifest constants & typedefs.
|
\****************************************************************************/

using namespace McciCatena;

/* how long do we wait between transmissions? (in seconds) */
enum    {
        // set this to interval between transmissions, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time, but we attempt
        // to compensate for the gross effects below.
        CATCFG_T_CYCLE = 6 * 60,        // every 6 minutes
        CATCFG_T_CYCLE_TEST = 30,       // every 10 seconds
        };

/* additional timing parameters; ususually you don't change these. */
enum    {
        CATCFG_T_WARMUP = 1,
        CATCFG_T_SETTLE = 5,
        CATCFG_T_OVERHEAD = (CATCFG_T_WARMUP + CATCFG_T_SETTLE),
        };

constexpr uint32_t CATCFG_GetInterval(uint32_t tCycle)
        {
        return (tCycle < CATCFG_T_OVERHEAD)
                ? CATCFG_T_OVERHEAD
                : tCycle - CATCFG_T_OVERHEAD
                ;
        }

enum    {
        CATCFG_T_INTERVAL = CATCFG_GetInterval(CATCFG_T_CYCLE),
        };

enum    {
        PIN_ONE_WIRE =  A2,        // XSDA1 == A2
        };

/****************************************************************************\
|
|       Read-only data.
|
\****************************************************************************/

//
// if this pin is written high, it enables the boost regulator. If running off batteries, this
// means that Vdd will be 3.3V (rather than Vbat).
// If running off USB power, this has no effect.
//
// Enabling the boost regulator is a good idea in the following situations:
// 
//  1) if you are enabling external Vdd to extenral sensors.
//  2) if you are trying for maximum transmit power from the SX1276.
//
// However, it's good only to do this when you need to. The 4612 doesn't need 3.3V
// when sleeping, and the boost regulator takes non-negligible power.
//
const int gkVddBoostEnable = A0;

//
// if this pin is written high, it enables the TCXO (temperature-compensated crystal
// oscillator). The TCXO must be powered up before trying to do anything serious 
// with the radio. The normal delay after turning on the TCXO is 1ms to 3ms. MCCI is
// changing the LMIC code to handle it, but on 2018-09-14 it still wasn't ready,
// so we do it manually. Once the LMIC is ready, it will manage this pin dynamically
// so if we forget, nothing bad will happen, as long as we just set it up at the
// beginning of time.
//
const int gkTcxoVdd = D33;

const char sVersion[] = "V1.0.0";

/****************************************************************************\
|
|       Variables.
|
\****************************************************************************/

Catena gCatena;

//
// the LED
//
StatusLed gLed (Catena::PIN_STATUS_LED);

// the RTC instance, used for sleeping
CatenaStm32L0Rtc gRtc;

//
// the LoRaWAN backhaul.  Note that we use the
// Catena version so it can provide hardware-specific
// information to the base class.
//
Catena::LoRaWAN gLoRaWAN;

//   The submersible temperature sensor
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensor_WaterTemp(&oneWire);
bool fHasWaterTemp;
bool fFoundWaterTemp;


/*

Name:	setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
        {
        Serial.begin(115200);
        gCatena.begin();

        setup_platform();
        setup_sensors();

        gCatena.SafePrintf("End of setup\n");
        }

/*

Name:   setup_platform()

Function:
        Setup everything related to the Catena framework. (Not app specific.)

Definition:
        void setup_platform(
            void
            );

Description:
        This function only exists to make clear what has to be done for
        the framework (as opposed to the actual application). It can be argued
        that all this should be part of the gCatena.begin() function.

Returns:
        No explicit result.

*/

void setup_platform(void)
        {
        // set up power supplies and TCXO
        pinMode(gkVddBoostEnable, OUTPUT);
        digitalWrite(gkVddBoostEnable, 1);
        pinMode(gkTcxoVdd, OUTPUT);
        digitalWrite(gkTcxoVdd, 1);

#ifdef USBCON
        // if running unattended, don't wait for USB connect.
        if (!(gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
                {
                while (!Serial)
                        /* wait for USB attach */
                        yield();
                }
        gCatena.SafePrintf("USB enabled\n");
#else
        gCatena.SafePrintf("USB disabled\n");
#endif

        gCatena.SafePrintf("\n");
        gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("This is the WeRadiate-ThermoSense program V%s.\n", sVersion);
                {
                char sRegion[16];
                gCatena.SafePrintf("Target network: %s / %s\n",
                                gLoRaWAN.GetNetworkName(),
                                gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                                );
                }
        gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
        gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");
        gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("\n");

        // set up the LED
        gLed.begin();
        gCatena.registerObject(&gLed);
        gLed.Set(LedPattern::FastFlash);

        // set up the RTC object
        gRtc.begin();

        gCatena.SafePrintf("LoRaWAN init: ");
        if (!gLoRaWAN.begin(&gCatena))
                {
                gCatena.SafePrintf("failed\n");
                gCatena.registerObject(&gLoRaWAN);
                }
        else
                {
                gCatena.SafePrintf("succeeded\n");
                gCatena.registerObject(&gLoRaWAN);
                }

        // display the CPU unique ID
        Catena::UniqueID_string_t CpuIDstring;

        gCatena.SafePrintf("CPU Unique ID: %s\n",
                gCatena.GetUniqueIDstring(&CpuIDstring)
                );

        /* find the platform */
        const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

        uint32_t flags;
        const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

        if (pPlatform)
                {
                gCatena.SafePrintf("EUI64: ");
                for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
                        {
                        gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
                        }
                gCatena.SafePrintf("\n");
                flags = gCatena.GetPlatformFlags();
                gCatena.SafePrintf(
                        "Platform Flags:  %#010x\n",
                        flags
                        );
                gCatena.SafePrintf(
                        "Operating Flags:  %#010x\n",
                        gCatena.GetOperatingFlags()
                        );
                }
        else
                {
                gCatena.SafePrintf("**** no platform, check provisioning ****\n");
                flags = 0;
                }

        /* is it modded? */
        uint32_t modnumber = gCatena.PlatformFlags_GetModNumber(flags);

        //fHasPower1 = false;

        if (modnumber != 0)
                {
                gCatena.SafePrintf("Catena 4612-M%u\n", modnumber);
                if (modnumber == 102 || modnumber == 103 || modnumber == 104)
                        {
                        fHasWaterTemp = flags & CatenaBase::fHasWaterOneWire;
                        // fSoilSensor = flags & CatenaBase::fHasSoilProbe;
                        }
                else
                        {
                        gCatena.SafePrintf("unknown mod number %d\n", modnumber);
                        }
                }
        else
                {
                gCatena.SafePrintf("No mods detected\n");
                fHasWaterTemp = flags & CatenaBase::fHasWaterOneWire;
                // fSoilSensor = flags & CatenaBase::fHasSoilProbe;
                }
        }

/*

Name:   setup_sensors()

Function:
        Set up the sensors we intend to use (app specific).

Definition:
        void setup_sensors(
            void
            );

Description:
        This function only exists to make clear what has to be done for
        the actual application. This is the code that cannot be part of
        the generic gCatena.begin() function.

Returns:
        No explicit result.

*/

void setup_sensors(void)
        {

        }

/*

Name:	loop()

Function:
        Arduino loop function.

Definition:
        void loop(
                void
                );

Description:
        This function is called repeatedly by the Arduino framework after
        setup() has been called.

        This version calls gCatena.poll() to drive all the event loops and
        timers. For manufacturing test mode, it continuously reads the sensor values,
        which will produce serial output. 

Returns:
        No explicit result.

*/


void loop(void)
        {
        // put your main code here, to run repeatedly:
        gCatena.poll();
        Serial.println("hello world");
        delay(2000);
        }
