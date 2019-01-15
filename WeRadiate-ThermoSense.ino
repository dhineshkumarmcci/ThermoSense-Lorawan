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
#include <Catena_TxBuffer.h>
#include <Adafruit_BME280.h>
#include <Arduino_LoRaWAN.h>
#include <Catena_Si1133.h>
#include <lmic.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>
#include <SPI.h>

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
        //CATCFG_T_CYCLE = 6 * 60,        // every 6 minutes
        CATCFG_T_CYCLE = 480 * 60,        // 3 times a day(every 480 minutes)
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

// forwards
static bool checkCompostSensorPresent(void);
void settleDoneCb(osjob_t *pSendJob);
void warmupDoneCb(osjob_t *pSendJob);
void txFailedDoneCb(osjob_t *pSendJob);
void sleepDoneCb(osjob_t *pSendJob);
Arduino_LoRaWAN::SendBufferCbFn sendBufferDoneCb;
    

/****************************************************************************\
|
|       Read-only data.
|
\****************************************************************************/

const char sVersion[] = "1.0.0";


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

//
// the LoRaWAN backhaul.  Note that we use the
// Catena version so it can provide hardware-specific
// information to the base class.
//
Catena::LoRaWAN gLoRaWAN;

// The submersible temperature sensor
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensor_CompostTemp(&oneWire);
bool fHasCompostTemp;

// The temperature/humidity sensor
Adafruit_BME280 gBme; // The default initalizer creates an I2C connection
bool fBme;

// The LUX sensor
Catena_Si1133 gSi1133;
bool fLux;

// USB power
bool fUsbPower;

// the job that's used to synchronize us with the LMIC code
static osjob_t sensorJob;
void sensorJob_cb(osjob_t *pJob);

// The contact sensors
bool fHasPower1;

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
        setup_built_in_sensors();
        setup_external_temp_sensor();
        
        gCatena.SafePrintf("End of setup\n");

        /* for stm32 core, we need wider tolerances, it seems */
        LMIC_setClockError(10 * 65536 / 100);
        
        /* trigger a join by sending the first packet */
        startSendingUplink();
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

        fHasPower1 = false;

        if (modnumber != 0)
                {
                gCatena.SafePrintf("Catena 4612-M%u\n", modnumber);
                if (modnumber == 102 || modnumber == 103 || modnumber == 104)
                        {
                        fHasCompostTemp = flags & CatenaBase::fHasWaterOneWire;
                        /* set D11 high so V_OUT2 is going to be high for onewire sensor */        
                        pinMode(D11, OUTPUT);
                        digitalWrite(D11, HIGH);
                        }
                else
                        {
                        gCatena.SafePrintf("unknown mod number %d\n", modnumber);
                        }
                }
        else
                {
                gCatena.SafePrintf("No mods detected\n");
                fHasCompostTemp = flags & CatenaBase::fHasWaterOneWire;
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

void setup_external_temp_sensor(void)
        {
        /* set D11 high so V_OUT2 is going to be high for onewire sensor */        
        pinMode(D11, OUTPUT);
        digitalWrite(D11, HIGH);
        
        bool fCompostTemp = checkCompostSensorPresent();
        
        if(!fCompostTemp)
                {
                gCatena.SafePrintf("No one-wire temperature sensor detected\n");
                }
        else
                {
                gCatena.SafePrintf("One-wire temperature sensor detected\n");
                }
        }

 static bool checkCompostSensorPresent(void)
        {
        sensor_CompostTemp.begin();
        return sensor_CompostTemp.getDeviceCount() != 0;
        }

void setup_built_in_sensors(void)
        {
        uint32_t flags;
        flags = gCatena.GetPlatformFlags();
  
        /* initialize the lux sensor */
        if (flags & CatenaStm32::fHasLuxSi1113)
                {
                if (gSi1133.begin())
                        {
                        fLux = true;
                        gSi1133.configure(0, CATENA_SI1133_MODE_SmallIR);
                        gSi1133.configure(1, CATENA_SI1133_MODE_White);
                        gSi1133.configure(2, CATENA_SI1133_MODE_UV);
                        gSi1133.start();
                        }
                else
                        {
                        fLux = false;
                        gCatena.SafePrintf("No Si1133 found: check wiring\n");
                        }
                }
        else
                {
                gCatena.SafePrintf("No Si1133 wiring\n");
                fLux = false;
                }
    
        /* initialize the BME280 */
        if (flags & CatenaStm32::fHasBme280)
                {
                if (gBme.begin(BME280_ADDRESS, Adafruit_BME280::OPERATING_MODE::Sleep))
                        {
                        fBme = true;
                        gCatena.SafePrintf("BME280 found\n");
                        }
                else
                        {
                        fBme = false;
                        gCatena.SafePrintf("No BME280 found: check wiring\n");
                        }
                }
        else
                {
                fBme = false;
                gCatena.SafePrintf("No BME280 found: check wiring. Just nothing. \n");
                }

        if (fLux)
                {
                delay(510); /* default measure interval is 500ms */
                }
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
        }

void startSendingUplink(void)
        {
        TxBuffer_t b;
        LedPattern savedLed = gLed.Set(LedPattern::Measuring);

        b.begin();
        FlagsSensor2 flag;

        flag = FlagsSensor2(0);

        b.put(FormatSensor2); /* the flag for this record format */
        uint8_t * const pFlag = b.getp();
        b.put(0x00); /* will be set to the flags */

        // vBat is sent as 5000 * v
        float vBat = gCatena.ReadVbat();
        gCatena.SafePrintf("vBat:    %d mV\n", (int) (vBat * 1000.0f));
        b.putV(vBat);
        flag |= FlagsSensor2::FlagVbat;
      
        // vBus is sent as 5000 * v
        float vBus = gCatena.ReadVbus();
        gCatena.SafePrintf("vBus:    %d mV\n", (int) (vBus * 1000.0f));
        fUsbPower = (vBus > 3.0) ? true : false;

        uint32_t bootCount;
        if (gCatena.getBootCount(bootCount))
                {
                b.putBootCountLsb(bootCount);
                flag |= FlagsSensor2::FlagBoot;
                }

        if (fBme)
                {
                Adafruit_BME280::Measurements m = gBme.readTemperaturePressureHumidity();
                // temperature is 2 bytes from -0x80.00 to +0x7F.FF degrees C
                // pressure is 2 bytes, hPa * 10.
                // humidity is one byte, where 0 == 0/256 and 0xFF == 255/256.
                gCatena.SafePrintf(
                        "BME280:  T: %d P: %d RH: %d\n",
                        (int) m.Temperature,
                        (int) m.Pressure,
                        (int) m.Humidity
                        );
                b.putT(m.Temperature);
                b.putP(m.Pressure);
                b.putRH(m.Humidity);
    
                flag |= FlagsSensor2::FlagTPH;
                }

        if (fLux)
                {
                /* Get a new sensor event */
                uint16_t data[3];
    
                gSi1133.readMultiChannelData(data, 3);
                gSi1133.stop();
                gCatena.SafePrintf(
                        "Si1133:  %u IR, %u White, %u UV\n",
                        data[0],
                        data[1],
                        data[2]
                        );
                b.putLux(data[1]);
                flag |= FlagsSensor2::FlagLux;
                }
    
        /*
        || Measure and transmit the "water temperature" (OneWire)
        || tranducer value. This is complicated because we want
        || to support plug/unplug and the sw interface is not
        || really hot-pluggable.
        */
        
        /* set D11 high so V_OUT2 is going to be high for onewire sensor */        
        pinMode(D11, OUTPUT);
        digitalWrite(D11, HIGH);
        
        bool fCompostTemp = checkCompostSensorPresent();
      
        if (fCompostTemp)
                {
                sensor_CompostTemp.requestTemperatures();
                float compostTempC = sensor_CompostTemp.getTempCByIndex(0);
                Serial.print("Compost temperature: "); Serial.print(compostTempC); Serial.println(" C");
                }
        else if (fHasCompostTemp)
                {
                gCatena.SafePrintf("No compost temperature\n");
                }
             
        /* set D11 low to turn off after measuring */        
//        digitalWrite(D11, LOW);
        pinMode(D11, INPUT);
      
        *pFlag = uint8_t(flag);
        if (savedLed != LedPattern::Joining)
                {
                gLed.Set(LedPattern::Sending);
                }
        else
                {
                gLed.Set(LedPattern::Joining);
                }
        
        gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, NULL);
}

void
sendBufferDoneCb(
        void *pContext,
        bool fStatus
        )
        {
        osjobcb_t pFn;

        gLed.Set(LedPattern::Settling);
        if (! fStatus)
                {
                gCatena.SafePrintf("send buffer failed\n");
                pFn = txFailedDoneCb;
                }
        else
                {
                pFn = settleDoneCb;
                }
        os_setTimedCallback(
                &sensorJob,
                os_getTime()+sec2osticks(CATCFG_T_SETTLE),
                pFn
                );
        }

void
txFailedDoneCb(
        osjob_t *pSendJob
        )
        {
        gCatena.SafePrintf("not provisioned, idling\n");
        gLoRaWAN.Shutdown();
        gLed.Set(LedPattern::NotProvisioned);
        }

void settleDoneCb(
        osjob_t *pSendJob
        )
        {
        // if connected to USB, don't sleep
        // ditto if we're monitoring pulses.
	// XXX: do we need this?
#ifdef USBCON
        if (fUsbPower || fHasPower1)
#else
        if (fHasPower1)
#endif
        {
        gLed.Set(LedPattern::Sleeping);
        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_INTERVAL),
                sleepDoneCb
                );
        return;
        }

        /* ok... now it's time for a deep sleep */
        gLed.Set(LedPattern::Off);
        Serial.end();
        Wire.end();
        SPI.end();

        gCatena.Sleep(CATCFG_T_INTERVAL);

        /* and now... we're awake again. trigger another measurement */
        Serial.begin();
        Wire.begin();
        SPI.begin();
        
        sleepDoneCb(pSendJob);
        }

void sleepDoneCb(
        osjob_t *pJob
        )
        {
        gLed.Set(LedPattern::WarmingUp);
        gSi1133.start();

        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_WARMUP),
                warmupDoneCb
                );
        }

void warmupDoneCb(
        osjob_t *pJob
        )
        {
        startSendingUplink();
        }
