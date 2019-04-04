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
#include <Catena_Mx25v8035f.h>
#include <Adafruit_BME280.h>
#include <Arduino_LoRaWAN.h>
#include <Catena_Si1133.h>
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

/* parameters for controlling the uplink timing */
enum    {
        // set this to interval between transmissions, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time, but we attempt
        // to compensate for the gross effects below.
        CATCFG_T_CYCLE = 5 * 60,                // every 5 minutes
        // uplink cycle time after bootup
        CATCFG_T_CYCLE_INITIAL = 30,            // every 30 seconds initially
        // number of uplinks at initial rate before resetting
        CATCFG_INTERVAL_COUNT_INITIAL = 30,     // repeat for 15 minutes
        };

/* additional timing parameters; ususually you don't change these. */
enum    {
        // the warm-up time, in seconds
        CATCFG_T_WARMUP = 1,
        // the settling time uplink, in seconds
        CATCFG_T_SETTLE = 5,
        // the amount of overhead, total, in seconds.
        CATCFG_T_OVERHEAD = (CATCFG_T_WARMUP + CATCFG_T_SETTLE + 2),
        // the minimum cycle time
        CATCFG_T_MIN = CATCFG_T_OVERHEAD,
        // length of day in seconds
        CATCFG_T_ONE_DAY = 24 * 60 * 60,
        // maximum programmable cycle time
        CATCFG_T_MAX = CATCFG_T_ONE_DAY,     // normally one hour max.
        // default uplink interval, in seconds
        CATCFG_INTERVAL_COUNT_DEFAULT = 30,
        };

// given a cycle time in seconds, how long should we sleep?
constexpr uint32_t CATCFG_GetInterval(uint32_t tCycle)
        {
        return (tCycle < CATCFG_T_OVERHEAD + 1)
                ? 1
                : tCycle - CATCFG_T_OVERHEAD
                ;
        }

enum    {
        // how long to sleep, in seconds.
        CATCFG_T_INTERVAL = CATCFG_GetInterval(CATCFG_T_CYCLE),
        };

enum    {
        PIN_ONE_WIRE =  A2,        // XSDA1 == A2
        };

// forwards
bool checkCompostSensorPresent(void);
void fillBuffer(TxBuffer_t &b);
void prepareToSleep(void);
Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;
void recoverFromSleep(void);
Arduino_LoRaWAN::SendBufferCbFn sendBufferDoneCb;
void settleDoneCb(osjob_t *pSendJob);
void setTxCycleTime(unsigned txCycle, unsigned txCount);
void sleepDoneCb(osjob_t *pSendJob);
void txFailedDoneCb(osjob_t *pSendJob);
void warmupDoneCb(osjob_t *pSendJob);

/****************************************************************************\
|
|       Read-only data.
|
\****************************************************************************/

const char sVersion[] = "1.1.0";

//
// set this to the branch you're using, if this is a branch.
const char sBranch[] = " (5 min)";
// keep by itself, more or less, for easy git rebasing.
//

/****************************************************************************\
|
|       Variables.
|
\****************************************************************************/

// the Catena instance
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

// The external temperature sensor
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensor_CompostTemp(&oneWire);
bool fHasCompostTemp;

// The temperature/humidity sensor
Adafruit_BME280 gBme; // The default initalizer creates an I2C connection
bool fBme;

// The LUX sensor
Catena_Si1133 gSi1133;
bool fLux;

// the Flash driver requires a SPI instance for the underlying transport.
SPIClass gSPI2(
        Catena::PIN_SPI2_MOSI,
        Catena::PIN_SPI2_MISO,
        Catena::PIN_SPI2_SCK
        );

//   The flash
Catena_Mx25v8035f gFlash;
bool fFlash;

// USB power
bool fUsbPower;

// the job that's used to synchronize us with the LMIC code
osjob_t sensorJob;

// the cycle time to use
unsigned gTxCycle;
// remaining before we reset to default
unsigned gTxCycleCount;

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

        /* for stm32 core, we need wider tolerances, it seems. This sets the clock tolerence to +/- 10% */
        LMIC_setClockError(10 * 65536 / 100);

        /* trigger a join by sending the first packet */
        if (! isManufacturingMode())
                {
                if (! gLoRaWAN.IsProvisioned())
                        gCatena.SafePrintf("LoRaWAN not provisioned yet. Use the commands to set it up.\n");
                else
                        {
                        /* send the first packet */
                        startSendingUplink();
                        }
                }
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
        gCatena.SafePrintf("This is the WeRadiate-ThermoSense program V%s%s.\n", sVersion, sBranch);
                {
                char sRegion[16];
                gCatena.SafePrintf("Target network: %s / %s\n",
                                gLoRaWAN.GetNetworkName(),
                                gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                                );
                }
        gCatena.SafePrintf("Current board: %s\n", gCatena.CatenaName());
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

        // set up the hook for downlinks: call receiveMessage() on downlink
        gLoRaWAN.SetReceiveBufferBufferCb(receiveMessage);

        // set up the uplink cycle time.
        setTxCycleTime(CATCFG_T_CYCLE_INITIAL, CATCFG_INTERVAL_COUNT_INITIAL);

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

        /* initialize the FLASH */
        if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
                {
                fFlash = true;
                gFlash.powerDown();
                gCatena.SafePrintf("FLASH found, put power down\n");
                }
        else
                {
                fFlash = false;
                gFlash.end();
                gSPI2.end();
                gCatena.SafePrintf("No FLASH found: check board\n");
                }

        /* is it modded? */
        uint32_t modnumber = gCatena.PlatformFlags_GetModNumber(flags);

        /* modnumber is 102 for WeRadiate app */
        if (modnumber != 0)
                {
                gCatena.SafePrintf("Catena 4612-M%u\n", modnumber);
                if (modnumber == 102)
                        {
                        fHasCompostTemp = flags & CatenaBase::fHasWaterOneWire;
                        }
                else
                        {
                        gCatena.SafePrintf("unknown mod number %d\n", modnumber);
                        }
                }
        else
                {
                gCatena.SafePrintf("No mods detected\n");
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

// return true if the compost sensor is attached.
bool checkCompostSensorPresent(void)
        {
        /* set D11 high so V_OUT2 is going to be high for onewire sensor */
        pinMode(D11, OUTPUT);
        digitalWrite(D11, HIGH);

        sensor_CompostTemp.begin();
        return sensor_CompostTemp.getDeviceCount() != 0;
        }

// setup all the on-board sensors
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
                        gCatena.SafePrintf("No Si1133 found: check platform selection\n");
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
                        gCatena.SafePrintf("No BME280 found: check platfom setting\n");
                        }
                }
        else
                {
                fBme = false;
                gCatena.SafePrintf("No BME280 found: check wiring. Just nothing. \n");
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

        /* for mfg test, don't tx, just fill */
        if (isManufacturingMode())
                {
                TxBuffer_t b;
                fillBuffer(b);
                delay(1000);
                }
        }

/*

Name:	fillBuffer()

Function:
        Make measurements and fill a TxBuffer

Definition:
        void fillBuffer(
                TxBuffer_t &b
                );

Description:
        This function initializes the buffer with a series of
        measurements taken from the sensors. If a serial port
        is attached, it also displays data; so it's useful
        for manufacturing test, even if you don't want to
        send the data.

Returns:
        No explicit result.

*/

void fillBuffer(TxBuffer_t &b)
        {
        b.begin();
        FlagsSensor3 flag;

        flag = FlagsSensor3(0);

        b.put(FormatSensor3); /* the flag for this record format */
        uint8_t * const pFlag = b.getp();
        b.put(0x00); /* will be set to the flags */

        // vBat is sent as 5000 * v
        float vBat = gCatena.ReadVbat();
        gCatena.SafePrintf("vBat:    %d mV\n", (int) (vBat * 1000.0f));
        b.putV(vBat);
        flag |= FlagsSensor3::FlagVbat;

        // vBus is sent as 5000 * v
        float const vBus = updateUsbPower();
        gCatena.SafePrintf("vBus:    %d mV\n", (int) (vBus * 1000.0f));
        b.putV(vBus);
        flag |= FlagsSensor3::FlagVcc;

        uint32_t bootCount;
        if (gCatena.getBootCount(bootCount))
                {
                b.putBootCountLsb(bootCount);
                flag |= FlagsSensor3::FlagBoot;
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

                flag |= FlagsSensor3::FlagTPH;
                }

        /*
        || Measure and transmit the compost temperature (OneWire)
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
                // transmit the measurement
                b.putT(compostTempC);
                flag |= FlagsSensor3::FlagWater;
                }
        else if (fHasCompostTemp)
                {
                gCatena.SafePrintf("No compost temperature\n");
                }

        /* set D11 low to turn off after measuring */
//        digitalWrite(D11, LOW);
        pinMode(D11, INPUT);

        *pFlag = uint8_t(flag);
        }

/*

Name:	startSendingUplink()

Function:
        Start sending a message to the cloud

Definition:
        void startSendingUplink(
                void
                );

Description:
        This function takes a set of measurements, and forwards
        them to the cloud.  It also has the side-effect of
        starting the cyclical finite state machine; at the end
        of a transmisison cycle, the system sleeps until the
        next cycle and then meausures and sends again.

Returns:
        No explicit result.

*/

void startSendingUplink(void)
        {
        TxBuffer_t b;
        LedPattern savedLed = gLed.Set(LedPattern::Measuring);

        fillBuffer(b);

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

// called from LMIC when transmit is complete.
void
sendBufferDoneCb(
        void *pContext,
        bool fStatus
        )
        {
        osjobcb_t pFn;

        gLed.Set(LedPattern::Settling);

        // assume that we'll continue the loop.
        pFn = settleDoneCb;

        if (! fStatus)
                {
                gCatena.SafePrintf("send buffer failed\n");

                // if not provisioned, shutdown the loop
                if (! gLoRaWAN.IsProvisioned())
                        pFn = txFailedDoneCb;
                }
        else
                {
                gCatena.SafePrintf("tx complete: adrAckReq: %d  adrChanged: %d\n",
                        LMIC.adrAckReq, LMIC.adrChanged
                        );
                }

        // wait for the LMIC to settle down
        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_SETTLE),
                pFn
                );
        }

// called after settling time to shutdown the LMIC in case of failure
void
txFailedDoneCb(
        osjob_t *pSendJob
        )
        {
        gCatena.SafePrintf("not provisioned, idling\n");
        gLoRaWAN.Shutdown();
        gLed.Set(LedPattern::NotProvisioned);
        }

// called after settling time to put the system to sleep
void settleDoneCb(
        osjob_t *pSendJob
        )
        {
        // if connected to USB, don't sleep
        // XXX: do we need this?
#ifdef USBCON
        const bool fCanSleep =! fUsbPower;
#else
        const bool fCanSleep = true;
#endif

        // adjust the uplink timing counters.
        updateTxCycleParameters();

        if (! fCanSleep)
                {
                /* we're not allowed to do a deep sleep, so use the LMIC mechanisms */
               ostime_t interval = sec2osticks(CATCFG_GetInterval(gTxCycle));

                gLed.Set(LedPattern::Sleeping);
                os_setTimedCallback(
                        &sensorJob,
                        os_getTime() + interval,
                        sleepDoneCb
                        );
                return;
                }
        else
                {
                /* we are allowed to do a deep sleep */
                prepareToSleep();

                ostime_t deepSleepSeconds = CATCFG_GetInterval(gTxCycle);
                gCatena.Sleep(deepSleepSeconds);

                /* and now... we're awake again. trigger another measurement */
                recoverFromSleep();

                sleepDoneCb(pSendJob);
                }
        }

// called after sleep to schedule work after sensors are awake
void sleepDoneCb(
        osjob_t *pJob
        )
        {
        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_WARMUP),
                warmupDoneCb
                );
        }

// called after warmup is done -- just sends another message.
void warmupDoneCb(
        osjob_t *pJob
        )
        {
        startSendingUplink();
        }

// prepare system for low-power sleep.
void prepareToSleep(void)
        {
        gLed.Set(LedPattern::Off);
        Serial.end();
        Wire.end();
        SPI.end();
        if (fFlash)
                gSPI2.end();
        }

// recover system after low-power sleep
void recoverFromSleep(void)
        {
        Serial.begin();
        Wire.begin();
        SPI.begin();
        if (fFlash)
                gSPI2.begin();
        gLed.Set(LedPattern::WarmingUp);
        gSi1133.start();
        }

// update the uplink cycle parameters
void updateTxCycleParameters(void)
        {
        if (gTxCycleCount > 1)
                --gTxCycleCount;
        else
                {
                if (gTxCycleCount > 0)
                        {
                       gCatena.SafePrintf("resetting tx cycle to default: %u\n", CATCFG_T_CYCLE);

                        gTxCycleCount = 0;
                        gTxCycle = CATCFG_T_CYCLE;
                        }
                }
        }

// read and return USB power, and update the global fUsbPower flag.
float updateUsbPower(void)
        {
        float vBus = gCatena.ReadVbus();
        fUsbPower = (vBus > 4.0) ? true : false;
        return vBus;
        }

/*

Name:	receiveMessage()

Function:
        Start sending a message to the cloud

Definition:
        Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;

        void receiveMessage(
                void *pContext,
                uint8_t port,
                const uint8_t *pMessage,
                size_t nMessage
                )

Description:
        This function takes a set of measurements, and forwards
        them to the cloud.  It also has the side-effect of
        starting the cyclical finite state machine; at the end
        of a transmisison cycle, the system sleeps until the
        next cycle and then meausures and sends again.

Returns:
        No explicit result.

*/

void receiveMessage(
        void *pContext,
        uint8_t port,
        const uint8_t *pMessage,
        size_t nMessage
        )
        {
        unsigned txCycle;
        unsigned txCount;

        if (port == 0)
                {
                gCatena.SafePrintf("MAC message:");
                for (unsigned i = 0; i < LMIC.dataBeg; ++i)
                        {
                        gCatena.SafePrintf(" %02x", LMIC.frame[i]);
                        }
                gCatena.SafePrintf("\n");
                return;
                }

        if (! (port == 1 && 2 <= nMessage && nMessage <= 3))
                {
                gCatena.SafePrintf("invalid message port(%02x)/length(%x)\n",
                        port, (unsigned) nMessage
                        );
                return;
                }

        txCycle = (pMessage[0] << 8) | pMessage[1];

        if (txCycle < CATCFG_T_MIN || txCycle > CATCFG_T_MAX)
                {
                gCatena.SafePrintf("tx cycle time out of range: %u\n", txCycle);
                return;
                }

        // byte [2], if present, is the repeat count.
        // explicitly sending zero causes it to stick.
        // sending nothing uses the default
        txCount = CATCFG_INTERVAL_COUNT_DEFAULT;
        if (nMessage >= 3)
                {
                txCount = pMessage[2];
                }

        setTxCycleTime(txCycle, txCount);
        }

// set the transmit cycle time
void setTxCycleTime(
        unsigned txCycle,
        unsigned txCount
        )
        {
        if (txCount > 0)
                gCatena.SafePrintf(
                        "message cycle time %u seconds for %u messages\n",
                        txCycle, txCount
                        );
        else
                gCatena.SafePrintf(
                        "message cycle time %u seconds indefinitely\n",
                        txCycle
                        );

        gTxCycle = txCycle;
        gTxCycleCount = txCount;
        }

// is device in manufacturing mode?
bool isManufacturingMode(void)
        {
        return (gCatena.GetOperatingFlags() &
                        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fManufacturingTest)) != 0;
        }
