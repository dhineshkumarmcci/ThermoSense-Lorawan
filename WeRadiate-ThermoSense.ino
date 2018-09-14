#include <Catena.h>
#include <Catena_Led.h>
#include <CatenaStm32L0Rtc.h>

using namespace McciCatena;

Catena gCatena;

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

const char sVersion[] = "V1.0";

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gCatena.begin();

  pinMode(gkVddBoostEnable, OUTPUT);
  digitalWrite(gkVddBoostEnable, 1);
  pinMode(gkTcxoVdd, OUTPUT);
  digitalWrite(gkTcxoVdd, 1);
  
  gCatena.SafePrintf("Catena 4612 test01 V%s\n", sVersion);

#ifdef USBCON
  gCatena.SafePrintf("USB enabled\n");
#else
  gCatena.SafePrintf("USB disabled\n");
#endif

  gLed.begin();
  gCatena.registerObject(&gLed);

  gCatena.SafePrintf("set up RTC\n");
  gRtc.begin();

  gCatena.SafePrintf("LoRaWAN init: ");
  if (!gLoRaWAN.begin(&gCatena))
    {
    gCatena.SafePrintf("failed\n");
    gCatena.registerObject(&gLoRaWAN);
    }
  else
    {
    gCatena.SafePrintf("OK\n");
    gCatena.registerObject(&gLoRaWAN);
    }

  gCatena.SafePrintf("End of setup\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  gCatena.poll();
  Serial.println("hello world");
  delay(2000);
}
