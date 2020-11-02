#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_PM25AQI.h"

#define VBATPIN A7

/*** BME SETUP ***/
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

/** Initialize sensors **/
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_BME680 bme;

static uint8_t payload[26];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        PM25_AQI_Data data;
        delay(1000);
        Serial.println();
        Serial.println(F("---------------------------------------"));
        Serial.println(F("Concentration Units (standard)"));
        Serial.println(F("---------------------------------------"));
        Serial.print(F("PM 1.0: "));
        Serial.print(data.pm10_standard);
        Serial.print(F("\t\tPM 2.5: "));
        Serial.print(data.pm25_standard);
        Serial.print(F("\t\tPM 10: "));
        Serial.println(data.pm100_standard);
        Serial.println(F("Concentration Units (environmental)"));
        Serial.println(F("---------------------------------------"));
        Serial.print(F("PM 1.0: "));
        Serial.print(data.pm10_env);
        Serial.print(F("\t\tPM 2.5: "));
        Serial.print(data.pm25_env);
        Serial.print(F("\t\tPM 10: "));
        Serial.println(data.pm100_env);
        Serial.println(F("---------------------------------------"));
        Serial.print(F("Particles > 0.3um / 0.1L air:"));
        Serial.println(data.particles_03um);
        Serial.print(F("Particles > 0.5um / 0.1L air:"));
        Serial.println(data.particles_05um);
        Serial.print(F("Particles > 1.0um / 0.1L air:"));
        Serial.println(data.particles_10um);
        Serial.print(F("Particles > 2.5um / 0.1L air:"));
        Serial.println(data.particles_25um);
        Serial.print(F("Particles > 5.0um / 0.1L air:"));
        Serial.println(data.particles_50um);
        Serial.print(F("Particles > 10 um / 0.1L air:"));
        Serial.println(data.particles_100um);
        Serial.println(F("---------------------------------------"));

        Serial.print("Temperature = ");
        Serial.print(bme.temperature);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(bme.pressure / 100.0);
        Serial.println(" hPa");

        Serial.print("Humidity = ");
        Serial.print(bme.humidity);
        Serial.println(" %");

        Serial.print("Gas = ");
        Serial.print(bme.gas_resistance / 1000.0);
        Serial.println(" KOhms");

        Serial.print("Approx. Altitude = ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");
        uint8_t pm10_standard = data.pm10_standard;
        uint8_t pm25_standard = data.pm25_standard;
        uint8_t pm100_standard = data.pm100_standard;
        uint8_t pm10_env = data.pm10_env;
        uint8_t pm25_env = data.pm25_env;
        uint8_t pm100_env = data.pm100_env;
        uint16_t pm03 = data.particles_03um;
        uint16_t pm05 = data.particles_05um;
        uint16_t pm10 = data.particles_10um;
        uint16_t pm25 = data.particles_25um;
        uint16_t pm50 = data.particles_50um;
        uint16_t pm100 = data.particles_100um;

        uint16_t temperature = bme.temperature * 100;
        uint16_t pressure = (bme.pressure / 100.0) * 10;
        uint16_t humidity = bme.humidity * 100;
        // uint32_t gas = bme.gas_resistance / 1000.0;

        uint16_t measuredvbat = analogRead(VBATPIN);
        // measuredvbat *= 2;    // we divided by 2, so multiply back
        // measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
        // measuredvbat /= 1024; // convert to voltage
        Serial.print("Battery: ");
        Serial.println(measuredvbat);

        // push the data onto the payload
        // i know this is ugly... i'll change it later after testing
        payload[0] = pm10_standard;
        payload[1] = pm25_standard;
        payload[2] = pm100_standard;
        payload[3] = pm10_env;
        payload[4] = pm25_env;
        payload[5] = pm100_env;
        payload[6] = pm03 & 0xff;
        payload[7] = pm03 >> 8;
        payload[8] = pm05 & 0xff;
        payload[9] = pm05 >> 8;
        payload[10] = pm10 & 0xff;
        payload[11] = pm10 >> 8;
        payload[12] = pm25 & 0xff;
        payload[13] = pm25 >> 8;
        payload[14] = pm50 & 0xff;
        payload[15] = pm50 >> 8;
        payload[16] = pm100 && 0xff;
        payload[17] = pm100 >> 8;
        payload[18] = temperature & 0xff;
        payload[19] = temperature >> 8;
        payload[20] = pressure & 0xff;
        payload[21] = pressure >> 8;
        payload[22] = humidity & 0xff;
        payload[23] = humidity >> 8;
        payload[24] = measuredvbat & 0xff;
        payload[25] = measuredvbat >> 8;

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{

    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("Adafruit PMSA003I Air Quality Sensor");

    // Wait one second for sensor to boot up!
    delay(1000);

    // There are 3 options for connectivity!
    if (!aqi.begin_I2C())
    { // connect to the sensor over I2C
        //if (! aqi.begin_UART(&Serial1)) { // connect to the sensor over hardware serial
        //if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial
        Serial.println("Could not find PM 2.5 sensor!");
        while (1)
            delay(10);
    }

    if (!bme.begin())
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1)
            ;
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    Serial.println(F("Starting"));
    pinMode(VBATPIN, INPUT);
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(false);
    LMIC_setDrTxpow(DR_SF12CR, 23);
    LMIC_selectSubBand(1);

    delay(5000);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}
