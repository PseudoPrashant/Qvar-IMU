#include <Wire.h>

// ============================================================
// ESP32 PINS
// ============================================================

#define SDA_PIN   21
#define SCL_PIN   22
#define INT2_PIN  27

// ============================================================
// ISM330BX REGISTERS
// ============================================================

#define WHO_AM_I    0x0F

#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define CTRL6_G     0x15
#define CTRL7       0x16
#define CTRL8_XL    0x17
#define CTRL9_XL    0x18
#define CTRL10_C    0x19

#define STATUS_REG  0x1E

#define OUTX_L_G    0x22
#define OUTX_L_A    0x28

#define QVAR_OUT_L  0x3A

#define WHO_AM_I_VALUE 0x71

// ============================================================
// CONFIGURATION
// ============================================================

uint8_t sensorAddress = 0x6A;

// ------------------------------------------------------------
// QVAR configurations
//
// 0x88:
// bit 7 = AH_QVAR_EN
// bit 3 = AH_QVAR1_EN
//
// 0x84:
// bit 7 = AH_QVAR_EN
// bit 2 = AH_QVAR2_EN
//
// No QVAR interrupt is used here.
// We are polling the QVAR output directly.
// ------------------------------------------------------------

#define QVAR1_CONFIG  0x88
#define QVAR2_CONFIG  0x84

// Accelerometer
#define ACC_CONFIG    0x06

// Gyroscope
#define GYRO_CONFIG   0x06

// QVAR sensitivity
#define QVAR_SENSITIVITY 78.0f

// STM32 firmware uses 5 ms settling
#define QVAR_SETTLING_MS 5

// Sample interval
#define SAMPLE_INTERVAL_MS 20

unsigned long lastSample = 0;


// ============================================================
// I2C WRITE
// ============================================================

void writeReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(sensorAddress);

    Wire.write(reg);
    Wire.write(value);

    Wire.endTransmission();
}


// ============================================================
// I2C READ
// ============================================================

uint8_t readReg(uint8_t reg)
{
    Wire.beginTransmission(sensorAddress);
    Wire.write(reg);

    Wire.endTransmission(false);

    Wire.requestFrom(
        sensorAddress,
        (uint8_t)1
    );

    if (Wire.available())
        return Wire.read();

    return 0;
}


// ============================================================
// READ MULTIPLE REGISTERS
// ============================================================

void readRegisters(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length
)
{
    Wire.beginTransmission(sensorAddress);

    Wire.write(reg);

    Wire.endTransmission(false);

    Wire.requestFrom(
        sensorAddress,
        length
    );

    for (uint8_t i = 0; i < length; i++)
    {
        if (Wire.available())
            buffer[i] = Wire.read();
        else
            buffer[i] = 0;
    }
}


// ============================================================
// READ 16-BIT VALUE
// ============================================================

int16_t read16(uint8_t reg)
{
    uint8_t data[2];

    readRegisters(
        reg,
        data,
        2
    );

    return (int16_t)(
        ((uint16_t)data[1] << 8) |
        data[0]
    );
}


// ============================================================
// FIND SENSOR
// ============================================================

bool findSensor()
{
    uint8_t addresses[] = {
        0x6A,
        0x6B
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t addr = addresses[i];

        Wire.beginTransmission(addr);

        if (Wire.endTransmission() == 0)
        {
            sensorAddress = addr;

            uint8_t whoami =
                readReg(WHO_AM_I);

            if (whoami == WHO_AM_I_VALUE)
            {
                return true;
            }
        }
    }

    return false;
}


// ============================================================
// SELECT QVAR1
// ============================================================

void selectQvar1()
{
    writeReg(
        CTRL7,
        QVAR1_CONFIG
    );

    // Same settling time used by STM32 firmware
    delay(QVAR_SETTLING_MS);
}


// ============================================================
// SELECT QVAR2
// ============================================================

void selectQvar2()
{
    writeReg(
        CTRL7,
        QVAR2_CONFIG
    );

    // Same settling time used by STM32 firmware
    delay(QVAR_SETTLING_MS);
}


// ============================================================
// CONFIGURE ISM330BX
// ============================================================

void configureSensor()
{
    // --------------------------------------------------------
    // First power down ACC + GYRO
    // --------------------------------------------------------

    writeReg(
        CTRL1_XL,
        0x00
    );

    writeReg(
        CTRL2_G,
        0x00
    );

    delay(20);


    // --------------------------------------------------------
    // Basic configuration
    // --------------------------------------------------------

    writeReg(
        CTRL3_C,
        0x44
    );

    writeReg(
        CTRL6_G,
        0x04
    );


    // --------------------------------------------------------
    // QVAR filter
    //
    // KEEP EXACTLY AS WORKING FIRMWARE
    //
    // HPF OFF
    // LPF OFF
    // --------------------------------------------------------

    writeReg(
        CTRL8_XL,
        0x00
    );

    writeReg(
        CTRL9_XL,
        0x00
    );

    writeReg(
        CTRL10_C,
        0x00
    );


    // --------------------------------------------------------
    // Enable QVAR1 initially
    //
    // IMPORTANT:
    // Same configuration family as your known-good firmware.
    // --------------------------------------------------------

    writeReg(
        CTRL7,
        QVAR1_CONFIG
    );

    delay(10);


    // --------------------------------------------------------
    // Enable accelerometer
    // --------------------------------------------------------

    writeReg(
        CTRL1_XL,
        ACC_CONFIG
    );


    // --------------------------------------------------------
    // Enable gyroscope
    // --------------------------------------------------------

    writeReg(
        CTRL2_G,
        GYRO_CONFIG
    );

    delay(100);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    Wire.setClock(100000);


    // --------------------------------------------------------
    // FIND SENSOR
    // --------------------------------------------------------

    if (!findSensor())
    {
        while (1)
        {
            delay(1000);
        }
    }


    // --------------------------------------------------------
    // CONFIGURE
    // --------------------------------------------------------

    configureSensor();


    // --------------------------------------------------------
    // INT2
    //
    // We don't actually use the interrupt in this test,
    // but leave the pin configuration compatible with
    // your previous firmware.
    // --------------------------------------------------------

    pinMode(
        INT2_PIN,
        INPUT
    );


    // --------------------------------------------------------
    // CSV HEADER
    // --------------------------------------------------------

    Serial.println(
        "timestamp,ax,ay,az,gx,gy,gz,qvar1,qvar2"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    if (
        millis() - lastSample
        < SAMPLE_INTERVAL_MS
    )
    {
        return;
    }

    lastSample = millis();


    // ========================================================
    // ACCELEROMETER
    // ========================================================

    int16_t rawAX =
        read16(OUTX_L_A + 0);

    int16_t rawAY =
        read16(OUTX_L_A + 2);

    int16_t rawAZ =
        read16(OUTX_L_A + 4);


    float ax =
        rawAX * 0.061f;

    float ay =
        rawAY * 0.061f;

    float az =
        rawAZ * 0.061f;


    // ========================================================
    // GYROSCOPE
    // ========================================================

    int16_t rawGX =
        read16(OUTX_L_G + 0);

    int16_t rawGY =
        read16(OUTX_L_G + 2);

    int16_t rawGZ =
        read16(OUTX_L_G + 4);


    float gx =
        rawGX * 0.00875f;

    float gy =
        rawGY * 0.00875f;

    float gz =
        rawGZ * 0.00875f;


    // ========================================================
    // QVAR1
    // ========================================================

    selectQvar1();

    int16_t rawQVAR1 =
        read16(QVAR_OUT_L);

    float qvar1 =
        rawQVAR1 / QVAR_SENSITIVITY;


    // ========================================================
    // QVAR2
    // ========================================================

    selectQvar2();

    int16_t rawQVAR2 =
        read16(QVAR_OUT_L);

    float qvar2 =
        rawQVAR2 / QVAR_SENSITIVITY;


    // ========================================================
    // RESTORE QVAR1
    //
    // This keeps the sensor in the same state as your
    // original working firmware after each measurement.
    // ========================================================

    writeReg(
        CTRL7,
        QVAR1_CONFIG
    );


    // ========================================================
    // CSV OUTPUT
    // ========================================================

    Serial.printf(
        "%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\n",

        millis(),

        ax,
        ay,
        az,

        gx,
        gy,
        gz,

        qvar1,
        qvar2
    );
}