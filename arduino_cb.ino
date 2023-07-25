#include "VescUart.h"
#include <Wire.h>
#include "SparkFunLSM9DS1.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

static int current = 0;

/* #define PRINT_ON_SERIAL 1 */

auto write_command() -> void
{
#if 0
    if (current >= 0)
    {
        VescUartSetCurrent(current);
    }
    else
    {
        VescUartSetCurrentBrake(-current);
    }
#else
    VescUartSetCurrent(current);
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {};

    Wire.begin();
    if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
    {
        Serial.println("Failed to communicate with LSM9DS1.");
        Serial.println("Double-check wiring.");
        Serial.println("Default settings in this sketch will " \
                "work for an out of the box LSM9DS1 " \
                "Breakout, but may need to be modified " \
                "if the board jumpers are.");
        while (1);
    }
}

auto get_roll() -> float
{
  float roll = atan2(imu.ay, imu.az);
  return roll;
}

void loop()
{
    while (1)
    {
        delay(10);

        auto time_s = micros();
        // Update the sensor values whenever new data is available
        if ( imu.accelAvailable() )
        {
            auto time_available = micros() - time_s;
            // To read from the accelerometer, first call the
            // readAccel() function. When it exits, it'll update the
            // ax, ay, and az variables with the most current data.
            imu.readAccel();
            auto time_read = micros() - time_s;

            auto roll = get_roll();
            auto time_roll = micros() - time_s;

#ifdef PRINT_ON_SERIAL
            Serial.print("roll: ");
            Serial.println(roll * 180 / 3.1416);
            auto time_serial = micros() - time_s;
#endif

#ifdef PRINT_ON_SERIAL
            Serial.print("timing: Available, read, convert, print: ");
            Serial.print(static_cast<unsigned long>(time_available));
            Serial.print(", ");
            Serial.print(static_cast<unsigned long>(time_read));
            Serial.print(", ");
            Serial.print(static_cast<unsigned long>(time_roll));
            Serial.print(", ");
            Serial.println(static_cast<unsigned long>(time_serial));
#endif

            if (roll > 10. * 3.14 / 180.)
            {
                current = +15;
            }
            else if (roll < -10. * 3.14 / 180.)
            {
                current = -15;
            }
            else
            {
                current = 0;
            }
            write_command();
        }
    }
}
