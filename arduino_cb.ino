#include "VescUart.h"
#include <Wire.h>
#include "SparkFunLSM9DS1.h"

#include "filter.hpp"

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

    Serial.println("Done initializing");
}

auto get_roll() -> float
{
  float roll = atan2(imu.ay, imu.az);
  return roll;
}

struct config
{
    using value_type = float;
    static constexpr auto cutoff_frequency() -> float { return 20.0f;  } // Hz
    static constexpr auto sample_period()    -> float { return 0.002f; } // seconds
};


auto p_controller(float roll) -> float
{
    auto gain = 50.f;  // A/rad

    return roll*gain;
}


void loop()
{
    auto lowpass = complementary_filter::lowpass<config>{};

    while (1)
    {
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

            auto filtered_roll = lowpass(roll);
            auto time_lowpass = micros() - time_s;

#ifdef PRINT_ON_SERIAL
            Serial.print("roll: ");
            Serial.println(filtered_roll.value * 180 / 3.1416);
            auto time_serial = micros() - time_s;
#endif

#ifdef PRINT_ON_SERIAL
            Serial.print("timing: Available, read, convert, filter, print: ");
            Serial.print(static_cast<unsigned long>(time_available));
            Serial.print(", ");
            Serial.print(static_cast<unsigned long>(time_read));
            Serial.print(", ");
            Serial.print(static_cast<unsigned long>(time_roll));
            Serial.print(", ");
            Serial.print(static_cast<unsigned long>(time_lowpass));
            Serial.print(", ");
            Serial.println(static_cast<unsigned long>(time_serial));
#endif

            current = p_controller(filtered_roll.value);
            write_command();
        }
    }
}
