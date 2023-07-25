#include "VescUart.h"

static constexpr int pin_left_hard = 5;
static constexpr int pin_left_soft = 7;
static constexpr int pin_right_soft = 6;
static constexpr int pin_right_hard = 4;

static constexpr int soft_current = 20;
static constexpr int hard_current = 60;

static int current = 0;

auto setup_io_pins() -> void
{
    pinMode(pin_left_hard, INPUT_PULLUP);
    pinMode(pin_left_soft, INPUT_PULLUP);
    pinMode(pin_right_soft, INPUT_PULLUP);
    pinMode(pin_right_hard, INPUT_PULLUP);

    pinMode(LED_BUILTIN, OUTPUT);
}

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
    setup_io_pins();

    Serial.begin(115200);
    while (!Serial) {};
}


auto is_active(int pin) -> bool
{
    return !digitalRead(pin);
}

void loop()
{
    while (1)
    {
        delay(10);
/*         delay(500); */

        {
            auto left_soft = is_active(pin_left_soft);
            if (left_soft)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                current = -soft_current;
                write_command();
                continue;
            }
        }

        {
            auto right_soft = is_active(pin_right_soft);
            if (right_soft)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                current = +soft_current;
                write_command();
                continue;
            }
        }

        {
            auto left_hard = is_active(pin_left_hard);
            if (left_hard)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                current = -hard_current;
                write_command();
                continue;
            }
        }

        {
            auto right_hard = is_active(pin_right_hard);
            if (right_hard)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                current = +hard_current;
                write_command();
                continue;
            }
        }

        // if reach then no button is pressed: Set current to 0
        digitalWrite(LED_BUILTIN, LOW);
        current = 0;
        write_command();
    }
}
