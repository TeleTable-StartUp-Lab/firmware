#include "ir_sensors.h"
#include "pins.h"

namespace IR
{

    void init()
    {
        pinMode(IR_LEFT_PIN, INPUT);
        pinMode(IR_RIGHT_PIN, INPUT);
    }

    int readLeft()
    {
        return digitalRead(IR_LEFT_PIN); // 1 = nothing, 0 = discovered something
    }

    int readRight()
    {
        return digitalRead(IR_RIGHT_PIN);
    }

}
