#include "daisy_seed.h"
#include "SX1509.h"

using namespace daisy;

DaisySeed hw;
SX1509 sx;

int main(void)
{
    hw.Init();

    SX1509::Config i2c_conf;
    i2c_conf.transport_config.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.transport_config.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.transport_config.i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.transport_config.i2c_config.pin_config.scl = seed::D13;
    i2c_conf.transport_config.i2c_config.pin_config.sda = seed::D14;
    i2c_conf.transport_config.i2c_address = 0x3E;
    sx.Init(i2c_conf);
    sx.Check();

    sx.PinMode(7, PIN_INPUT_PULLUP);
    sx.PinMode(0, PIN_OUTPUT);
    sx.DebounceConfig(1);
    sx.DebouncePin(7);

    while(1) {
        if (sx.ReadPin(7))
        {
            sx.WritePin(0, HIGH);
        }
        else
        {
            sx.WritePin(0, LOW);
        }
    }
}
