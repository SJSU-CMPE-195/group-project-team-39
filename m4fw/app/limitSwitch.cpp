#include "limitSwitch.hpp"

limitSwitch::limitSwitch(GPIO_Type *p_base, uint32_t p_gpio_pin)
    : m_base(p_base),
      m_pin(p_gpio_pin)
{
    configASSERT(p_base != nullptr);

    gpio_pin_config_t inCfg = {
        .direction     = kGPIO_DigitalInput,
        .outputLogic   = 0U,
        .interruptMode = kGPIO_NoIntmode,
    };

    GPIO_PinInit(m_base, m_pin, &inCfg);
}

uint8_t limitSwitch::read()
{
    return static_cast<uint8_t>(GPIO_PinRead(m_base, m_pin));
}
