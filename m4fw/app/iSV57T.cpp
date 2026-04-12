#include "iSV57T.hpp"

iSV57T::iSV57T(RGPIO_Type *p_base, uint32_t p_dir_pin, uint32_t p_pul_pin,
               uint16_t p_pulse_per_rev)
    : m_base(p_base),
      m_dir_pin(p_dir_pin),
      m_pul_pin(p_pul_pin),
      m_pulse_per_rev(p_pulse_per_rev)
{
    configASSERT(p_base != nullptr);

    m_base->PDDR |= (1U << m_dir_pin) | (1U << m_pul_pin);

    m_base->PCOR = (1U << m_dir_pin) | (1U << m_pul_pin);
}

void iSV57T::rotate(uint8_t p_direction, float p_degree)
{
    if (p_direction)
        m_base->PSOR = (1U << m_dir_pin);
    else
        m_base->PCOR = (1U << m_dir_pin);

    delay_us(10);

    const long pulses =
        static_cast<long>((p_degree / 360.0f) * m_pulse_per_rev + 0.5f);

    const float steps_per_sec =
        (target_rpm * static_cast<float>(m_pulse_per_rev)) / 60.0f;
    configASSERT(steps_per_sec > 0.0);

    uint32_t period_us = static_cast<uint32_t>(1000000.0f / steps_per_sec);

    if (period_us <= static_cast<uint32_t>(pulse_high_us) + 1) {
        period_us = static_cast<uint32_t>(pulse_high_us) + 2;
    }

    const uint32_t low_us = period_us - static_cast<uint32_t>(pulse_high_us);

    for (long i = 0; i < pulses; ++i) {
        m_base->PSOR = (1U << m_pul_pin);
        delay_us(static_cast<uint32_t>(pulse_high_us + 0.5f));

        m_base->PCOR = (1U << m_pul_pin);
        delay_us(low_us);
    }
}

void iSV57T::set_pulse_high_us(float p_pulse_high_us)
{
    configASSERT(p_pulse_high_us >= 2.5f && p_pulse_high_us <= 5.0f);
    pulse_high_us = p_pulse_high_us;
}

void iSV57T::set_target_rpm(float p_target_rpm)
{
    configASSERT(p_target_rpm >= 100.0f && p_target_rpm <= 3000.0f);
    target_rpm = p_target_rpm;
}
