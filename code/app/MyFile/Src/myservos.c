#include "myservos.h"
#include "tim.h"
#include "i2c_ex.h"

volatile uint8_t servo_tim_count[2];
volatile uint16_t servo_pulse[2];
volatile uint8_t servo_angle[2];

uint8_t servo_angle_last[2];
uint16_t servo_pulse_last[2];

#define ANGLE_TO_PULSE(angle) ((uint16_t)(500 + (angle * 2000 / 180)))
#define PULSE_TO_COUNT(pulse) ((uint8_t)(pulse / 10))
#define PULSE_TO_ANGLE(pulse) ((uint8_t)((pulse - 500) * 180 / 2000))

static void UpdateTimPulse(uint8_t pos, uint16_t pulse)
{
  if(pos == 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
  }
  else if(pos == 1)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);

  }
}

void ServoInit(void)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        servo_angle_last[i] = 255;
        servo_angle[i] = 255;
    }

    I2CAddReg(0x00, (uint8_t *)servo_angle, 2, 8);
    I2CAddReg(0x10, (uint8_t *)servo_pulse, 4, 16);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

}

void SetServoAngle(uint8_t pos, uint8_t angle) 
{
    if(angle > 180) return;
    servo_angle[pos] = angle;
    servo_pulse[pos] = ANGLE_TO_PULSE(angle);
    servo_tim_count[pos] = PULSE_TO_COUNT(servo_pulse[pos]);
    UpdateTimPulse(pos, servo_pulse[pos]);
}

void SetServoPulse(uint8_t pos, uint16_t pulse)
{
    if(pulse > 2500) return;
    if(pulse < 500) return;
    servo_pulse[pos] = pulse;
    servo_angle[pos] = PULSE_TO_ANGLE(pulse);
    servo_tim_count[pos] = PULSE_TO_COUNT(pulse);
    UpdateTimPulse(pos, servo_pulse[pos]);  
}

void ServoUpdate(void)
{
    for(uint8_t i = 0; i < 2; i++) {
        if(servo_angle[i] != servo_angle_last[i]) 
        {
            SetServoAngle(i, servo_angle[i]);
            servo_angle_last[i] = servo_angle[i];
            servo_pulse_last[i] = servo_pulse[i];
        }

        if(servo_pulse[i] != servo_pulse_last[i])
        {
            SetServoPulse(i, servo_pulse[i]);
            servo_angle_last[i] = servo_angle[i];
            servo_pulse_last[i] = servo_pulse[i];
        }
    }
}
