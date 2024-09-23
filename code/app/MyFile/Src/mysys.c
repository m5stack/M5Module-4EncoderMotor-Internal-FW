
#include "mysys.h"
#include "mymotorbdc.h"
#include "mymotorencoder.h"
#include "myservos.h"
#include "i2c_ex.h"
#include "pid.h"

int8_t speed[4];
volatile int8_t motor_throttle[4];
extern volatile int32_t encoder_raw[4]; 

pid_pos_t pid_pos[4];
pid_pos_t pid_speed[4];

/*
   |  0  |       1     |      2     |     3      |    4, 5, 6, 7  |          8         |     9    |    10   |    11   |     12      |          
   | mod |  position-p | position-i | position-d | position-point | position-max-speed |  speed-p | speed-i | speed-d | speed-point | 
*/
volatile uint8_t config[4][13];

// speed (20ms speed)
void SpeedUpdate()
{
    static uint32_t next_update_time = 0;
    static int32_t encoder_last[4] = {0, 0, 0, 0};
    if(next_update_time < HAL_GetTick())
    {
        next_update_time = HAL_GetTick() + 20;
        for(uint8_t i = 0; i < 4; i++)
        {
            speed[i] = (encoder_raw[i] - encoder_last[i]);
            encoder_last[i] = encoder_raw[i];
        }
    }
}

void PositionPID(uint8_t pos)
{
    static uint32_t next_update_time_pos[4] = { 0, 0, 0, 0};
    int32_t out_data = 0;
    
    if(next_update_time_pos[pos] < HAL_GetTick())
    {
        next_update_time_pos[pos] = HAL_GetTick() + 10;
        if(((*(int32_t *)&config[pos][4] - encoder_raw[pos]) < 2) && ((*(int32_t *)&config[pos][4] - encoder_raw[pos]) > -2))
        {
            out_data = 0;
        }
        else
        {
            out_data = PIDPos(&pid_pos[pos], *(int32_t *)&config[pos][4], encoder_raw[pos]);
        }
        motor_throttle[pos] = CONSTRAIN(out_data, 0 - config[pos][8], config[pos][8]);
    }
}

void SpeedPID(uint8_t pos)
{
    static uint32_t next_update_time_speed[4] = { 0, 0, 0, 0};
    static int16_t last_speed[4] = {0, 0, 0, 0};

    int32_t out_data = 0;
    if(next_update_time_speed[pos] < HAL_GetTick())
    {
        next_update_time_speed[pos] = HAL_GetTick() + 20;
        if (*(int8_t *)&config[pos][12] == speed[pos] && *(int8_t *)&config[pos][12] == 0) {
            out_data = 0;
            motor_throttle[pos] = 0;
        }
        else {
            out_data = PIDPos(&pid_speed[pos], *(int8_t *)&config[pos][12], speed[pos]) / 5;
            out_data = CONSTRAIN(out_data, -127, 127);
            motor_throttle[pos] = CONSTRAIN(out_data + motor_throttle[pos], -127, 127);
        }
    }
}

void InitMysys(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        encoder_raw[i] = 0;
        motor_throttle[i] = 0;
        
        config[i][1] = 3;   // position-p
        config[i][2] = 1;   // position-i
        config[i][3] = 15;  // position-d
        config[i][8] = 127; // position max speed

        config[i][9] = 10;   // speed-p
        config[i][10] = 1;   // speed-i
        config[i][11] = 15;  // speed-d

        pid_pos[i].kp = (int8_t *)&config[i][1];
        pid_pos[i].ki = (int8_t *)&config[i][2];
        pid_pos[i].kd = (int8_t *)&config[i][3];

        pid_speed[i].kp = (int8_t *)&config[i][9];
        pid_speed[i].ki = (int8_t *)&config[i][10];
        pid_speed[i].kd = (int8_t *)&config[i][11];

        pid_pos[i].i_max = 7;
        pid_pos[i].i_min = -7;

        pid_speed[i].i_min = -10;
        pid_speed[i].i_max = 10;
    }

    I2CAddReg(0x20, (uint8_t *)motor_throttle, 4, 8);
    I2CAddReg(0x30, (uint8_t *)encoder_raw, 16, 32);
    I2CAddReg(0x40, (uint8_t *)speed, 4, 8);

    I2CAddReg(0x50, (uint8_t *)config[0], 13, 8);
    I2CAddReg(0x60, (uint8_t *)config[1], 13, 8);
    I2CAddReg(0x70, (uint8_t *)config[2], 13, 8);
    I2CAddReg(0x80, (uint8_t *)config[3], 13, 8);
    
    I2CInit();
    InitMotorPwm();
    InitMotorEncoder();
    ServoInit();
}

void LoopMysys(void)
{
    while(1)
    {
        ServoUpdate();
        SpeedUpdate();

        for(uint8_t i = 0; i < 4; i++)
        {
            if(config[i][0] == 0x01)
            {
                PositionPID(i);
            }
            else if(config[i][0] == 0x02)
            {
                SpeedPID(i);
            }
        }

        SetMotorThrottle(PWM_M1, motor_throttle[0]);
        SetMotorThrottle(PWM_M2, motor_throttle[1]);
        SetMotorThrottle(PWM_M3, motor_throttle[2]);
        SetMotorThrottle(PWM_M4, motor_throttle[3]);
    }
}
