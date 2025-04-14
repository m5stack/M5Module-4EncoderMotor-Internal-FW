#include "mymotorbdc.h"

#include "tim.h"

#include "mysys.h"

#include <stdlib.h>

#define M1R_H GPIOB->BSRR=(uint32_t)1<<14
#define M1R_L GPIOB->BSRR=(uint32_t)(1<<14)<<16
#define M1F_H GPIOB->BSRR=(uint32_t)1<<15
#define M1F_L GPIOB->BSRR=(uint32_t)(1<<15)<<16

#define M2R_H GPIOB->BSRR=(uint32_t)1<<12
#define M2R_L GPIOB->BSRR=(uint32_t)(1<<12)<<16
#define M2F_H GPIOB->BSRR=(uint32_t)1<<13
#define M2F_L GPIOB->BSRR=(uint32_t)(1<<13)<<16

#define M3R_H GPIOB->BSRR=(uint32_t)1<<4
#define M3R_L GPIOB->BSRR=(uint32_t)(1<<4)<<16
#define M3F_H GPIOB->BSRR=(uint32_t)1<<5
#define M3F_L GPIOB->BSRR=(uint32_t)(1<<5)<<16

#define M4R_H GPIOA->BSRR=(uint32_t)1<<15
#define M4R_L GPIOA->BSRR=(uint32_t)(1<<15)<<16
#define M4F_H GPIOB->BSRR=(uint32_t)1<<3
#define M4F_L GPIOB->BSRR=(uint32_t)(1<<3)<<16

uint32_t normal_motor_pid_velocity_output_ramp = 100;
float normal_output_rate[4] = {0};
int16_t setpoint[4] = {0}, last_setpoint[4] = {0};
int16_t output[4] = {0}, output_prev[4] = {0};
uint32_t my_timestamp_now[4] = {0};
uint32_t my_timestamp_prev[4] = {0};
float my_Ts[4] = {0};
uint8_t is_bootup[4] = {0};

void SetMotorDir(uint8_t port,uint8_t dir)
{
    switch(port)
    {
        case PWM_M1:
        {
            if(dir)
            {
                // M1F_H;
                // M1R_L;
                M1F_L;
                M1R_H;
            }
            else
            {
                // M1R_H;
                // M1F_L;
                M1R_L;
                M1F_H;
            }
            break;
        }
        case PWM_M2:
        {
            if(dir)
            {
                // M2F_H;
                // M2R_L;
                M2F_L;
                M2R_H;
            }
            else
            {
                // M2R_H;
                // M2F_L;
                M2R_L;
                M2F_H;
            }
            break;
        }
        case PWM_M3:
        {
            if(dir)
            {
                // M3F_H;
                // M3R_L;
                M3F_L;
                M3R_H;
            }
            else
            {
                // M3R_H;
                // M3F_L;
                M3R_L;
                M3F_H;
            }
            break;
        }
        case PWM_M4:
        {
            if(dir)
            {
                // M4F_H;
                // M4R_L;
                M4F_L;
                M4R_H;
            }
            else
            {
                // M4R_H;
                // M4F_L;
                M4R_L;
                M4F_H;
            }
            break;
        }
    }
}

void SetMotorPwm(uint8_t port,uint8_t value)
{
    switch(port)
    {
        case PWM_M1:
        {
            TIM1->CCR2=value;
            break;
        }
        case PWM_M2:
        {
            TIM1->CCR1=value;
            break;
        }
        case PWM_M3:
        {
            TIM1->CCR4=value;
            break;
        }
        case PWM_M4:
        {
            TIM1->CCR3=value;
            break;
        }
    }
}

void SetMotorThrottle(uint8_t port, int8_t throttle)
{
    int32_t ramp_temp;
	
    if(config[port][0] == 0x00 && (((soft_start_stop_switch >> port) & 0x01)== 0x01)) {
        setpoint[port] = throttle;
        if (last_setpoint[port] != setpoint[port]) {
            if (abs(last_setpoint[port]) <= 20) {
                is_bootup[port] = 1;
            }
            else if (abs(setpoint[port]) <= 20) {
                is_bootup[port] = 2;
            }
            else if ((setpoint[port] > 0 && last_setpoint[port] < 0) || (setpoint[port] < 0 && last_setpoint[port] > 0)) {
                is_bootup[port] = 3;
            }
            output[port] = setpoint[port];
            last_setpoint[port] = setpoint[port];
        }
        my_timestamp_now[port] = micros();
        my_Ts[port] = (my_timestamp_now[port] - my_timestamp_prev[port]) * 1e-6f; 
        if (is_bootup[port]) {
            if (my_Ts[port] >= 0.1f) {
                if(normal_motor_pid_velocity_output_ramp > 0) {
                    // limit the acceleration by ramping the output
                    normal_output_rate[port] = (setpoint[port] - output_prev[port]) / my_Ts[port];
                    ramp_temp = -normal_motor_pid_velocity_output_ramp;
                    if (normal_output_rate[port] > normal_motor_pid_velocity_output_ramp) {
                        output[port] = output_prev[port] + normal_motor_pid_velocity_output_ramp * my_Ts[port];
                    }
                    else if (normal_output_rate[port] < ramp_temp)
                        output[port] = output_prev[port] - normal_motor_pid_velocity_output_ramp * my_Ts[port];
                    else if ((normal_output_rate[port] <= normal_motor_pid_velocity_output_ramp) && (normal_output_rate[port] >= ramp_temp)) {
                        output[port] = setpoint[port];
                        is_bootup[port] = 0;
                    }
                }

                if(output[port]>0)
                {
                    SetMotorDir(port, MDIR_FFW);
                    SetMotorPwm(port, output[port] * 255 / 127);
                }
                else
                {
                    SetMotorDir(port, MDIR_REV);
                    SetMotorPwm(port, (-output[port]) * 255 / 127);   
                }        
                output_prev[port] = output[port];
                my_timestamp_prev[port] = my_timestamp_now[port]; 
            } 
        }
        else {
            if(throttle>0)
            {
                SetMotorDir(port, MDIR_FFW);
                SetMotorPwm(port, throttle * 255 / 127);
            }
            else
            {
                SetMotorDir(port, MDIR_REV);
                SetMotorPwm(port, (-throttle) * 255 / 127);   
            }
            output_prev[port] = output[port];
            my_timestamp_prev[port] = my_timestamp_now[port];                         
        } 
    }
    else {
        if(throttle>0)
        {
            SetMotorDir(port, MDIR_FFW);
            SetMotorPwm(port, throttle * 255 / 127);
        }
        else
        {
            SetMotorDir(port, MDIR_REV);
            SetMotorPwm(port, (-throttle) * 255 / 127);
        }
    }
}


void InitMotorPwm(void)
{
    //initial hardware pwm
    #if MPWM_HIGHFREQUENCY
    TIM1->PSC=32-1;
    TIM1->ARR=255-1;
    #else
    TIM1->PSC=188-1;
    TIM1->ARR=255-1;    
    #endif

    
    //reset motor throttle
    SetMotorThrottle(PWM_M1 ,0);
    SetMotorThrottle(PWM_M2, 0);
    SetMotorThrottle(PWM_M3, 0);
    SetMotorThrottle(PWM_M4, 0);
    
    //start pwm generating
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

