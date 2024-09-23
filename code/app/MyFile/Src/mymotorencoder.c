#include "mymotorencoder.h"
#include "tim.h"

//int32_t encoder_raw[0],encoder_raw[1],encoder_raw[2],encoder_raw[3];
volatile int32_t encoder_raw[4];

int32_t GetEncoderPos(uint8_t port)
{
    int32_t pos;
    switch(port)
    {
        case ENC_M1:
        {
            pos=encoder_raw[0];
            break;
        }
        case ENC_M2:
        {
            pos=encoder_raw[1];
            break;
        }
        case ENC_M3:
        {
            pos=encoder_raw[2];
            break;
        }
        case ENC_M4:
        {
            pos=encoder_raw[3];
            break;
        }
    }
    return pos;    
}

void SetEncoderPos(uint8_t port,int32_t pos)
{
    switch(port)
    {
        case ENC_M1:
        {
            encoder_raw[0]=pos;
            break;
        }
        case ENC_M2:
        {
            encoder_raw[1]=pos;
            break;
        }
        case ENC_M3:
        {
            encoder_raw[2]=pos;
            break;
        }
        case ENC_M4:
        {
            encoder_raw[3]=pos;
            break;
        }
    }
}
void ResetEncoderPos(uint8_t port)
{
    switch(port)
    {
        case ENC_M1:
        {
            encoder_raw[0]=0;
            break;
        }
        case ENC_M2:
        {
            encoder_raw[1]=0;
            break;
        }
        case ENC_M3:
        {
            encoder_raw[2]=0;
            break;
        }
        case ENC_M4:
        {
            encoder_raw[3]=0;
            break;
        }
    }
}

void InitMotorEncoder(void)
{
   ResetEncoderPos(ENC_M1);
   ResetEncoderPos(ENC_M2);
   ResetEncoderPos(ENC_M3);
   ResetEncoderPos(ENC_M4);
}

void InterruptMotorEncoder(void)
{
    volatile uint32_t gpioa_buf = GPIOA->IDR;
    volatile uint32_t gpiob_buf = GPIOB->IDR;

    if (!encoder_ab_mode) {
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET) //M1, PA6+PA7
        {

            if((gpioa_buf & GPIO_PIN_7) ^ ((gpioa_buf & GPIO_PIN_6) << 1))
            {
                encoder_raw[0] += 1;
            }
            else
            {
                encoder_raw[0] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) //M2, PA4+PA5
        {
            if((gpioa_buf & GPIO_PIN_5) ^ ((gpioa_buf & GPIO_PIN_4) << 1))
            {
                encoder_raw[1] += 1;
            }
            else
            {
                encoder_raw[1] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) //M3, PB9+PB8
        {
            if(((gpiob_buf & GPIO_PIN_8) << 1) ^ (gpiob_buf & GPIO_PIN_9))
            {
                encoder_raw[2] += 1;
            }
            else
            {
                encoder_raw[2] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) //M4, PB7+PB6
        {
            if((gpiob_buf & GPIO_PIN_6) ^ ((gpiob_buf & GPIO_PIN_7) >> 1))
            {
                encoder_raw[3] += 1;
            }
            else
            {
                encoder_raw[3] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
        }
    }
    else {
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) //M1, PA6+PA7
        {

            if((gpioa_buf & GPIO_PIN_6) ^ ((gpioa_buf & GPIO_PIN_7) >> 1))
            {
                encoder_raw[0] += 1;
            }
            else
            {
                encoder_raw[0] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) //M2, PA4+PA5
        {
            if((gpioa_buf & GPIO_PIN_4) ^ ((gpioa_buf & GPIO_PIN_5) >> 1))
            {
                encoder_raw[1] += 1;
            }
            else
            {
                encoder_raw[1] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) //M3, PB9+PB8
        {
            if(((gpiob_buf & GPIO_PIN_9) >> 1) ^ (gpiob_buf & GPIO_PIN_8))
            {
                encoder_raw[2] += 1;
            }
            else
            {
                encoder_raw[2] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
        }
        
        if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET) //M4, PB7+PB6
        {
            if((gpiob_buf & GPIO_PIN_6) ^ ((gpiob_buf & GPIO_PIN_7) >> 1))
            {
                encoder_raw[3] += 1;
            }
            else
            {
                encoder_raw[3] -= 1;
            }
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
        }
    }


}

