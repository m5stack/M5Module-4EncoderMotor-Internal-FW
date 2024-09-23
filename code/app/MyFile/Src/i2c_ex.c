/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include <string.h>
#include <stdlib.h>
#include "stdint.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include "i2c_ex.h"
#include "flash.h"

#define I2C_WRITE_OPERATION		0
#define I2C_READ_OPERATION		1
#define	I2C_RECEIVE_BUFFER_LEN	16

#define FIRMWARE_VERSION 3

static const uint32_t crc_table[0x100] = {
  0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 
  0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75, 0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD, 
  0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D, 
  0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 
  0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072, 0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA, 
  0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA, 
  0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 
  0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2, 0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A, 
  0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53, 
  0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 
  0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B, 0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3, 
  0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3, 
  0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 
  0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC, 0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654, 
  0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4, 
  0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4, 
};

typedef struct _I2CReg_t {
	uint8_t address;
	uint8_t* user_buff;
	uint8_t* i2c_buff;
	uint8_t bit;
	uint8_t len;
	struct _I2CReg_t* next;
	/* data */
}	I2CReg_t;

I2CReg_t *reg_list = NULL;

volatile uint8_t rx_buffer[I2C_RECEIVE_BUFFER_LEN];
volatile uint8_t tx_buffer[I2C_RECEIVE_BUFFER_LEN];
volatile uint8_t Receive_Buffer[I2C_UPGRADE_BUFFER_LEN] = {0};
volatile uint16_t tx_len = 0;
volatile uint8_t tx_state = 0;

volatile uint8_t fm_version = FIRMWARE_VERSION;

volatile uint32_t jump_bootloader_timeout = 0;

static void I2CRead(uint8_t reg);
static void I2CWrite(uint8_t reg, uint8_t* data, uint8_t len);

/* --- CRC32 --- */
uint32_t compute_cfg_crc32(uint8_t *p_data, uint16_t length){
    uint32_t chk_sum = 0xFFFFFFFF;

    for(unsigned int i=0; i < length - 4; i++)
    {
        uint8_t top = (uint8_t)(chk_sum >> 24);
        top ^= p_data[i];
        chk_sum = (chk_sum << 8) ^ crc_table[top];
    }

    return chk_sum;
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_10 | GPIO_PIN_11);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

__attribute__((weak)) void i2c2_receive_callback(uint8_t *rx_data, uint16_t len) {
	/* Prevent unused argument(s) compilation warning */
	if(len <= 1)
	{
		if (rx_data[0] == 0xFE)
		{
			i2c2_set_send_data((uint8_t *)&fm_version, 1);
		}
		else if (rx_data[0] == 0xFF)
		{
			i2c2_set_send_data(i2c_address, 1);
		}	 
		else if (rx_data[0] == 0xFC)
		{
			uint8_t bootloader_version = *(uint8_t*)(0x08001000-1);
			i2c2_set_send_data((uint8_t *)&bootloader_version, 1);
		}	 
		else if (rx_data[0] >= 0x90 && rx_data[0] <= 0x93)
		{
			i2c2_set_send_data((uint8_t *)&i_in_value, 4);
		}			
		else if (rx_data[0] >= 0xC0 && rx_data[0] <= 0xC3)
		{
			int32_t i_in_value_int = i_in_value * 100;
			i2c2_set_send_data((uint8_t *)&i_in_value_int, 4);			
		}			
		else if (rx_data[0] == 0xA0)
		{
			i2c2_set_send_data(usAdcValue8, 1);
		}
		else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xB1)
		{
			i2c2_set_send_data((uint8_t *)&usAdcValue16[0], 2);
		}						
		else if (rx_data[0] == 0xD0)
		{
			i2c2_set_send_data((uint8_t *)&encoder_ab_mode, 1);
		}						
		else if (rx_data[0] == 0xD1)
		{
			i2c2_set_send_data((uint8_t *)&soft_start_stop_switch, 1);
		}						
		else		
			I2CRead(rx_data[0]);
	} 
	else
	{
		if (rx_data[0] == 0xFF)
		{
			if (len == 2) {
				if (rx_data[1] < 128) {
					i2c_address[0] = rx_data[1];
					flash_data_write_back();
					user_i2c_init();
				}
			} 
		}
		else if (rx_data[0] == 0xFD)
		{
			if (rx_data[1] == 1) {
				HAL_I2C_DeInit(&hi2c2);  
				i2c_port_set_to_input();
				HAL_TIM_Base_MspDeInit(&htim3);
				HAL_TIM_PWM_DeInit(&htim1);
				HAL_ADC_DeInit(&hadc);
				__HAL_RCC_DMA1_CLK_DISABLE();
				HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);       
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);       
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);    
				HAL_TIM_Base_MspDeInit(&htim1);   
				while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
				{
					jump_bootloader_timeout++;
					if (jump_bootloader_timeout >= 60000) {
					flag_jump_bootloader = 0;
					break;
					}
				}
				if (jump_bootloader_timeout < 60000) {
					HAL_NVIC_SystemReset();
				} else {
					user_i2c_init();
					MX_GPIO_Init();
					MX_DMA_Init();
					MX_TIM1_Init();
					MX_ADC_Init();
					MX_TIM3_Init();  
					HAL_I2C_EnableListen_IT(&hi2c2);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);        
					jump_bootloader_timeout = 0;
				}			    
			}
		}
		else if (rx_data[0] == 0xE0) {
			uint16_t data_index = (rx_data[1] | (rx_data[2] << 8));
			uint16_t data_length = (rx_data[3] | (rx_data[4] << 8));
			if (data_index >= I2C_UPGRADE_BUFFER_LEN) data_index = I2C_UPGRADE_BUFFER_LEN-1;
			Receive_Buffer[data_index] = rx_data[5];
			if (data_index == data_length - 1) {
				uint32_t crc32_rx = (Receive_Buffer[data_length-4] | (Receive_Buffer[data_length-3] << 8) |
				(Receive_Buffer[data_length-2] << 16) | (Receive_Buffer[data_length-1] << 24));
				uint32_t crc32_compute = compute_cfg_crc32(Receive_Buffer, data_length);
				if (crc32_compute == crc32_rx) {
					while(!Write_Code());
				}
			}
		}	 
		else if (rx_data[0] == 0xD0) {
			encoder_ab_mode = rx_data[1];
			set_ab_mode();
			flash_data_write_back();
		}  
		else if (rx_data[0] == 0xD1) {
			soft_start_stop_switch = rx_data[1];
			flash_data_write_back();
		}  
		else			
			I2CWrite(rx_data[0], &rx_data[1], len - 1);
	}
	/* NOTE : This function should not be modified, when the callback is needed,
			  the i2c2_receive_callback could be implemented in the user file
	 */
}

__attribute__((weak)) void i2c2_addr_req_callback(uint8_t TransferDirection) {
  UNUSED(TransferDirection);
}

void i2c2_set_send_data(uint8_t *tx_ptr, uint16_t len) {
  if (len > I2C_RECEIVE_BUFFER_LEN) {
    len = I2C_RECEIVE_BUFFER_LEN;
	}
	tx_len = len;

  if (len == 0 || tx_ptr == NULL) {
    return;
  }
  memcpy((void *)tx_buffer, tx_ptr, len);
}

void I2CInit(void)
{
  HAL_I2C_EnableListen_IT(&hi2c2);	
}

void I2CAddReg(uint8_t reg, uint8_t* buff, uint8_t len, uint8_t bit)
{
	I2CReg_t *end = reg_list;
  
	if(end != NULL)
	{
		while (end->next != NULL)
		{
			end = end->next;
		}
		end->next = (I2CReg_t*)malloc(sizeof(I2CReg_t));
		end = end->next;
	} else {
		end = (I2CReg_t*)malloc(sizeof(I2CReg_t));
		reg_list = end;
	}

	end->next = NULL;
	end->address = reg;
	end->user_buff = buff;
  end->bit = bit;
	end->len = len;
  end->i2c_buff = (uint8_t *)malloc(len);
	memcpy(end->i2c_buff, end->user_buff, end->len);
}

uint8_t I2CGetTxState()
{
	return tx_state;
}

static void I2CRead(uint8_t reg)
{
	I2CReg_t* list_ptr = reg_list;
	uint8_t offset = reg & 0x0f;
	uint8_t num = 0;
	uint8_t len = 0;
	reg = reg & 0xf0;

	while(list_ptr !=NULL	&& list_ptr->address != reg)
	{
		list_ptr = list_ptr->next;
	}

	if(list_ptr == NULL)
	{
		i2c2_set_send_data(NULL, 0);
		return ;
	}

	len = (offset > list_ptr->len) ? 0 : (list_ptr->len - offset);

	if(list_ptr->bit == 16)
	{
		num = list_ptr->len / 2;
		for(uint8_t i = 0; i < num; i++)
		{
			list_ptr->i2c_buff[2*i] = list_ptr->user_buff[i*2 + 1];
			list_ptr->i2c_buff[2*i + 1] = list_ptr->user_buff[i*2];
		}
	}
	else if(list_ptr->bit == 32)
	{
		num = list_ptr->len / 4;
		for(uint8_t i = 0; i < num; i++)
		{
			list_ptr->i2c_buff[4*i] = list_ptr->user_buff[4*i + 3];
			list_ptr->i2c_buff[4*i + 1] = list_ptr->user_buff[4*i + 2];
			list_ptr->i2c_buff[4*i + 2] = list_ptr->user_buff[4*i + 1];
			list_ptr->i2c_buff[4*i + 3] = list_ptr->user_buff[4*i];
		}
	} 
	else
	{
		memcpy(list_ptr->i2c_buff, list_ptr->user_buff, list_ptr->len);
	}
	
	i2c2_set_send_data(&list_ptr->i2c_buff[offset], len);
}

static void I2CWrite(uint8_t reg, uint8_t* data, uint8_t len)
{
	I2CReg_t* list_ptr = reg_list;
	uint8_t offset = reg & 0x0f;
	uint8_t num = 0;
	reg = reg & 0xf0;
	
	while(list_ptr !=NULL	&& list_ptr->address != reg)
	{
		list_ptr = list_ptr->next;
	}

	if(list_ptr == NULL || list_ptr->len < offset)
	{
		return ;
	}

	len = ((list_ptr->len - offset) < len) ? (list_ptr->len - len) : len;

	memcpy(&list_ptr->i2c_buff[offset], data, len);
	
	if(list_ptr->bit == 16)
	{
		num = list_ptr->len / 2;
		for(uint8_t i = 0; i < num; i++)
		{
			*(uint16_t *)&list_ptr->user_buff[i*2] = (list_ptr->i2c_buff[2*i] << 8) | list_ptr->i2c_buff[2*i + 1];
		}
	}
	else if(list_ptr->bit == 32)
	{
		num = list_ptr->len / 4;
		for(uint8_t i = 0; i < num; i++)
		{
			*(uint32_t *)&list_ptr->user_buff[i*4] = (list_ptr->i2c_buff[4*i] << 24) | 
																							 (list_ptr->i2c_buff[4*i + 1] << 16) |
																							 (list_ptr->i2c_buff[4*i + 2] << 8) |
																							 (list_ptr->i2c_buff[4*i + 3]);
		}
	}
	else
	{
		memcpy(list_ptr->user_buff, list_ptr->i2c_buff, list_ptr->len);
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (hi2c->Instance == hi2c2.Instance) {
		hi2c->State = HAL_I2C_STATE_READY;
		i2c2_addr_req_callback(TransferDirection);
		if (TransferDirection == I2C_WRITE_OPERATION) {
			HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t *)rx_buffer, I2C_RECEIVE_BUFFER_LEN);
		}
		else {
			HAL_I2C_Slave_Transmit_IT(hi2c, (uint8_t *)tx_buffer, tx_len);
      tx_state = 1;
		}
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// read finish will callback
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {					//���ص�
	if (hi2c->Instance == hi2c2.Instance) {

    if (tx_state != 1) {
      i2c2_receive_callback((uint8_t *)&rx_buffer[0], I2C_RECEIVE_BUFFER_LEN - hi2c->XferSize);
    }
    tx_state = 0;
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// write finish will callback
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {						//д�ص�
	if (hi2c->Instance == hi2c2.Instance) {
		i2c2_receive_callback((uint8_t *)&rx_buffer[0], I2C_RECEIVE_BUFFER_LEN);
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// write finish will callback
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {
    tx_state = 0;
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == hi2c2.Instance) {
		HAL_I2C_EnableListen_IT(&hi2c2);
		__HAL_I2C_GENERATE_NACK(&hi2c2);
	}
}

