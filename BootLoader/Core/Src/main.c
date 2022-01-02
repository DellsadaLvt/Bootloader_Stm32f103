/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN
#define USER_BASE_FLASH 					((uint32_t)0x8003C00)
#define MANUFACTOR_BASE_FLASH			((uint32_t)0x08000000)
#define MANUFACTOR_FLASH_SIZE			((uint32_t)(64u*1024u))
#define MANUFACTOR_END_FLASH			(MANUFACTOR_BASE_FLASH + MANUFACTOR_FLASH_SIZE)
#define MANUFACTOR_BASE_RAM				((uint32_t)0x20000000)
#define MANUFACTOR_RAM_SIZE				((uint32_t)(20u*1024u))
#define MANUFACTOR_END_RAM				(MANUFACTOR_BASE_RAM + MANUFACTOR_RAM_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */	

/* Bootloader command */
#define BL_GET_VER									((uint8_t)0x51)
#define BL_GET_HELP 								((uint8_t)0x52)
#define BL_GET_CID									((uint8_t)0x53)
#define BL_GET_RDP_STATUS						((uint8_t)0x54)
#define BL_GO_TO_ADDR 							((uint8_t)0x55)
#define BL_FLASH_ERASE							((uint8_t)0x56)
#define BL_MEM_WRITE 								((uint8_t)0x57)
#define BL_ENDIS_RW_PROTECT					((uint8_t)0x58
#define BL_MEM_READ 								((uint8_t)0x59)
#define BL_READ_SECTOR_STATUS				((uint8_t)0x5A)
#define BL_OTP_READ 								((uint8_t)0x5B)
static const uint8_t g_u8_supported_commands[]= {
		BL_GET_VER,
		BL_GET_HELP,
		BL_GET_CID,
		BL_GET_RDP_STATUS,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WRITE,
		BL_READ_SECTOR_STATUS
};
/* ack and non_ack byte */
#define ACK													((uint8_t)0xA5)
#define NACK												((uint8_t)0x7F)

/* Bootloader version */
#define BOOTLOADER_VER							((uint8_t)1U)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t UART2_Transmit(const uint8_t *const buff, const uint8_t len);
static uint32_t get_tick( void );
static uint8_t led_status_uart_transmit(void);
static uint8_t user_delay( uint32_t time_delay );
static uint8_t printmsg(char *format, ...);
static void bootloader_uart_read_data(void);
static void bootloader_jump_to_user_app(const uint32_t user_base_flash);
static uint8_t UART2_Receive(uint8_t *buff, uint8_t len);
/* Bootloader command */
static uint8_t bootloader_handle(uint8_t *rx_buffer, uint8_t (*ptrFunc)(uint8_t *buff));
static uint8_t bootloader_handle_getver_cmd(uint8_t *buff);
static uint8_t bootloader_handle_gethelp_cmd(uint8_t *buff);
static uint8_t bootloader_handle_getcid_cmd(uint8_t *buff);
static uint8_t bootloader_handle_getrdp_cmd(uint8_t *buff);
static uint8_t bootloader_handle_go_cmd(uint8_t *rx_buffer);
static uint8_t bootloader_handle_flash_erase_cmd(uint8_t *rx_buffer);
static uint8_t bootloader_handle_mem_write_cmd(uint8_t *rx_buffer);
//static uint8_t bootloader_handle_endis_rw_protect(uint8_t *rx_buffer);
//static uint8_t bootloader_handle_mem_read(uint8_t *rx_buffer);
//static uint8_t bootloader_handle_read_sector_status(uint8_t *rx_buffer);
//static uint8_t bootloader_handle_read_otp(uint8_t *rx_buffer);
static uint8_t bootloader_send_ack(const uint8_t len_follow);
static uint8_t bootloader_send_nack(void);
static uint8_t crc_reset(void);
static uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);
static uint8_t bootloader_uart_write_data(uint8_t *pData, uint8_t len);
static uint8_t verify_address(uint32_t address);
static uint8_t flash_earse(uint8_t page_no, uint8_t num_page);
static uint8_t execute_mem_write(const uint32_t address_base,const uint8_t *const payload,const uint8_t payload_len);
static uint8_t execute_mem_write(uint32_t address_base,const uint8_t *payload,const uint8_t payload_len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if( 0u == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
			bootloader_uart_read_data();
		}
		else{
			bootloader_jump_to_user_app(USER_BASE_FLASH);
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*************** This is the mask functions ****************/
static uint8_t UART2_Transmit(const uint8_t *const buff, const uint8_t len){
	if(HAL_OK == HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, HAL_MAX_DELAY))
		return 0u;
	
	return 1u;
}

static uint8_t UART2_Receive(uint8_t *buff, uint8_t len){
	if( HAL_OK == HAL_UART_Receive(&huart2, buff, len, HAL_MAX_DELAY))
		return 0u;
	
	return 1u;
}


static uint8_t bootloader_uart_write_data(uint8_t *pData, uint8_t len){
	UART2_Transmit(pData, len);
	
	return 0u;
}


static uint32_t get_tick( void ){
	uint32_t buff = HAL_GetTick();
	
	return buff;
}

static uint8_t user_delay( uint32_t time_delay ){
	HAL_Delay(time_delay);
	
	return 0u;
}

static uint8_t flash_earse(uint8_t page_no, uint8_t num_page){
	FLASH_EraseInitTypeDef flash_earse_init= {0};
	if( page_no == 0x65){
		flash_earse_init.TypeErase = FLASH_TYPEERASE_MASSERASE;		
	}
	else{
		uint8_t remain_page= 64u - page_no;
		if(num_page > remain_page)
			num_page = remain_page;
		
		flash_earse_init.TypeErase = FLASH_TYPEERASE_PAGES;
		flash_earse_init.PageAddress= MANUFACTOR_BASE_FLASH + (page_no*1024u);
		flash_earse_init.NbPages= num_page;	
	}
	
	HAL_FLASH_Unlock();
	uint32_t page_err= 0u;
	uint8_t status = (uint8_t)HAL_FLASHEx_Erase(&flash_earse_init, &page_err);
	HAL_FLASH_Lock();
	
	return status;
}

/*************** Generate functions ***************/
static uint8_t led_status_uart_transmit(void){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	user_delay(50u);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	user_delay(50u);
	
	return 0u;
}


static uint8_t printmsg(char *format, ...){
#ifdef BL_DEBUG_MSG_EN
	char str[80u];
	
	/* Extract the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, (const char*)format, args);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
	
#endif
	
	return 0u;
}


static uint8_t verify_address(uint32_t address){
	/* accept flash memory */
	if( (address >= MANUFACTOR_BASE_FLASH) && (address < MANUFACTOR_END_FLASH))
		return 0u;
	else if( (address >= MANUFACTOR_BASE_RAM) && (address < MANUFACTOR_END_RAM))
		return 0u;
	
	return 1u;
}


static uint8_t execute_mem_write(uint32_t address_base,const uint8_t *payload,const uint8_t payload_len){
	if( address_base %2u == 1u)
		address_base += 1u;
	if(0u == verify_address(address_base)){
		int16_t temp_payload_len = payload_len;
//		uint8_t page_no = (address_base - MANUFACTOR_BASE_FLASH)/1024u;
//		uint8_t num_page = 1u + (payload_len/1024u);
//		flash_earse(page_no, num_page);
		
		HAL_FLASH_Unlock();
		while(temp_payload_len > 0){
			/* Send 8bytes */
			if( (0u == address_base%8u) && (0u == (uint32_t)payload%8u) && (temp_payload_len >= 8u) ){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address_base, *(uint64_t*)payload);
				temp_payload_len -= 8u;
				address_base += 8u;
				payload += 8u;
			}
			/* Send 4bytes */
			else if( (0u == address_base%4u) && (0u == (uint32_t)payload%4u) && (temp_payload_len >= 4u) ){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address_base, *(uint32_t*)payload);
				temp_payload_len -= 4u;
				address_base += 4u;
				payload += 4u;
			}
			/* Send 2bytes */
			else{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address_base, *(uint16_t*)payload);
				temp_payload_len -= 2u;
				address_base += 2u;
				payload += 2u;
			}
		}
		HAL_FLASH_Lock();
		return 0u;
	}
	
	return 1u;
}
/********** Functions bootloader mode *********/
static void bootloader_uart_read_data(void){
	while(1){
		uint8_t rx_buffer[200u];
		memset(rx_buffer, 0u, 200u);
		
		/* receive the first data into rx_buffer[0] to know the length of data */
		UART2_Receive(rx_buffer, 1u);
		/* receive the remain data with rx_buffer[0] length */
		UART2_Receive((rx_buffer+1u), rx_buffer[0]);
		
		switch(rx_buffer[1u]){
			case BL_GET_VER:
				bootloader_handle(rx_buffer, bootloader_handle_getver_cmd);
				break;
			
			case BL_GET_HELP:
				bootloader_handle(rx_buffer, bootloader_handle_gethelp_cmd);
				break;
			
			case BL_GET_CID:
				bootloader_handle(rx_buffer, bootloader_handle_getcid_cmd);
				break;
			
			case BL_GET_RDP_STATUS:
				bootloader_handle(rx_buffer, bootloader_handle_getrdp_cmd);
				break;
			
			case BL_GO_TO_ADDR:
				bootloader_handle(rx_buffer, bootloader_handle_go_cmd);
				break;
			
			case BL_FLASH_ERASE:
				bootloader_handle(rx_buffer, bootloader_handle_flash_erase_cmd);
				break;
			
			case BL_MEM_WRITE:
				bootloader_handle(rx_buffer, bootloader_handle_mem_write_cmd);
				break;
			
			default:
				break;
		}
		
	}
	
}

static void bootloader_jump_to_user_app(const uint32_t user_base_flash){
	void (*ptr_reset_handler)(void);
	
	/* set top of stack */
	__set_MSP(*(uint32_t*)user_base_flash);
	
	/* jump to reset handler */
	ptr_reset_handler = (void*)(*(uint32_t*)(user_base_flash + 4u));
	ptr_reset_handler();

}
/********** Functions CRC checking *********/
static uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host){
	uint32_t uwCRCValue = crc_host - 1u;
	
	for(uint8_t i= 0u; i< len; i += 1u){
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1u);
	}
	
	if( uwCRCValue == crc_host ){
		crc_reset();
		return 0u;
	}
	
	return 1u;
}

static uint8_t crc_reset(void){
	CRC->CR = 1U;
	
	return 0u;
}

/********** Functions using send ack when check crc *********/
static uint8_t bootloader_send_ack(const uint8_t len_follow){
	uint8_t buff[2u];
	buff[0u] = ACK;
	buff[1u] = len_follow;
	UART2_Transmit(buff, 2u);
	
	return 0u;
}

static uint8_t bootloader_send_nack(void){
	uint8_t buff = NACK;
	UART2_Transmit(&buff, 1u);
	
	return 0u;
}


/********** Bootloader command *********/
static uint8_t bootloader_handle(uint8_t *rx_buffer, uint8_t (*ptrFunc)(uint8_t *buff)){
	/* Total length of message */
	uint32_t command_packet_len= rx_buffer[0u] + 1u; // 1 is the 1byte of length to follow
	
	/* Get host crc from host */
	uint32_t host_crc = *((uint32_t*)(rx_buffer + command_packet_len - 4u)); // crc is the last 4bits of the packet.
	
	if(0u == bootloader_verify_crc(rx_buffer, command_packet_len- 4u, host_crc)){
		ptrFunc(rx_buffer);
	}
	else{
		bootloader_send_nack();
	}
		
	return 0;
}

static uint8_t bootloader_handle_getver_cmd(uint8_t *buff){
	bootloader_send_ack(1u);
	uint8_t bl_version = BOOTLOADER_VER;
	bootloader_uart_write_data(&bl_version, 1u);
	
	return 0;
}

static uint8_t bootloader_handle_gethelp_cmd(uint8_t *buff){
	bootloader_send_ack(sizeof(g_u8_supported_commands));
	bootloader_uart_write_data((uint8_t*)g_u8_supported_commands, sizeof(g_u8_supported_commands));
	
	return 0;
}

static uint8_t bootloader_handle_getcid_cmd(uint8_t *buff){
	bootloader_send_ack(2u);
	
	uint16_t cid = (uint16_t)(DBGMCU->IDCODE)&0xFFF;
	bootloader_uart_write_data((uint8_t*)&cid, 2u);
	
	return 0;
}

static uint8_t bootloader_handle_getrdp_cmd(uint8_t *buff){
	uint8_t rdp_status = 0u;
#if 1
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else
	volatile uint32_t *pOB_add = (uint32_t*)0x1FFFC000;
	rdp_status = (uint8_t)(*pOB_add >> 8u);
	
#endif
	
	bootloader_send_ack(1u);
	bootloader_uart_write_data(&rdp_status, 1u);
	
	return 0;	
}

static uint8_t bootloader_handle_go_cmd(uint8_t *rx_buffer){
	/*Helper function to handle BL_GO_TO_ADDR command */
    uint8_t addr_valid = 0u;
    uint8_t addr_invalid = 1u;
	
    bootloader_send_ack(1u);
		//extract the go address
		uint32_t go_address = *(uint32_t*)(*(uint32_t*)(rx_buffer+2u));
		if( verify_address(go_address) == 0u )
		{
			//tell host that address is fine
			bootloader_uart_write_data(&addr_valid,1);

			/*jump to "go" address.
			we dont care what is being done there.
			host must ensure that valid code is present over there
			Its not the duty of bootloader. so just trust and jump */

			/* Not doing the below line will result in hardfault exception for ARM cortex M */
			//watch : https://www.youtube.com/watch?v=VX_12SjnNhY

			//go_address+=1; //make T bit =1
			void (*lets_jump)(void) = (void *)go_address;
				/* set top of stack */
			__set_MSP(*(uint32_t*)USER_BASE_FLASH);
			lets_jump();

		}
		else{
			//tell host that address is invalid
			bootloader_uart_write_data(&addr_invalid,1);
		}

		return 0;
}

static uint8_t bootloader_handle_flash_erase_cmd(uint8_t *rx_buffer){
	bootloader_send_ack(1u);
	uint8_t earse_status = flash_earse(rx_buffer[2u], rx_buffer[3u]);
	bootloader_uart_write_data(&earse_status, 1u);
	
	return 0;
}

static uint8_t bootloader_handle_mem_write_cmd(uint8_t *rx_buffer){
	bootloader_send_ack(1u);
	
	volatile uint8_t status= 1u;
	uint8_t payload_len= rx_buffer[6u];
	uint8_t *start_payload = rx_buffer+7u;
	uint32_t address_base = *(uint32_t*)(rx_buffer+2u);
	
	if( 0u == verify_address(address_base)){
		status = execute_mem_write(address_base, start_payload, payload_len);
		bootloader_uart_write_data((uint8_t*)&status, 1u);
	}
	else{
		bootloader_uart_write_data((uint8_t*)&status, 1u);
	}
	
	
	return 0;
}

//static uint8_t bootloader_handle_endis_rw_protect(uint8_t *rx_buffer){

//	
//	return 0;
//}

//static uint8_t bootloader_handle_mem_read(uint8_t *rx_buffer){

//	
//	return 0;
//}

//static uint8_t bootloader_handle_read_sector_status(uint8_t *rx_buffer){

//	
//	return 0;
//}

//static uint8_t bootloader_handle_read_otp(uint8_t *rx_buffer){

//	
//	return 0;
//}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
