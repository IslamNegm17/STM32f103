/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>
#include <math.h>
#include "../mbedtls/mbedtls/sha256.h"
#include "../mbedtls/mbedtls/platform.h"
//#include "../RCC/rcc_init.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void hashMessage(const uint8_t *message, size_t message_len, uint8_t *output_hash){
    mbedtls_sha256_context sha256_ctx;
    mbedtls_sha256_init(&sha256_ctx);
    mbedtls_sha256_starts(&sha256_ctx, 0); // 0 for SHA-256
    mbedtls_sha256_update(&sha256_ctx, message, message_len);
    mbedtls_sha256_finish(&sha256_ctx, output_hash);
    mbedtls_sha256_free(&sha256_ctx);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Register Description */
/* ESP32 Public Key */
#define RSA_ESP_N                   187
#define RSA_ESP_E                   7
/* STM32 Crypto Key */
#define RSA_STM_P                   19
#define RSA_STM_Q                   13
#define RSA_STM_N                   247
#define RSA_STM_EULER               216
#define RSA_STM_E                   11
#define RSA_STM_D                   59
/* FLASH Control Registrs */
#define FLASH_ACR		                 (*((volatile uint32_t*)  0x40022000))
#define FLASH_KEYR		               (*((volatile uint32_t*)  0x40022004))
#define FLASH_OPTKEYR	               (*((volatile uint32_t*)  0x40022008))
#define FLASH_SR		                 (*((volatile uint32_t*)  0x4002200C))
#define FLASH_CR		                 (*((volatile uint32_t*)  0x40022010))
#define FLASH_AR		                 (*((volatile uint32_t*)  0x40022014))
#define FLASH_OBR		                 (*((volatile uint32_t*)  0x4002201C))
#define FLASH_WRPR		               (*((volatile uint32_t*)  0x40022020))
/* Option Bytes Description */
#define FLASH_OPTRDPADRR			       (*((volatile uint16_t*)  0x1FFFF800))
#define FLASH_OPTUSERADDR			       (*((volatile uint16_t*)  0x1FFFF802))
#define FLASH_OPTDATA0ADDR		       (*((volatile uint16_t*)  0x1FFFF804))
#define FLASH_OPTDATA1ADDR		       (*((volatile uint16_t*)  0x1FFFF806))
#define FLASH_OPTWRP0ADDR			       (*((volatile uint16_t*)  0x1FFFF808))
#define FLASH_OPTWRP1ADDR			       (*((volatile uint16_t*)  0x1FFFF80A))
#define FLASH_OPTWRP2ADDR			       (*((volatile uint16_t*)  0x1FFFF80C))
#define FLASH_OPTWRP3ADDR			       (*((volatile uint16_t*)  0x1FFFF80E))
/* ACR Register Bits */
#define LATENCY		                   0
#define HLFCYA		                   3
#define PRFTBE		                   4
#define PRFTBS		                   5
/* SR Register Bits*/
#define BSY			                     0
#define ERLYBSY		                   1
#define PGERR		                     2
#define WRPRTERR	                   4
#define EOP			                     5
/* CR Register Bits */
#define PG			                     0
#define PER			                     1
#define MER			                     2
#define OPTPG		                     4
#define OPTER		                     5
#define STRT                         6
#define LOCK                         7
#define OPTWRE	                     9
#define ERRIE		                     10
#define EOPIE		                     12
/* OBR Register Bits */
#define OPTERR	  	                 0
#define RDPRT		                     1
#define WDG_SW		                   2
#define NRST_STOP	                   3
#define NRST_STDBY	                 4
#define DATA0		                     10
#define DATA1		                     18
/* Bits Status Definitions */
#define FLASH_uint8_tLOCKED		       1
#define FLASH_uint8_tBSY			       1
/* Flash Base Address */
#define FLASH_uint32_tBASEADDRESS		0x08000000
#define FLASH_uint16_tPAGESIZE			0x0400
/* Flash Unlock Key Definitions */
#define FLASH_uint32_tKEY1				  0x45670123
#define FLASH_uint32_tKEY2				  0xCDEF89AB
/* Half Word Definitions */
#define HALFWORDMASK				        0x0000FFFF
#define HALFWORDOFFSET			        16
#define	BYTEMASK						        0x000000FF
/* Flash Read Protection */
#define FLASH_uint8_tREADUNPROTECTED		0
#define FLASH_uint8_tREADPROTECTED		  1
#define FLASH_uint16_tREADPROTECTKEY		0x00A5
/* Option Data Bytes */ 
#define FLASH_OPT_DATA0	         	      0
#define FLASH_OPT_DATA1                 1
/* Bootloader */
#define BOOTLOADER_START_ADDR  					0x08000000U
/* Application 1 */
#define APP1_START_ADDR        					0x08002800U
/* Application 2 */
#define APP2_START_ADDR         				0x08009400U
#define FLASH_OPTDATA0ADDR			        (*((volatile uint16_t*)  0x1FFFF804))
#define FLASH_OPTDATA1ADDR			        (*((volatile uint16_t*)  0x1FFFF806))
#define	BYTEMASK				    		        0x000000FF	
#define SIT_BIT(Reg,BitNo)           Reg|=(1<<(BitNo))
#define CLR_BIT(Reg,BitNo)           Reg&=~(1<<(BitNo))
#define GET_BIT(Reg,BitNo)           ((Reg & (1<<(BitNo)))>>(BitNo))

/*********** RCC_CSR Register Bits *******************/
#define LSION			  0
#define LSIRDY			1
#define RMVF			  24
#define PINRSTF			26
#define PORRSTF			27
#define SFTRSTF			28
#define IWDGRSTF		29
#define WWDGRSTF		30
#define LPWRSTF			31
/* Reset Reasons */
#define RCC_u8PINRESET			0   /* reset from the NRST pin occurs. */
#define RCC_u8PORRESET			1   /* POR/PDR reset occurs. */
#define RCC_u8SOFTRESET			2   /* software reset occurs. */
#define RCC_u8IWDGRESET			3   /* independent watchdog reset from VDD domain occurs. */
#define RCC_u8WWDGRESET			4   /* window watchdog reset occurs. */
#define RCC_u8LOWPOWERRESET	5   /* Low-power management reset occurs. */
#define RCC_u8POWERRESET		6   /**/
#define RCC_CSR			(*((volatile uint32_t*)  0x40021024))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void RCC_vidResetResetFlags(void);
uint8_t RCC_uint8_tGetResetFlag (uint8_t uint8_tResetFlag);
unsigned long long modExp(unsigned long long base, unsigned long long exp, unsigned long long mod);
unsigned long long RSA_Encryption(unsigned long long Org_Mess);
unsigned long long RSA_Decryption(unsigned long long Enc_Mess);
char* RSA_Encryption_Message (const char* OrgMess, uint32_t MessLength);
char* RSA_Decryption_Message (const char* DecMess, uint32_t MessLength);

/* USER CODE BEGIN PFP */
bool compare_hashes (const uint8_t hash1[32], const uint8_t hash2[32]){
    for (int i = 0; i < 32; i++) {
        if (hash1[i] != hash2[i]) {
            return false;
        }
    }
    return true;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef rxHeader;          
uint8_t canRX[8];                       
volatile uint8_t indecator_Var;  
volatile uint8_t BootloaderKey = 0x7F; 
volatile uint8_t ACK  = 0xCD;
volatile uint8_t NACK = 0xAB;
uint8_t MessagetoLogic [] = {"Hello Message to Logic Analyzer\n\r"};
uint8_t MessagetoTT    [] = {"Cause of Reset occured\n\r"};
uint8_t debug1         [] = {0x01, 0x02, 0x03, 0x4, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0F, 0xF, 0xFF};
uint8_t debug2         [] = {"RESET\n\r"};
uint8_t BL_Buffer      [50];
uint8_t Buff           [2];
bool cond = false;
uint8_t value = 0xFF;
uint8_t Buffer = 0;
volatile uint8_t uint8_tResetReason = 0;

uint8_t Encrypted_Message_length = 0;
uint8_t Hash_Message_length = 0;
char length_str[10];
uint8_t Encrypted_Message [15];
uint8_t ReceivedHash      [32];
uint8_t HashEncMass       [32];
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   const uint8_t message[] = "HelloSTM32";
	 size_t message_len = sizeof(message);
	 uint8_t hash[32];
   
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
  //MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_CAN_Start(&hcan);  
  //HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt
	//hashMessage(message, message_len, hash);
	//HAL_UART_Transmit(&huart2, hash, 32, HAL_MAX_DELAY);
	while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		CAN_ReceiveUptoEightBytes(&hcan, CAN_RX_FIFO0, &indecator_Var, NULL);
		if (BootloaderKey == indecator_Var){ 		 
		  //BL_UART_Fetch_Host_Command();
			/* Receive Length and Encrypted Message */
			CAN_ReceiveUptoEightBytes(&hcan, CAN_RX_FIFO0, &Encrypted_Message_length, NULL);
			CAN_ReceiveNumberOfBytes(&hcan, CAN_RX_FIFO0, Encrypted_Message, Encrypted_Message_length);
			/* Receive Length and Hashing */		
			CAN_ReceiveUptoEightBytes(&hcan, CAN_RX_FIFO0, &Hash_Message_length, NULL);
			CAN_ReceiveNumberOfBytes(&hcan, CAN_RX_FIFO0, ReceivedHash, Hash_Message_length);
			
			/* Display Encrypted Message */
			HAL_UART_Transmit(&huart2, "Length of Encrypted Message: ", sizeof("Length of Encrypted Message: "), HAL_MAX_DELAY);
			sprintf(length_str, "%u", Encrypted_Message_length);
			for(int i = 0; length_str[i] != '\0'; i++) {
        HAL_UART_Transmit(&huart2, (uint8_t *)&length_str[i], 1, HAL_MAX_DELAY);
      }
      HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);					
			HAL_UART_Transmit(&huart2, "Encrypted Message: ", sizeof("Encrypted Message: "), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, Encrypted_Message, Encrypted_Message_length, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);
			
      /* Display Hash Message*/			
			HAL_UART_Transmit(&huart2, "Length of Hashing Message: ", sizeof("Length of Hashing Message: "), HAL_MAX_DELAY);
			sprintf(length_str, "%u", Hash_Message_length);
			for(int i = 0; length_str[i] != '\0'; i++) {
        HAL_UART_Transmit(&huart2, (uint8_t *)&length_str[i], 1, HAL_MAX_DELAY);
      }
      HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, "Hashing: ", sizeof("Hashing: "), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, ReceivedHash, Hash_Message_length, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);
			
			/* Decrypt Message */
			char *DecryptedMessage = RSA_Decryption_Message(Encrypted_Message, Encrypted_Message_length);
			HAL_UART_Transmit(&huart2, "Decrypted Message: ", sizeof("Decrypted Message: "), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, DecryptedMessage, Encrypted_Message_length, HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);
			
			/* Hash the Decrypted Message*/
      hashMessage((uint8_t*)DecryptedMessage, Encrypted_Message_length, HashEncMass);
			HAL_UART_Transmit(&huart2, "Hash Message: ", sizeof("Hash Message: "), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, HashEncMass, 32, HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);
			
			/* Verification */
       if (compare_hashes(ReceivedHash, HashEncMass)){
          HAL_UART_Transmit(&huart2, "Verified\n\r", sizeof("Verified\n\r"), HAL_MAX_DELAY);
       } 
			 else {
          HAL_UART_Transmit(&huart2, "Failed\n\r", sizeof("Failed\n\r"), HAL_MAX_DELAY);
       }			
			//char decryptedMess = RSA_Decryption(indecator_Var);
		  //HAL_UART_Transmit(&huart2, &decryptedMess, 1, HAL_MAX_DELAY);
		  //HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	HAL_UART_Transmit(&huart2, canRX, sizeof(canRX), HAL_MAX_DELAY);
}


uint8_t RCC_uint8_tGetResetFlag (uint8_t uint8_tResetFlag)
{
	uint8_t uint8_tFlags = ((RCC_CSR & (0xFFFF << PINRSTF)) >> PINRSTF);
	uint8_t uint8_tReturnValue = 0;
	
	if (uint8_tResetFlag == RCC_u8POWERRESET)
	{
		if (uint8_tFlags == 3)
		{
			uint8_tReturnValue = 1;
		}
	}
	else if ( (uint8_tFlags & (1<<uint8_tResetFlag) ) >>uint8_tResetFlag == 1)
	{
		uint8_tReturnValue = 1;
	}
	return uint8_tReturnValue;
}

void RCC_vidResetResetFlags(void)
{
	RCC_CSR |= (1 << RMVF);
}

void RTC_IRQHandler (void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}


unsigned long long modExp(unsigned long long base, unsigned long long exp, unsigned long long mod) {
    unsigned long long result = 1;
    base = base % mod;
    while (exp > 0) {
        if (exp % 2 == 1) {
            result = (result * base) % mod;
        }
        exp = exp >> 1;
        base = (base * base) % mod;
    }
    return result;
}

unsigned long long RSA_Encryption(unsigned long long Org_Mess) {
    unsigned long long Encrypted_Mess = modExp(Org_Mess, RSA_ESP_E, RSA_ESP_N);
    return Encrypted_Mess;
}

unsigned long long RSA_Decryption(unsigned long long Enc_Mess) {
    unsigned long long Original_Mess = modExp(Enc_Mess, RSA_STM_D, RSA_STM_N);
    return Original_Mess;
}

char* RSA_Encryption_Message (const char* OrgMess, uint32_t MessLength){
   char* Buffer = (char*)malloc(MessLength);
    if (Buffer == NULL) {
        return NULL;
    }
   for (uint32_t itr = 0; itr < MessLength - 1; itr++){
     Buffer[ itr ] = RSA_Encryption(OrgMess[ itr ]);
   }
   return Buffer;
}

char* RSA_Decryption_Message (const char* DecMess, uint32_t MessLength){
   char* Buffer = (char*)malloc(MessLength);
    if (Buffer == NULL) {
        return NULL;
    }
   for (uint32_t itr = 0; itr < MessLength - 1; itr++){
     Buffer[ itr ] = RSA_Decryption(DecMess[ itr ]);
   }
   return Buffer;
}
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
