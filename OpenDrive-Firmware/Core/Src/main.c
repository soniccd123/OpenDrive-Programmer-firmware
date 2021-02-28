/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Control States
#define nullState_mode		0
#define readBuffer_mode		10
#define readAddr_mode		12
#define writeFlash_mode		25
#define	writeFeram_mode		22
#define writeFlashAddr_mode	23
#define erase_mode			30
#define resetFlash_mode		41
#define	readSR_mode			42
#define	clearSR_mode		43

//Control Pins
#define CE_Pin				GPIO_PIN_2
#define ADDR_EN_Pin			GPIO_PIN_14
#define ADDR_2AAA_Pin		GPIO_PIN_1
#define ADDR_5555_Pin		GPIO_PIN_0
#define TIME_Pin			GPIO_PIN_15

#define FERAM_ADDR			0x100000
#define	flash				0
#define feram				1
#define set					1
#define reset				0
#define readbuffer_size		512
#define port_read			0x88888888
#define port_write			0x33333333
#define ENABLE				0
#define DISABLE				1
#define sram_ce				0xFF
#define flash_ce			0x00
#define SR_ready			0x80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
int mode_control = 0;
int next_block = 0;
uint8_t mem_buffer[512];
uint8_t write_buffer[256];
int ready = 0;
int cont = 0;
int buffer_addr;
int buffer_write_addr;
int memory_mode;
uint32_t addr_write;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void ready_led(void)	//Alternates leds, used to indicate successful initialization to user
{
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1);
}

void reset_addr()					//Resets address counter chips
{
	HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1);		//Simulates needed waveform
	HAL_GPIO_WritePin(GPIOB, RST_Pin, 1); 		//for successfully reseting
	HAL_GPIO_WritePin(GPIOB, RST_Pin, 0); 		//address counter
	HAL_GPIO_WritePin(GPIOB, RST_Pin, 1); 		//(74HC590)
	HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);		//
}

void goto_addr(uint32_t addr)		//Makes address counter chips
{									//count to a desired address
	uint32_t loop;
	reset_addr();	//Reset address counter
	while (loop < addr)		//Set a loop to count to a desired number (addr)
	{
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1);	//Strobe 74HC590 count pin
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);	//
		loop=loop+1;
	}
}

void write(uint8_t low, uint8_t high)		//Write a word to the Flash with a previously set address
{
	GPIOA->ODR = low;	//Set GPIOA with low part of the word (LSB)
	uint16_t gpiob_odr = GPIOB->ODR & 0xFF;		//Bit Mask low part of GPIOB (for preserving pins status)
	gpiob_odr|= (high) << 8;	//Combines high part of the word with masked bits
	GPIOB->ODR = gpiob_odr;		//Set GPIOB with high part of the word and masked bits

	HAL_GPIO_WritePin(GPIOB, WE_Pin, 0); //Strobe WE pin to write word to flash
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //

}

void write_feram(uint8_t low, uint8_t high)	//Write a word to the Flash with a previously set address
{
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 0); 	//Enable WE and CE pins (active low)
	delay_us(5);							//in sequence as FM1808 requires for
	HAL_GPIO_WritePin(GPIOB, CE_Pin, 0);	//a CE controlled write. Address is
	delay_us(5);							//latched on CE falling edge

	GPIOA->ODR = low; //Set GPIOA with low part of the word (LSB)
	uint16_t gpiob_odr = GPIOB->ODR & 0xFF;		//Bit Mask low part of GPIOB (for preserving pins status)
	gpiob_odr|= (high) << 8;	//Combines high part of the word (MSB) with masked bits
	GPIOB->ODR = gpiob_odr;		//Set GPIOB with high part of the word and masked bits


	HAL_GPIO_WritePin(GPIOB, CE_Pin, 1); 	//Disable CE and WE pins in sequence
	delay_us(5);							//as required by FM1808. Data is
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); 	//latched in CE rising edge.
	delay_us(5);							//

}

void command_5555 (uint8_t cmd)		//Write a command to address 0x5555 of the flash
{
	HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 1); 		//Disable 74HC590 Address counter output
	HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 1); 	//Set 0x5555 address in the flash
	HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); 	//Disable 0x2AAA address in the flash

	GPIOA->ODR = cmd; //Set GPIOA with low part of the word (LSB), in this function the command itself (byte command)
	uint16_t gpiob_odr = GPIOB->ODR & 0xFF;		//Bit Mask low part of GPIOB (for preserving pins status)
	gpiob_odr|= (0x00) << 8; 	//Combines high part of the word (MSB) with masked bits
	GPIOB->ODR = gpiob_odr; 	//Set GPIOB with high part of the word and masked bits

	HAL_GPIO_WritePin(GPIOB, WE_Pin, 0); //Strobe WE pin to write word to flash
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //
}

void command_2AAA (uint8_t cmd)		//Write a command to address 0x2AAA of the flash
{
	HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 1); 		//Disable 74HC590 Address counter output
	HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0); 	//Disable 0x5555 address in the flash
	HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 1); 	//Set 0x2AAA address in the flash

	GPIOA->ODR = cmd; //Set GPIOA with low part of the word (LSB), in this function the command itself (byte command)
	uint16_t gpiob_odr = GPIOB->ODR & 0xFF;		//Bit Mask low part of GPIOB (for preserving pins status)
	gpiob_odr|= (0x00) << 8;	//Combines high part of the word (MSB) with masked bits
	GPIOB->ODR = gpiob_odr;		//Set GPIOB with high part of the word and masked bits

	HAL_GPIO_WritePin(GPIOB, WE_Pin, 0); //Strobe WE pin to write word to flash
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //
}

void write_addr(uint8_t low, uint8_t high, int addr)
{
	HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0); //ADDR EN
	HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0); //5555 DIS
	HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); //2AAA DISABLE
	uint32_t loop = 0;
	reset_addr();
	while (loop < addr)
	{
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1); //ADDR INCR
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);
		loop=loop+1;
	}
	GPIOA->ODR = low;
	uint16_t gpiob_odr = GPIOB->ODR & 0xFF;
	gpiob_odr|= (high) << 8;
	GPIOB->ODR = gpiob_odr;
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 0); //SET READ
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //SET READ
}

void buffer_write()			//Write the 256bytes write buffer to the flash memory
{
	int buffer_read_addr = 0;
	while (buffer_read_addr < 256)	//Sets a loop to write the 256bytes of the buffer
	{
		write(write_buffer[buffer_read_addr+1],write_buffer[buffer_read_addr]); //Write a buffer's word with "write" function
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1); 		//Strobe the address increase pin in the
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);		//address counter and increases it by one
		buffer_read_addr = buffer_read_addr+2;		//Increment loop by 2 (1word = 2bytes written)
	}
}

void buffer_write_feram()	//Write the 256bytes write buffer to the FeRAM memory
{
	int buffer_read_addr = 0;
	while (buffer_read_addr < 256)	//Sets a loop to write the 256bytes of the buffer
	{
		write_feram(write_buffer[buffer_read_addr+1],write_buffer[buffer_read_addr]); //Write a buffer's word with "write_feram" function
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1); 	//Strobe the address increase pin in the
		HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);	//address counter and increases it by one
		buffer_read_addr = buffer_read_addr+2;	//Increment loop control variable by 2 (1word = 2bytes written)
	}
}

void polling()		//Wait till 0x80 (Write completed) is returned by the Flash SR
{
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); 	//Enable Flash data output
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); 	//Set programmer's bus transceivers to read direction

	uint8_t SR;		//Create local variable to read SR content
	GPIOA->CRL = port_read;		//Set GPIOA as input
	SR = GPIOA->IDR & 0x80;		//Read SR contents to SR variable
	while (SR != SR_ready)		//Set a loop that only stops when SR return 0x80 (Write complete)
	{
		SR = GPIOA->IDR & SR_ready;		//Read SR contents to SR variable
	}
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); 	//Disable Flash data output
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1); 	//Set programmer's bus transceivers to write direction
	GPIOA->CRL = port_write;	//Set GPIOA as output
}

void buffering()	//Fill a 256byte write buffer for faster batch write operation
{
	ready = reset;	//Reset "ready" global flag
	buffer_write_addr = 0;	//Reset buffer address

	int buffer_write_loop = 0;		//Set a buffer filling loop, four 64byte iterations
	while (buffer_write_loop < 4)	//totaling 256bytes. Done this way because STM32
	{								//USB buffer max at 64bytes

		next_block = set;					//Set "next_block" global flag signaling CDC receive function to store the next USB transfer

		mem_buffer[0] = 'n';				//Sends 'n' to host computer signaling that the programmer is ready to
		CDC_Transmit_FS(mem_buffer, 1);		//receive the next 64bytes of data

		while(ready != set){}				//Wait this "ready" global flag is set, signaling that all 64bytes have been stored in the write buffer
		ready = reset;						//Reset "ready" global flag for next transfer

		buffer_write_loop = buffer_write_loop+1; //Increases loop by 1
	}
}

void switch_to_sram()		//Signals cartridge to map the FeRAM to a visible address (0x100000 in Mega Drive and 32X games)
{
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 0);	//Time signal goes low in preparation for creating a rising edge when the correct command is at the data bus
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);	//WE and OE are disabled to not interfere in the data bus
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 1);	//
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1);	//Programmer data bus direction is set to the cartridge
	GPIOA->CRL = port_write;	//Set GPIOA data pins as output
	GPIOA->ODR = sram_ce;	//0xFF (mapper command to select FeRAM) is placed in the data bus
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 1);	//Time goes high and the mapper latches the command, starting to show the FeRAM at 0x10000
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 0);	//Time goes down in preparation for next pulses
	GPIOA->CRL = port_read;		//Set GPIOA as input
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0);	//Programmer data bus direction is set to the MCU
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 0);	//OE is enabled again

}

void switch_to_flash()		//Signals cartridge to map the entire Flash addresses and hide FeRAM
{
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 0);	//Time signal goes low in preparation for creating a rising edge when the correct command is at the data bus
	HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);	//WE and OE are disabled to not interfere in the data bus
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 1);	//
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1);	//Programmer data bus direction is set to the cartridge
	GPIOA->CRL = port_write;	//Set GPIOA as output
	GPIOA->ODR = flash_ce;	//0x00 (mapper command to select Flash) is placed in the data bus
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 1);	//Time goes high and the mapper latches the command, hiding the FeRAM
	HAL_GPIO_WritePin(GPIOC, TIME_Pin, 0);	//Time goes down in preparation for next pulses
	GPIOA->CRL = port_read;		//Set GPIOA as input
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0);	//Programmer data bus direction is set to the MCU
	HAL_GPIO_WritePin(GPIOB, OE_Pin, 0);	//OE is enabled again
}
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
	  MX_USB_DEVICE_Init();
	  MX_TIM1_Init();
	  /* USER CODE BEGIN 2 */
	  HAL_TIM_Base_Start(&htim1);
	  ready_led();
	  reset_addr();

	  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR WRITE
	  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET DIR WRITE
	  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //SET READ
	  HAL_GPIO_WritePin(GPIOC, TIME_Pin, 0); //TIME DISABLE
	  HAL_GPIO_WritePin(GPIOB, CE_Pin, 0); //CE EN
	  switch_to_flash();
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)	//nullState, nothing happens unless is said to. Waiting for orders mode
	    {

	  	  if (mode_control == readBuffer_mode) //Read Buffer mode
	  	  {
	  		  reset_addr();	//Reset address counter to 0
	  		  delay_us(3000);	//Not sure if necessary, probably not
	  		  if (memory_mode == flash) //Check memory mode
	  		  {
	  			  switch_to_flash(); //Map full Flash range to address space and hide FeRAM
	  		  }

	  		  if (memory_mode == feram) //Check memory mode
	  		  {
	  			  switch_to_sram();	//Map FeRAM to 0x100000
	  			  goto_addr(FERAM_ADDR);	//Set Address counter to FeRAM first address
	  		  }

	  		  GPIOA->CRL = port_read;	//Set GPIOA data lines as input
	  		  GPIOB->CRH = port_read;	//Set GPIOB data lines as input

	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);	//Disable WE as not a reading operation
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); 	//Enable memory output
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0);	//Set Programmer data bus toward the MCU
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0);	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); 	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0);	//Enable address counter output
	  		  HAL_GPIO_WritePin(GPIOB, CE_Pin, 1);	//Disable CE signal (because of FeRAM compatibility
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);	//Turn status LED on signaling the user of operation start

	  		  mem_buffer[0] = 'r';				//Send a 'r' character to the host
	  		  CDC_Transmit_FS(mem_buffer, 1);	//to signal that the firmware is ready to send data

	  		  int reading = set;	//Set a reading loop controlled by the reading var
	  		  while(reading == 1)	//
	  		  {
	  			  while(ready != 1){}	//Wait for ready global flag to set signaling that a transfer was received
	  			  ready = reset;	//Reset the ready global flag for the next iteration
	  			  if (mem_buffer[0] == 'n')	//Checks if the transfer received was a 'n' with is the host command to request the next 512byte block
	  			  {
	  				  int buffer_addr = 0;						//Set a read loop to fill a 512byte b
	  				  while (buffer_addr < readbuffer_size)		//contents to the host
	  				  {
	  					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); //Enable CE to latch the address for the FeRAM (Flash does not care)
	  					  delay_us(10);	//Not sure if necessary
	  					  mem_buffer[buffer_addr+1] = (GPIOA->IDR & 0x00FF);	//Read GPIOA input register to a buffer address. Does it through a 0x00FF bit mask to select only the data pins (0-7).
	  					  mem_buffer[buffer_addr] = (GPIOB->IDR & 0xFF00)>>8;	//Read GPIOB input register to a buffer address. Does it through a 0xFF00 bit mask to select only the data pins (8-15) and shift the bits left 8 times.
	  					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); //Disable CE to set it to the next iteration
	  					  buffer_addr = buffer_addr+2;	//Increment buffer_addr by two because two bytes were read

	  					  HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1); //Strobe address counter increase pin to increment it by 1
	  					  HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);


	  				  }
	  				  CDC_Transmit_FS(mem_buffer, readbuffer_size);	//Transmit the 512byte buffer to the host
	  			  }
	  			  else	//If the received char is not 'n' -> Terminate exection and go back to nullState
	  			  {
	  				  reading = reset;	//Reset reading to terminate the loop
	  				  ready = reset;	//Reset ready because why not
	  				  mode_control = nullState_mode;	//Return control mode to nullState
	  				  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1);	//Turn the led off to signal the user that the operation terminated
	  			  }
	  		  }
	  	  }

	  	  if (mode_control == readAddr_mode) //Read Address, Ignore for now
	  	  {
	  		  GPIOA->CRL = port_read;
	  		  GPIOB->CRH = port_read;


	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR READ
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0); //5555 DISABLE
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); //2AAA DISABLE
	  		  HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0); //ADDR EN

	  		  reset_addr();

	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0); //Status pin on
	  		  uint32_t addr = mem_buffer[1] | (mem_buffer[2] << 8) | (mem_buffer[3] << 16);
	  		  uint32_t loop = 0;
	  		  HAL_GPIO_WritePin(GPIOB, CE_Pin, 1); //CE DISABLE
	  		  while (loop < addr)
	  		  {
	  			  HAL_GPIO_WritePin(GPIOB, INCR_Pin, 1); //ADDR INCR
	  			  HAL_GPIO_WritePin(GPIOB, INCR_Pin, 0);
	  			  loop=loop+1;
	  		  }
	  		  HAL_GPIO_WritePin(GPIOB, CE_Pin, 0); //CE EN
	  		  delay_us(3000);
	  		  mem_buffer[0] = (GPIOA->IDR & 0x00FF);
	  		  mem_buffer[1] = (GPIOB->IDR & 0xFF00)>>8;
	  		  CDC_Transmit_FS(mem_buffer, 2);
	  		  mode_control = 0;
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1); //Status off
	  	  }

	  	  if (mode_control == writeFlash_mode) //Write file to Flash
	  	  {
	  		  reset_addr();	//Reset address counter to 0
	  		  switch_to_flash(); //Map full Flash range to address space and hide FeRAM

	  		  GPIOA->CRL = port_write;	//Set GPIOA data pins as output
	  		  GPIOB->CRH = port_write;	//Set GPIOB data pins as output
	  		  uint32_t blocks = mem_buffer[1] | (mem_buffer[2] << 8) | (mem_buffer[3] << 16);	//Parse the 3byte file size in blocks to a uint32 var

	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);	//Disable WE, important for correct waveform
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); 	//Disable Flash output, set cart level-shifter toward Flash chip
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1);	//Set Programmer data bus toward the Cartridge
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0);	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); 	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0);	//Disable address counter output, will be important later
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);	//Turn status LED on signaling the user of operation start
	  		  ready = 0;

	  		  uint32_t loop = 0;		//Initiate a block writing loop using the
	  		  while (loop < blocks)		//file size in blocks received earlier
	  		  {
	  			  buffering();	//Calls this function, it receives the data from the host and organizes it in a 256byte buffer
	  			  command_5555(0xAA);	//Necessary command sequence to put
	  			  command_2AAA(0x55);	//the Flash internal controller in
	  			  command_5555(0xA0);	//write mode

	  			  HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0);	//Enable address counter output
	  			  HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); //Pulls 0x2AAA down essentialy disabling it
	  			  HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0); //Pulls 0x5555 down essentialy disabling it

	  			  buffer_write();	//Calls this function, it writes the whole 256byte buffer to the Flash
	  			  loop=loop+1;	//Increase by one block

	  			  //HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //Not sure why i've done this, may not be necessary, must test
	  			  delay_us(3000);	//A nice delay, needed for whatever reason to the write not have glitches, the polling function should have this function, but i must messed up something
	  			  polling();	//Waits for the 0x80 write completed signal from the Flash
	  		  }
	  		  mem_buffer[0] = 'e';				//Signals the host that the loop and operation has ended
	  		  CDC_Transmit_FS(mem_buffer, 1);	//sending a 'e' chat
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1); //Turn the led off to signal the user that the operation terminated
	  		  mode_control = nullState_mode;	//Return firmware to standby nullState
	  	  }

	  	  if (mode_control == writeFeram_mode) //Write file to FeRAM
	  	  {
	  		  reset_addr();	//Reset address counter to 0

	  		  switch_to_sram();	//Map FeRAM to 0x100000
	  		  goto_addr(FERAM_ADDR);	//Set Address counter to FeRAM first address

	  		  GPIOA->CRL = port_write;	//Set GPIOA data pins as output
	  		  GPIOB->CRH = port_write;	//Set GPIOB data pins as output
	  		  uint32_t blocks = mem_buffer[1] | (mem_buffer[2] << 8) | (mem_buffer[3] << 16);	//Parse the 3byte file size in blocks to a uint32 var

	  		  HAL_GPIO_WritePin(GPIOB, CE_Pin, 1); //Disable CE, important for the FeRAM
	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);	//Disable WE, important for correct waveform
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); 	//Disable Flash output, set cart level-shifter toward Flash chip
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1);	//Set Programmer data bus toward the Cartridge
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_5555_Pin, 0);	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOB, ADDR_2AAA_Pin, 0); 	//Pull down 0x5555
	  		  HAL_GPIO_WritePin(GPIOC, ADDR_EN_Pin, 0);	//Enable Addres counter output
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0);	//Turn status LED on signaling the user of operation start
	  		  ready = 0; //Don't know if this is important

	  		  uint32_t loop = 0;		//Set a block write loop just like Flash
	  		  while (loop < blocks)		//not really needed for FeRAM, but keeps compatibility high to the host
	  		  {
	  			  buffering();	//Calls this function, it receives the data from the host and organizes it in a 256byte buffer
	  			  buffer_write_feram();	//Calls this function, writes the whole buffer to the FeRAM
	  			  loop=loop+1;	//Increases by one block
	  			  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //Also not sure about this, may not be needed
	  			  delay_us(3000);	//Reminiscent of the other function, may not be needed, must test
	  		  }
	  		  mem_buffer[0] = 'e';	//
	  		  CDC_Transmit_FS(mem_buffer, 1);
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1); // LED OFF
	  		  mode_control = 0;
	  	  }

	  	  if (mode_control == writeFlashAddr_mode) //WRITE Address
	  	  {
	  		  GPIOA->CRL = 0x33333333;
	  		  GPIOB->CRH = 0x33333333;
	  		  uint32_t addr = mem_buffer[1] | (mem_buffer[2] << 8) | (mem_buffer[3] << 16);

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1); //SET DIR WRITE
	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //WRITE DISABLE
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //5555 DISABLE
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //2AAA DISABLE
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1); //ADDR DISABLE
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0); // LED ON

	  		  //--------------------------------------------------------
  			  command_5555(0xAA);
  			  command_2AAA(0x55);
  			  command_5555(0xA0);
	  		  write_addr(mem_buffer[4],mem_buffer[5],addr);

	  		  //--------------------------------------------------------
	  		  GPIOA->CRL = 0x88888888;
	  		  GPIOB->CRH = 0x88888888;

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR READ
	  		  HAL_Delay(1000);

	  		  mem_buffer[0] = (GPIOA->IDR & 0x00FF);
	  		  mem_buffer[1] = (GPIOB->IDR & 0xFF00)>>8;
	  		  CDC_Transmit_FS(mem_buffer, 2);
	  		  mode_control = 0;
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1); //Status off
	  	  }

	  	  if (mode_control == resetFlash_mode) //RESET FLASH
	  	  {
	  		  GPIOA->CRL = port_write;
	  		  GPIOB->CRH = port_write;

	  		  //--------------------------------------------
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1); //SET DIR WRITE
	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //WRITE DISABLE
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0); // LED ON
  			  command_5555(0xAA);
  			  command_2AAA(0x55);
  			  command_5555(0xF0);

	  		  //--------------------------------------------
	  		  GPIOA->CRL = port_read;
	  		  GPIOB->CRH = port_read;

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR READ
	  		  HAL_Delay(1000);

	  		  mem_buffer[0] = (GPIOA->IDR & 0x00FF);
	  		  mem_buffer[1] = (GPIOB->IDR & 0xFF00)>>8;
	  		  CDC_Transmit_FS(mem_buffer, 2);
	  		  mode_control = 0;
	  		  HAL_GPIO_WritePin(GPIOC,  STATUS_LED_Pin, 1); //Status off
	  	  }

	  	  if (mode_control == readSR_mode) //STATUS REG READ
	  	  {

	  		  GPIOA->CRL = port_write;
	  		  GPIOB->CRH = port_write;

	  		  //--------------------------------------------
	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1); //SET DIR WRITE
	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1); //WRITE DISABLE
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0); // LED ON
  			  command_5555(0xAA);
  			  command_2AAA(0x55);
  			  command_5555(0x70);

	  		  //--------------------------------------------
	  		  GPIOA->CRL = port_read;
	  		  GPIOB->CRH = port_read;

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET READ
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR READ
	  		  HAL_Delay(1000);

	  		  mem_buffer[0] = (GPIOA->IDR & 0x00FF);
	  		  mem_buffer[1] = (GPIOB->IDR & 0xFF00)>>8;
	  		  CDC_Transmit_FS(mem_buffer, 2);
	  		  mode_control = 0;
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1); //Status off
	  	  }

	  	  if (mode_control == erase_mode) //ERASE
	  	  {
	  		  GPIOA->CRL = port_write;
	  		  GPIOB->CRH = port_write;

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 1);
	  		  HAL_GPIO_WritePin(GPIOB, WE_Pin, 1);
	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 1);
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 0); //Status pin on

  			  command_5555(0xAA);
  			  command_2AAA(0x55);
  			  command_5555(0x80);
  			  command_5555(0xAA);
  			  command_2AAA(0x55);
  			  command_5555(0x10);

	  		  GPIOA->CRL = port_read;
	  		  GPIOB->CRH = port_read;

	  		  HAL_GPIO_WritePin(GPIOB, OE_Pin, 0); //SET READ

	  		  HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0); //SET DIR READ

	  		  HAL_Delay(1000);

	  		  mem_buffer[0] = (GPIOA->IDR & 0x00FF);
	  		  mem_buffer[1] = (GPIOB->IDR & 0xFF00)>>8;
	  		  CDC_Transmit_FS(mem_buffer, 2);
	  		  mode_control = 0;
	  		  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin, 1); //Status off
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OE_Pin|DIR_Pin|INCR_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_Pin DIR_Pin INCR_Pin RST_Pin
                           WE_Pin */
  GPIO_InitStruct.Pin = OE_Pin|DIR_Pin|INCR_Pin|RST_Pin
                          |WE_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
