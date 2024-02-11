/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define USART_TXBUF_LEN 400//długość bufora nadawczego
#define USART_RXBUF_LEN 400//długość bufora odbiorczego

#define MAX_DEVICES 10//maksymalna ilość urzązeń na oneWire

//Nadawanie
uint8_t USART_TxBuf[USART_TXBUF_LEN];
volatile int USART_TX_Empty=0;
volatile int USART_TX_Busy=0;

//Wysyłanie
uint8_t USART_RxBuf[USART_RXBUF_LEN];
volatile int USART_RX_Empty=0;
volatile int USART_RX_Busy=0;


char frameBuf[200];//tablica służaca do zapisywania ramki
int frameState = 0;//zmienna służąca do sprawdzania czy znaleziono początek ramki
int frameLength = 0;//zmienna przechowująca długość ramki
int minFrameLength = 11;//minimalna długość ramki
int maxFrameLength = 41;//maksymalna długość ramki
char receiverAddress[4];//tablica przechowująca adres odbiorcy
char senderAddress[4];//tablica przechowująca adres nadawcy
char command[4];//tablica przechowująca komendę
int commandDataLength = 0;//zmienna przechowująca długość danych
int checksumLength = 2;//długość sumy kontrolnej
int measurementInterval = 1000;//zmienna przechowująca interwał czasowy pomiaru (standardowo 1000ms)
char tempUnit = 'C';//zmienna przechowująca aktualną jednostkę temperatury (standardowo stopnie Celsjusza)
float lastTemp = 0;//zmienna przechowująca ostatnią zmierzoną temperaturę
uint8_t devicesFound = 0;//zmienna przechowująca ilość znalezionych urządzeń na oneWire
uint64_t devices[MAX_DEVICES];//tablica przechowująca adresu urzązeń na oneWire


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

//BUFOR KOLOWY
void USART_fsend(char* format,...){//funkcja wysyłająca tekst do terminala
	char tmp_rs[128];
	int i;
	__IO int idx;
	va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx=USART_TX_Empty;
	for(i=0;i<strlen(tmp_rs);i++){
	  USART_TxBuf[idx]=tmp_rs[i];
	  idx++;
	  if(idx >= USART_TXBUF_LEN)idx=0;
	}
	__disable_irq();
	if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){
	  USART_TX_Empty=idx;
	  uint8_t tmp=USART_TxBuf[USART_TX_Busy];
	  USART_TX_Busy++;
	  if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
	  HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}else{
	  USART_TX_Empty=idx;
	}
	__enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){//USART Callback nadawanie
   if(huart==&huart2){
	   if(USART_TX_Empty!=USART_TX_Busy){
		   uint8_t tmp=USART_TxBuf[USART_TX_Busy];
		   USART_TX_Busy++;
		   if(USART_TX_Busy >= USART_TXBUF_LEN){
			   USART_TX_Busy=0;
		   }
		   HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	   }
   }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){//USART Callback odbiór
	 if(huart==&huart2){
		 USART_RX_Empty++;
		 if(USART_RX_Empty>=USART_RXBUF_LEN){
			 USART_RX_Empty=0;
		 }
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1);

	 }
}


//FUNCKJE SPRAWDZAJĄCE POPRAWNOŚĆ RAMKI
uint8_t genChecksum(char *data, size_t len)//funkcja do oblicznia sumy kontrolnej CRC-8
{
    uint8_t checksum = 0x00;
    size_t i, j;
    for (i = 0; i < len; i++) {
        checksum ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((checksum & 0x80) != 0){
                checksum = (uint8_t)((checksum << 1) ^ 0x07);
            }else{
                checksum <<= 1;
            }
        }
    }
    return checksum;
}

int compareChecksums(char *frameChecksum, uint8_t generatedChecksum){//funkcja do sprawdzania poprawności sumy kontrolnej
	int frameChecksumInt = (int)strtol(frameChecksum, NULL, 16);
	if(frameChecksumInt == generatedChecksum){
		return 1;
	}else{
		return 0;
	}
}

int checkCharacters(char *txt, int isChecksum){//funkcja do sprawdzenia czy w pola Nadawca, Odbiorca, Komenda oraz Suma kontrolna wpisano dozwolone znaki
	size_t len = strlen(txt);
	if(isChecksum){//suma kontrolna ma inny zakres znaków
		for(size_t i = 0; i < len; i++){
			if((txt[i] < '0' || txt[i] > '9') && (txt[i] < 'A' || txt[i] > 'F')){
				return 0; //jeśli jakiś znak nie jest w zakresie, to zwracane jest 0
			}
		}
		return 1; //jeśli wszystkie znaki są w zakresie to zwracane jest 1
	}else{
		for(size_t i = 0; i < len; i++){
			if(txt[i] < 'A' || txt[i] > 'Z'){
				return 0;
			}
		}
		return 1;
	}
}

int checkSenderAndReceiver(char *sender, char *receiver){ //funkcja do sprawdzania czy adresy nadawcy i odbiorcy są prawidłowe
	if((!strcmp(sender, "HST")) && (!strcmp(receiver, "STM"))){
		return 1;
	}else{
		return 0;
	}
}

//ONE WIRE ORAZ OBSLUGA DS18B20
void usDelay(int us){//opóźnienie w mikrosekundach za pomocą TIM6
	__HAL_TIM_SET_COUNTER(&htim6, 0);//zerowanie licznika timera
	while(__HAL_TIM_GET_COUNTER(&htim6) < us){}//odczytywanie stanu licznika, który jest zwiększany co 1us
}

HAL_StatusTypeDef wireReset(){//generowanie sekwencji reset (rozpoczęcie komunikacji), opóźnienia zgodne ze specyfikacją
	int rc;
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	usDelay(480);
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	usDelay(70);
	rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	usDelay(410);
	if(rc == 0){
		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

void writeBit(int value){//funkcja wysyłająca pojedyncze bity
	if(value){//wysyłanie 1
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		usDelay(6);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		usDelay(64);
	}else{//wysyłanie 0
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		usDelay(60);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		usDelay(10);
	}
}

int readBit(){//funkcja odczytująca pojedyncze bity
	int rc;
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	usDelay(6);
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	usDelay(9);
	rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	usDelay(55);
	return rc;
}

void wireWrite(uint8_t byte){//funkcja pisząca cały bajt
	int i;
	for(i = 0; i < 8; i++){
		writeBit(byte & 0x01);//wysyłanie najmniej znaczącego bitu
		byte >>=1;//przesunięcie w prawo
	}
}

uint8_t wireRead(){//zczytywanie całego bajtu
	uint8_t value = 0;
	int i;
	for(i = 0; i < 8; i++){
		value >>= 1;//przesunięcie w prawo
		if(readBit()){//jeśli bit wynosi 1
			value |= 0x80;//ustawienie MSB na 1
		}
	}
	return value;
}

void searchROM() {//funkcja wyszukująca adresy urządzeń na oneWire
	static uint8_t lastDiscrepancy = 0;
	    uint8_t discrepancyMarker = 0;
	    uint8_t bitPosition;
	    uint8_t bitA;
	    uint8_t bitB;
	    uint8_t bitCounter = 0;
	    uint8_t romByte = 0;
	    uint8_t searchResult;
	    uint64_t romCode = 0;
	    if (wireReset() == HAL_ERROR) {
	        return;
	    }
	    wireWrite(0xF0);
	    bitPosition = 0;
	    do {
	        bitA = readBit();
	        bitB = readBit();
	        if (bitA && bitB) {
	            break;
	        } else {
	            if (bitA ^ bitB) {
	                search_result = bitA;
	            } else {
	                if (bitPosition < lastDiscrepancy) {
	                    searchResult = ((romCode & ((uint64_t)1 << bitPosition)) > 0);
	                } else {
	                    searchResult = (bitPosition == lastDiscrepancy);
	                }
	                if (searchResult == 0) {
	                    discrepancyMarker = bitPosition;
	                }
	            }
	            romCode |= ((uint64_t)searchResult) << bitPosition;
	            writeBit(searchResult);
	            bitPosition++;
	            bitCounter++;
	            romByte >>= 1;
	            if (searchResult) {
	                romByte |= 0x80;
	            }
	            if (bitCounter == 8) {
	                bitCounter = 0;
	                romByte = 0;
	            }
	        }
	    } while (bitPosition < 64);
	    if (!(romCode == 0)) {
	        devices[devicesFound++] = romCode;
	    }
	    lastDiscrepancy = discrepancyMarker;
	    return;
}


void measureTemp(){
	static uint32_t lastMeasurementTime = 0;
	uint32_t currentTime = HAL_GetTick();//czas od uruchomienia programu (w ms)

	if((currentTime - lastMeasurementTime >= measurementInterval) || currentTime == 0){//sprawdzanie czy upłynęło wystarczająco dużo czasu
		wireReset();//wysyłanie sekwencji resetującej
		wireWrite(0xcc);//polecenie Skip ROM (pominięcie adresowania)
		wireWrite(0x44);//polecenie Convert T (rozpoczęcie pomiaru)

		while (wireRead() == 0); //oczekiwanie na zakończenie pomiaru

		wireReset();
		wireWrite(0xcc);//polecenie SkipRom
		wireWrite(0xbe);//polecenie Read Scratchpad (odczyt)

		int i;
		uint8_t scratchpad[9];//tablica przechowująca odczyt
		for(i = 0; i < 9; i++){
			scratchpad[i] = wireRead();//zapisywanie do tablicy
		}

		float result = (scratchpad[1] << 8) | scratchpad[0];//składanie 2 najbardziej znaczących bajtów w jedną liczbę
		lastTemp = result/16;//aby uzyskać temperaturę w stopniach Celsjusza należy podzielić wynik przez 16
		lastMeasurementTime = currentTime;//aktualizowanie ostatniego czasu pomiaru
	}
}

//KOMENDY I KOMUNIKATY ZWROTNE
void wrongCmd(){
	char frame[50];
	uint8_t checksum = genChecksum("STMHSTWCD", strlen("STMHSTWCD"));
	sprintf(frame, ":STMHSTWCD%X;", checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void wrongDataFormat(){
	char frame[50];
	uint8_t checksum = genChecksum("STMHSTWDF", strlen("STMHSTWDF"));
	sprintf(frame, ":STMHSTWDF%02X;", checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void wrongDataRange(){
	char frame[50];
	uint8_t checksum = genChecksum("STMHSTWDR", strlen("STMHSTWDR"));
	sprintf(frame, ":STMHSTWDR%02X;", checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void changeMeasurementInterval(char *newInterval){ //zmiana interwału pomiarowego
	char *endptr;
	int intervalInt = (int)strtol(newInterval, &endptr, 10);
	if(*endptr != '\0'){
		wrongDataFormat();
	}else{
		if(intervalInt >= 500 && intervalInt <= 60000){
			measurementInterval = intervalInt;
		}else{
			wrongDataRange();
		}
	}
}

void getMeasurementInterval(){ //odczyt interwału pomiarowego
	char frame[50];
	char dataForChecksum[50];
	uint8_t checksum;
	sprintf(dataForChecksum, "STMHSTCMI=%d", measurementInterval);
	checksum = genChecksum(dataForChecksum, strlen(dataForChecksum));
	sprintf(frame, ":STMHSTCMI=%d%02X;", measurementInterval, checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void changeTmpUnit(char newUnit){ //zmiana jednostki temperatury
	tempUnit = newUnit;
}

void getCurrentTmpUnit(){ //odczyt ustawionej jednostki temperatury
	char frame[50];
	char dataForChecksum[50];
	uint8_t checksum;
	sprintf(dataForChecksum, "STMHSTCTU%c", tempUnit);
	checksum = genChecksum(dataForChecksum, strlen(dataForChecksum));
	sprintf(frame, ":STMHSTCTU%c%02X;", tempUnit, checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void getTemperature(){//odczyt temperatury
	char frame[50];
	char dataForChecksum[50];
	uint8_t checksum;
	float temperature;
	switch(tempUnit){
		case 'C':
			temperature = lastTemp;
			break;
		case 'F':
			temperature = (lastTemp * 9/5) + 32;
			break;
		case 'K':
			temperature = lastTemp + 273.15;
			break;
	}
	sprintf(dataForChecksum, "STMHSTTMP=%0.2f%c", temperature, tempUnit);
	checksum = genChecksum(dataForChecksum, strlen(dataForChecksum));
	sprintf(frame, ":STMHSTTMP=%0.2f%c%02X;", temperature, tempUnit, checksum);
	USART_fsend("STM->%s\r\n", frame);
}

void searchOneWire(){//wyszukiwanie urządzeń na oneWire
	searchROM();
	int i, j;
	for (i = 0; i < devicesFound; i++) {
		char frame[50];
		char dataForChecksum[50];
		uint8_t checksum;
		uint8_t rom[8];//tablica przechowująca adres jednego urządzenia
		for (j = 0; j < 8; j++) {
			rom[j] = (uint8_t)(devices[i] >> (j * 8));//przepisywanie adresu jednego urządzenia
		}
		sprintf(dataForChecksum, "STMHSTOWR%02X%02X%02X%02X%02X%02X%02X%02X", rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
		checksum = genChecksum(dataForChecksum, strlen(dataForChecksum));
		sprintf(frame, ":STMHSTOWR%02X%02X%02X%02X%02X%02X%02X%02X%02X;", rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7], checksum);
		USART_fsend("STM->%s\r\n", frame);
	}
}

void analyzeCommand(char *cmd, char *data){//analiza komendy
	if(!strcmp(cmd, "SMI")){
		changeMeasurementInterval(data);
	}else if(!strcmp(cmd, "GMI")){
		getMeasurementInterval();
	}else if(!strcmp(cmd, "GTM")){
		getTemperature();
	}else if(!strcmp(cmd, "SOW")){
		searchOneWire();
	}else if(!strcmp(cmd, "STF")){
		changeTmpUnit('F');
	}else if(!strcmp(cmd, "STC")){
		changeTmpUnit('C');
	}else if(!strcmp(cmd, "STK")){
		changeTmpUnit('K');
	}else if(!strcmp(cmd, "GTU")){
		getCurrentTmpUnit();
	}else{
		wrongCmd();
	}
}




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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,&USART_RxBuf[0],1);//rozpoczęcie pobierania znaków
  USART_fsend("\r\nSZYMON POMIECINSKI\r\n");
  USART_fsend("MIKROPROCESORY (PROJEKT)\r\n");
  USART_fsend("OBSLUGA TERMOMETRU DS18B20\r\n");
  USART_fsend("HST->");


  while (1)
  {

	  uint8_t singleChar; //pojedynczy znak pobierany z bufora

	  if(USART_RX_Empty != USART_RX_Busy){//początek poszukiwania ramki
		if(frameState == 0){//szukanie ramki
		  singleChar = USART_RxBuf[USART_RX_Busy];//pobieranie pierwszego znaku

		  if(singleChar == 0x3A){//jeśli znaleziono : (początek ramki)
			  frameState = 1;
			  frameLength = 0;
		  }

		  USART_RX_Busy++;
		  if(USART_RX_Busy >= USART_RXBUF_LEN){
			  USART_RX_Busy = 0;
		  }

		}else if(frameState == 1){//jeśli początek ramki jest znaleziony
		  singleChar = USART_RxBuf[USART_RX_Busy];//pobieranie kolejnego znaku

		  if(singleChar == 0x3A){//sprawdzenie czy znak początku się powtarza
			  if(frameLength > 1){
				  frameLength = 0;
			  }
		  }else{
			  frameBuf[frameLength] = singleChar;//zapisywanie ramki
			  frameLength++;
		  }

		  USART_RX_Busy++;
		  if(USART_RX_Busy >= USART_RXBUF_LEN){
			  USART_RX_Busy = 0;
		  }

		  if(singleChar == 0x3B){//zapisywanie do ramki do napotkania znaku końca ramki (;)
			  if(frameLength - 1 > maxFrameLength){//sprawdzenie czy ramka nie jest zbyt długa
				  USART_fsend("HST->");
				  frameLength = 0;
				  frameState = 0;
				  memset(frameBuf, 0, sizeof(frameBuf));//czyszczenie bufora ramki
				  continue;//i przechodzenie do następnej iteracji pętli
			  }
			  if(frameLength > minFrameLength){//sprawdzanie czy ramka ma minimalną długość
				  //zapisywanie poszczególnych pól ramki
				  frameBuf[frameLength - 1] = '\0';

				  memcpy(senderAddress, &frameBuf[0], 3);
				  senderAddress[3] = '\0';

				  memcpy(receiverAddress, &frameBuf[3], 3);
				  receiverAddress[3] = '\0';

				  memcpy(command, &frameBuf[6], 3);
				  command[3] = '\0';

				  commandDataLength = frameLength - 9 - checksumLength - 1; //9, ponieważ taki rozmiar mają pola przed polem danych
				  char commandData[commandDataLength + 1];
				  memcpy(commandData, &frameBuf[9], commandDataLength);
				  commandData[commandDataLength] = '\0';

				  char checksum[checksumLength + 1];
				  memcpy(checksum, &frameBuf[9 + commandDataLength], checksumLength);
				  checksum[checksumLength] = '\0';

				  if(checkCharacters(senderAddress, 0) && checkCharacters(receiverAddress, 0) &&
					 checkCharacters(command, 0) && checkCharacters(checksum, 1)){//sprawdzanie czy w pola wpisano dozwolone znaki
					  if(checkSenderAndReceiver(senderAddress, receiverAddress)){//sprawdzanie poprawności adresów
						  //oddzielanie danych z ramki do obliczeń sumy kontrolnej
						  int frameWithoutChsumLen = frameLength - checksumLength;
						  char frameWithoutChsum[frameWithoutChsumLen];
						  strncpy(frameWithoutChsum, frameBuf, frameWithoutChsumLen);
						  frameWithoutChsum[frameWithoutChsumLen - 1] = '\0';

						  //obliczanie sumy kontrolnej
						  uint8_t calculatedChecksum = genChecksum(frameWithoutChsum, strlen(frameWithoutChsum));

						  if(compareChecksums(checksum, calculatedChecksum)){ //sprawdzanie poprawności sumy kontrolnej
							  //analizowanie komendy
							  analyzeCommand(command, commandData);
						  }
					  }
				  }
			  }

			  USART_fsend("HST->");
			  frameState = 0;
			  frameLength = 0;
			  memset(frameBuf, 0, sizeof(frameBuf));//czyszczenie bufora ramki
		  }
		}
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 44;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS_Pin */
  GPIO_InitStruct.Pin = DS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
