/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ili9488.h"
#include "xpt2046.h"
#include "stdio.h"

#include "008_Open_Sans_Bold.h"
#include "009_Open_Sans_Bold.h"
#include "010_Open_Sans_Bold.h"
#include "012_Open_Sans_Bold.h"
#include "014_Open_Sans_Bold.h"
#include "016_Open_Sans_Bold.h"
#include "018_Open_Sans_Bold.h"
#include "020_Open_Sans_Bold.h"
#include "022_Open_Sans_Bold.h"
#include "024_Open_Sans_Bold.h"
#include "026_Open_Sans_Bold.h"
#include "028_Open_Sans_Bold.h"
#include "036_Open_Sans_Bold.h"
#include "048_Open_Sans_Bold.h"
#include "072_Open_Sans_Bold.h"
#include "096_Open_Sans_Bold.h"
#include "112_Open_Sans_Bold.h"
#include "128_Open_Sans_Bold.h"

#define _Open_Sans_Bold_8      &Open_Sans_Bold_8
#define _Open_Sans_Bold_9      &Open_Sans_Bold_9
#define _Open_Sans_Bold_10     &Open_Sans_Bold_10
#define _Open_Sans_Bold_11     &Open_Sans_Bold_11
#define _Open_Sans_Bold_12      &Open_Sans_Bold_12
#define _Open_Sans_Bold_14      &Open_Sans_Bold_14
#define _Open_Sans_Bold_16      &Open_Sans_Bold_16
#define _Open_Sans_Bold_18      &Open_Sans_Bold_18
#define _Open_Sans_Bold_20      &Open_Sans_Bold_20
#define _Open_Sans_Bold_22      &Open_Sans_Bold_22
#define _Open_Sans_Bold_24      &Open_Sans_Bold_24
#define _Open_Sans_Bold_26      &Open_Sans_Bold_26
#define _Open_Sans_Bold_28      &Open_Sans_Bold_28
#define _Open_Sans_Bold_36      &Open_Sans_Bold_36
#define _Open_Sans_Bold_48      &Open_Sans_Bold_48
#define _Open_Sans_Bold_72      &Open_Sans_Bold_72
#define _Open_Sans_Bold_96      &Open_Sans_Bold_96
#define _Open_Sans_Bold_112      &Open_Sans_Bold_112
#define _Open_Sans_Bold_128      &Open_Sans_Bold_128

volatile uint8_t SPI1_TX_completed_flag = 1;

char buf1[20];



#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define BBLACK 6
//#define Aqua 7

#define BYELLOW 8


uint16_t AA[315]={315,314,313,312,311,310,309,308,307,306,305,304,303,302,301,300,299,298,297,296,
		295,294,293,292,291,290,289,288,287,286,285,284,283,282,281,280,279,278,277,276,275,274,
		273,272,271,270,269,268,267,266,265,264,263,262,261,260,259,258,257,256,255,254,253,252,
		251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,
		229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,
		207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,
		185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,
		163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,
		141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,
		119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,
		96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,
		66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,
		36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,
		4,3,2,1};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define VREFINT_CAL_ADDR ((uint16_t*)0x1FFF7A2A) //compared against actual VREFINT to determine actual VDDA for ADC accuracy

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SPI1_TX_completed_flag = 1;
}


//ADC vars
#define ADC_BUF_LEN 1024//1024
#define SYSCLK_FREQ 100000000
#define CLOCKTIM_PRESC 0
volatile uint16_t adc_buf[ADC_BUF_LEN];
//test ADCVars
uint32_t adcValue = 0;
uint16_t adcCallbackTracker = 0; //track how many times buffer has filled so can call VREFINT
int triggerVDDAInfer = 0;
int triggerDisplay = 0;
char adcCharBuffer[16];

//stats to be kept track of for each buffer
uint16_t maxVoltage = 0;
uint16_t minVoltage = 4095;
uint32_t avgVoltage = 0;
double rmsVoltage = 0;
uint8_t attenuation = 2;
//triggering stuff
int triggering = 1; //if we should look for triggers
uint16_t triggerVoltage = 500; //voltage level needed to trigger
int triggersFound, triggerPoint1, triggerPoint2 = 0;
int risingTrigger = 0; //if trigger should be rising or falling
//timing stuff
uint32_t wavePeriod = 0;
uint32_t waveFrequency = 0;
uint32_t timerFrequency = 0;
uint32_t timeStep = 3; //timestep index : per div in microseconds
uint32_t timeStepOptions[11] = {35, 50, 100, 250, 500, 1000, 5000, 25000, 100000, 500000, 1000000};
//screen vars
uint8_t xDiv = 17;
uint8_t yDiv = 11;
uint8_t samplesPerDiv = 32; //should be a calculation ideally but okay to keep as constant . Found by (ADC_BUF_LEN/2)/(xDiv-1)
//manually selecting channels for VREFINT calibration, wont be needed later when switch to DMA
ADC_ChannelConfTypeDef ADC_CH_Cfg = {0};

//vars for VREFINT based ADC calibration
uint32_t vrefintReading = 0;
uint32_t inferredVDDA = 0;

//TESTING DELETE ME
uint16_t yTrack = 20;
uint16_t xTrack = 0;

//use reading from VREFINT to infer what real VDDA is
//assumes &hadc1
void inferVDDA(void){
	HAL_ADC_Stop_DMA(&hadc1);
	ADC1->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL); // disable external trigger
	ADC_CH_Cfg.Channel = ADC_CHANNEL_VREFINT;        // Select The ADC Channel [i]
	ADC_CH_Cfg.Rank = 1;          // Must specify rank
	ADC_CH_Cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES; // Use a long sample time
	HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg); // Configure The Selected ADC Channel
    HAL_ADC_Start(&hadc1);                         // Start ADC Conversion @ Selected Channel
    HAL_ADC_PollForConversion(&hadc1, 1);         // Poll The ADC Channel With TimeOut = 1mSec

    vrefintReading = HAL_ADC_GetValue(&hadc1); // Read the result
    uint32_t vrefint_cal = *VREFINT_CAL_ADDR; // Factory calibration value
    //3300 is expected VDA in millivolts
    //dividing calibration value from factory by what was read to infer real VDDA
    inferredVDDA = (vrefint_cal*3300)/(vrefintReading);//(int)3.3f * (vrefint_cal / vrefintReading)*1000;

    HAL_ADC_Stop(&hadc1);
	ADC_CH_Cfg.Channel = ADC_CHANNEL_0;        // Select The ADC Channel
	ADC_CH_Cfg.Rank = 1;          // Must specify rank
	ADC_CH_Cfg.SamplingTime = ADC_SAMPLETIME_15CYCLES; //SET TO 15 RIGHT NOW FOR ACCURACY TAKE A LOOK AT LATER IF MORE SPEED IS NEEDED
	HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg); // Configure The Selected ADC Channel
	ADC1->CR2 |= (ADC_CR2_EXTEN_0); // Rising edge
	ADC1->CR2 |= (ADC_CR2_EXTSEL_3); //Selects Timer 3 TRGO
}

//halffull ADC buffer callback
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
//	if (hadc->Instance == ADC1) {
//		// adc_buf[0] to adc_buf[half ADC_BUF_LEN-1] are ready to process
//		triggerDisplay = 1;
//		bufferHalf = 1;
//    }
//}
//full ADC Buffer Callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		adcCallbackTracker += 1;
        // adc_buf[half ADC_BUF_LEN-1] to adc_buf[ADC_BUF_LEN-1] are ready to process
		triggerDisplay = 1;
		if(adcCallbackTracker>0){
			triggerVDDAInfer = 1;
			adcCallbackTracker = 0;
		}
    }
}
//display readings or whatever variable in a tiled update fashion
void testDMADisplay(){
	adcToVoltageString(adc_buf[2], adcCharBuffer, sizeof(adcCharBuffer), inferredVDDA);
//		sprintf(adcCharBuffer, "%lu", (unsigned long)inferredVDDA);
//		sprintf(adcCharBuffer, "%lu", (unsigned long)adcValue);

	LCD_Font(xTrack, yTrack, adcCharBuffer, _Open_Sans_Bold_24  , 1, WHITE);
	yTrack += 20;
	if(yTrack>300){
		xTrack += 100;
		yTrack = 20;
		if(xTrack > 400) {
			ILI9341_Fill_Screen(RED);
			yTrack = 0;
			xTrack = 0;
		}
	}
	HAL_Delay(250);
}

//adcVal = Reading from adc 0-4095
//voltageScale = Value in millivolts of what adc reading should scale to. Default 3.3V
// buffer and buffer size for snprintf
void adcToVoltageString(uint32_t adcVal, char* buffer, size_t bufferSize, uint32_t voltageScale){
    uint32_t millivolts = (adcVal * voltageScale) / 4095;

    // Extract whole volts and fractional part
    uint32_t volts = millivolts / 1000;
    uint32_t frac = millivolts % 1000;

    // Format as "X.XXX V"
    snprintf(buffer, bufferSize, "%lu.%03lu V", volts, frac);
}

void uiLoop(){

	drawGraticule(xDiv, yDiv, 20);
	if(triggering)
		findTrigger();
	graphData(340, 220, YELLOW);
	displayStats(20);
	//displayGraticuleStats(20);

//	adcToVoltageString(minVoltage, adcCharBuffer, sizeof(adcCharBuffer), inferredVDDA);
//	LCD_Font(200, 200, adcCharBuffer, _Open_Sans_Bold_24  , 1, WHITE);
//	adcToVoltageString(maxVoltage, adcCharBuffer, sizeof(adcCharBuffer), inferredVDDA);
//	LCD_Font(200, 100, adcCharBuffer, _Open_Sans_Bold_24  , 1, WHITE);
//
//	adcToVoltageString(avgVoltage, adcCharBuffer, sizeof(adcCharBuffer), inferredVDDA);
//	LCD_Font(200, 50, adcCharBuffer, _Open_Sans_Bold_24  , 1, WHITE);

	HAL_Delay(1);
}

//assumes graph top left corner at 0,0 and that x and y divs will be equal space. No option to change total width and length of graticule atm
void drawGraticule(uint16_t xDivCount, uint16_t yDivCount, uint16_t divLength) {
	uint16_t w = xDivCount * divLength;
	uint16_t h = yDivCount * divLength;

	setAddrWindow(0, 0, w-1, h-1);

	ILI9341_Draw_Colour_Burst(BLACK, w * h);
	for (int i = 0; i <= w; i += divLength)
		drawFastVLine(i, 0, h, WHITE);

    for (int i = 0; i <= h; i += divLength)
    	drawFastHLine(0, i, w, WHITE);

}

//w and h = width and height of graph area
void graphData(uint16_t w, uint16_t h, uint32_t color){
	w = w-2;
	maxVoltage = 0;
	minVoltage = 4095;
	avgVoltage = 0;
	rmsVoltage = 0;

	for(int i = 0; i<ADC_BUF_LEN/2; i++){
		int iPTP = i + triggerPoint1; //i plus trigger point
	    maxVoltage = (adc_buf[iPTP] > maxVoltage) ? adc_buf[iPTP] : maxVoltage;
	    minVoltage = (adc_buf[iPTP] < minVoltage) ? adc_buf[iPTP] : minVoltage;
	    avgVoltage += adc_buf[iPTP];
	    rmsVoltage += adc_buf[iPTP]*adc_buf[iPTP];
		if(i%4==0 && i>0){
	    	//LCD_Font((w*i)/ADC_BUF_LEN*2, h-((h)*adc_buf[iPTP])/4095, ".", _Open_Sans_Bold_20, 1, color);
			drawLine((w*(i-4))/ADC_BUF_LEN*2, h-((h)*adc_buf[iPTP-4])/4095, (w*i)/ADC_BUF_LEN*2, h-((h)*adc_buf[iPTP])/4095, color);
	    }
	}
	avgVoltage = avgVoltage/(ADC_BUF_LEN/2);
	rmsVoltage = sqrt(rmsVoltage/(ADC_BUF_LEN/2));
}

void findTrigger(){
	triggersFound = 0;
	triggerPoint1 = 0;
	triggerPoint2 = 0;
	waveFrequency = 0;
	wavePeriod = 0;
	for(int i = 0; i<ADC_BUF_LEN/2; i++){
        if ((risingTrigger && adc_buf[i] >= triggerVoltage && adc_buf[i - 1] < triggerVoltage) || (risingTrigger == 0 && adc_buf[i] <= triggerVoltage && adc_buf[i - 1] > triggerVoltage))
        {
            if (triggersFound==0) {
                triggerPoint1 = i;
                triggersFound = 1;
            }
            else // Looking for the second one
            {
            	if(triggerPoint2 == 0){
            		triggerPoint2 = i;
            		triggersFound = 2;

            	}
                break;

            }
        }
	}
	if(triggersFound==2){
		// get difference by samples of trigger points, mult by microseconds per 32 samples (timeStep), divide by 32
		wavePeriod = ((triggerPoint2-triggerPoint1)* timeStepOptions[timeStep])/samplesPerDiv;
		//period to frequency with 1 -> 1,000,000 so can divide by microseconds instead of seconds
		waveFrequency = 1000000/(wavePeriod);
	}

}

uint8_t rightButtonPressed = 0;
uint8_t leftButtonPressed = 0;
uint8_t selection = 0;
uint32_t last = 0;

//handle buttons
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint32_t now = HAL_GetTick();
	if(now - last > 250){
		last = now;
	}else{
		return;
	}
	if(GPIO_Pin == SelectButton_Pin){
		selection = (selection > 2) ? 0 : selection+1;
		if(triggering==0 && (selection==1 || selection==2)){
			selection = 3;
		}
	}
	if(GPIO_Pin == LeftButton_Pin){
		leftButtonPressed = 1;
	}
	if(GPIO_Pin == RightButton_Pin){
			rightButtonPressed = 1;
	}
}

//stringBuffer for stat display
char stringBuffer[32];
void displayStats(uint8_t spacing){
	setAddrWindow(341, 0, ILI9488_TFTHEIGHT-1, ILI9488_TFTWIDTH-1);
	ILI9341_Draw_Colour_Burst(BLACK, 140*250);

	uint8_t pointer = 15;

	uint16_t postProcessedInferredVDDA = inferredVDDA * attenuation;

	adcToVoltageString(avgVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "Vave: %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
	pointer+=spacing;

	adcToVoltageString(rmsVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "Vrms: %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
	pointer+=spacing;

	adcToVoltageString(maxVoltage-minVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "Vpp:   %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
	pointer+=spacing;

	adcToVoltageString(maxVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "Vmax:%s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
	pointer+=spacing;

	adcToVoltageString(minVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "Vmin: %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
	pointer+=spacing;

	snprintf(stringBuffer, sizeof(stringBuffer), "Triggering", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16, 1, (selection==0) ? YELLOW : (triggering) ? GREEN : RED);
	pointer+=spacing;

	if(triggering){

		snprintf(stringBuffer, sizeof(stringBuffer), (risingTrigger) ? "Rising Edge": "Falling Edge", adcCharBuffer);
		LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16, 1, (selection==1) ? YELLOW : WHITE);
		pointer+=spacing;

		adcToVoltageString(triggerVoltage, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
		snprintf(stringBuffer, sizeof(stringBuffer), "TV: %s", adcCharBuffer);
		LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, (selection==2) ? YELLOW : WHITE);
		pointer+=spacing;

		periodToString(wavePeriod, adcCharBuffer, sizeof(adcCharBuffer));
		snprintf(stringBuffer, sizeof(stringBuffer), "Cyc: %s", adcCharBuffer);
		LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
		pointer+=spacing;

		frequencyToString(waveFrequency, adcCharBuffer, sizeof(adcCharBuffer));
		snprintf(stringBuffer, sizeof(stringBuffer), "F: %s", adcCharBuffer);
		LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16, 1, WHITE);
		pointer+=spacing;
	}


	periodToString(timeStepOptions[timeStep], adcCharBuffer, sizeof(adcCharBuffer));
	snprintf(stringBuffer, sizeof(stringBuffer), "xDiv: %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, (selection==3) ? YELLOW : WHITE);
	pointer+=spacing;

	adcToVoltageString(410, adcCharBuffer, sizeof(adcCharBuffer), postProcessedInferredVDDA);
	snprintf(stringBuffer, sizeof(stringBuffer), "yDiv: %s", adcCharBuffer);
	LCD_Font(350, pointer, stringBuffer, _Open_Sans_Bold_16  , 1, WHITE);
}

void displayGraticuleStats(uint8_t spacing){
	setAddrWindow(0, 221, ILI9488_TFTHEIGHT-1, ILI9488_TFTWIDTH-1);
	ILI9341_Draw_Colour_Burst(WHITE, 140*200);
}

void periodToString(uint32_t period, char* buffer, size_t bufferSize){
    uint16_t milliSeconds = period / 1000;
    uint16_t microSeconds = period % 1000;

    snprintf(buffer, bufferSize, "%lu.%03lu ms", milliSeconds, microSeconds);
}

void frequencyToString(uint32_t freqVal, char* buffer, size_t bufferSize){
    uint16_t khz = freqVal / 1000;
    uint16_t hz = freqVal % 1000;
	snprintf(buffer, bufferSize, "%lu.%03lu kHz", khz, hz);
}


void fillBuffer() {
	triggerDisplay = 0;
//	htim3.Instance->CR1
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
	while(triggerDisplay==0);
	triggerDisplay=1;
	HAL_TIM_Base_Stop(&htim3);
}

void setTimerFreq(int freq){

	uint16_t arr = (SYSCLK_FREQ / ((CLOCKTIM_PRESC + 1) * freq)) - 1;
	htim3.Instance->ARR = arr;
	htim3.Instance->PSC = 0;//1279;
	//Update generation, reinits counter. Reference manual page 402
	htim3.Instance->EGR = TIM_EGR_UG;
}



int changeTimeStep = 0;

void processInput(){
	if(selection==0){
		if(rightButtonPressed || leftButtonPressed){
			triggering = (triggering==1) ? 0 : 1;
		}
	}else if(selection==1){
		if(rightButtonPressed || leftButtonPressed){
			risingTrigger = (risingTrigger==1) ? 0 : 1;
		}
	}else if(selection==2){
		if(rightButtonPressed){
			triggerVoltage = (triggerVoltage < 3997) ? triggerVoltage+100 : 0;
		}else if(leftButtonPressed){
			triggerVoltage = (triggerVoltage > 99) ? triggerVoltage-100 : 4000;
		}
	}else if (selection==3){
		if(rightButtonPressed){
			if (timeStep == 10){
				timeStep = 0;
			}else{
				timeStep += 1;
			}
			changeTimeStep = 1;
		}else if(leftButtonPressed){
			if (timeStep == 0){
				timeStep = 10;
			}else{
				timeStep += -1;
			}
			changeTimeStep = 1;
		}
	}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ILI9488_Init();

 //   HAL_Delay(1000);
    setRotation(1);


    //HAL_ADC_Start(&hadc1);
 //   uint16_t touchx = 0, touchy = 0;

    inferVDDA();
	ILI9341_Fill_Screen(BLACK);
//	drawImage(image_data_test, 0, 0, 480,80);
	//multiply samplesPerDiv by 1,000,000 in order to account for the fact that we measure timesteps in milliseconds. this avoids any pesky decimals
	timerFrequency = (samplesPerDiv*1000000)/timeStepOptions[timeStep];
	setTimerFreq(timerFrequency);
	invertDisplay(0);
	while (1) {
		fillBuffer();
		if(triggerVDDAInfer==1){
			//ILI9341_Fill_Screen(BLACK);
			triggerVDDAInfer = 0;
			inferVDDA();
		}
		if(leftButtonPressed || rightButtonPressed){
			processInput();
			rightButtonPressed = 0;
			leftButtonPressed = 0;
		}
		if(changeTimeStep == 1){
			timerFrequency = (samplesPerDiv*1000000)/timeStepOptions[timeStep];
			setTimerFreq(timerFrequency);
			changeTimeStep = 0;
		}
		uiLoop();
}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 336;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin|TFT_RST_Pin|TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_CS_Pin TFT_RST_Pin TFT_DC_Pin TP_CS_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|TFT_RST_Pin|TFT_DC_Pin|TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SelectButton_Pin RightButton_Pin LeftButton_Pin */
  GPIO_InitStruct.Pin = SelectButton_Pin|RightButton_Pin|LeftButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
