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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int x;
    int y;
    int r;
    int g;
    int b;
    float intensity; // Intensity of the sparkle, 0.0 (off) to 1.0 (full brightness)
    int fadingOut; // Indicates if the sparkle is fading out
} Sparkle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LED 968
#define USE_BRIGHTNESS 1
#define GLOBAL_BRIGHTNESS 30

#define DELAY_MS 100
#define TAIL_LENGTH 5 // Length of the tail effect
#define TRANSITION_STEPS 100
#define PI 3.14159265
const int matrixWidth = 44;
const int matrixHeight = 22;
#define maxSparkles 200 // Total number of potential sparkles
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
uint8_t LED_Data[NUM_LED][3];
uint8_t LED_Mod[NUM_LED][3];  // for brightness
uint16_t pwmData[(24*NUM_LED)+100];
volatile uint8_t datasentflag=0;
Sparkle sparkles[maxSparkles];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  datasentflag = 1; // Set the flag
}

void WS2812_Update(void) {
	uint32_t indx=0;
	uint32_t color;

	for (int i=0; i<50; i++) {
	pwmData[indx] = 0;
	indx++;
	}


	for (int i = 0; i<NUM_LED; i++) {
		#if USE_BRIGHTNESS
		color = ((LED_Mod[i][0]<<16) | (LED_Mod[i][1]<<8) | (LED_Mod[i][2]));
		#else
		color = ((LED_Data[i][0]<<16) | (LED_Data[i][1]<<8) | (LED_Data[i][2]));
		#endif
		for (int i=23; i>=0; i--) {
			if (color&(1<<i)) {
				pwmData[indx] = 60;  // 2/3 of 90
			} else {
				pwmData[indx] = 30;  // 1/3 of 90
			}
			indx++;
		}
	}

	for (int i=0; i<50; i++) {
		pwmData[indx] = 0;
		indx++;
	}

    datasentflag = 0; // Reset the flag before starting
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
    // Wait for the flag to be set
//    while (!datasentflag) {
//        // Optionally, you could add a timeout here
////    	HAL_Delay(2);
//    }
    datasentflag = 0; // Reset flag for next transmission
}

void setLED (int led, int RED, int GREEN, int BLUE)
{
	LED_Data[led][0] = GREEN;
	LED_Data[led][1] = RED;
	LED_Data[led][2] = BLUE;
}

void setBrightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 100) brightness = 100;
	for (int i=0; i<NUM_LED; i++)
	{
		for (int j=0; j<3; j++)
		{
			LED_Mod[i][j] = (LED_Data[i][j]*brightness)/100;
		}
	}

#endif

}

void smoothTransition(uint8_t startColor[3], uint8_t endColor[3]) {
    for (int step = 0; step <= TRANSITION_STEPS; step++) {
        float ratio = (float)step / (float)TRANSITION_STEPS;

        // Calculate current RGB values based on the ratio
        uint8_t currentR = (uint8_t)(startColor[0] + (endColor[0] - startColor[0]) * ratio);
        uint8_t currentG = (uint8_t)(startColor[1] + (endColor[1] - startColor[1]) * ratio);
        uint8_t currentB = (uint8_t)(startColor[2] + (endColor[2] - startColor[2]) * ratio);

        // Set the LED with the current color
        for (int i = 0; i < NUM_LED; i++) {
        	if (1) {
        		setLED(i, (currentR > i) ? currentR : 0, (currentG > i) ? currentG : 0, (currentB > i) ? currentB : 0); // Adjust for your LED index
        	} else {
        		setLED(i, 0, 0, 0); // Adjust for your LED index
        	}
        }
        setBrightness(GLOBAL_BRIGHTNESS);
		WS2812_Update(); // Update the LEDs
//        HAL_Delay(2); // Delay for a smooth transition effect
    }
}

int get_index(int matrixIndex, int x, int y) {
    // Determine the matrix index and local x-position based on the column (x)
    int matrixIndexa = (x < 22) ? 0 : 1; // First matrix for x < 22, second matrix for x >= 22
    int localX = (x < 22) ? x : (x - 22); // Calculate local X in the matrix (0 to 21)

    // Flip the row order every row
    if ((y) % 2 == 1) {
        // Reverse the order for every odd row (flipping left and right)
        localX = 21 - localX; // Flip horizontally
    }

    // Return the index within the matrix
    return y * 22 + localX + (matrixIndexa * 22 * 22);
}



const uint8_t digitPatterns[10][7] = {
    {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}, // 0
    {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111}, // 1
    {0b11110, 0b00001, 0b00001, 0b11110, 0b10000, 0b10000, 0b11111}, // 2
    {0b11110, 0b00001, 0b00001, 0b11110, 0b00001, 0b00001, 0b11110}, // 3
    {0b10001, 0b10001, 0b10001, 0b11111, 0b00001, 0b00001, 0b00001}, // 4
    {0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001, 0b11110}, // 5
    {0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}, // 6
    {0b11111, 0b00001, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100}, // 7
    {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}, // 8
    {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00001, 0b11110}  // 9
};

// Function to light up LEDs in a specific digit pattern (e.g., a 7-segment display mapping)
void displayDigit(int matrixIndex, int x, int y, uint8_t digit) {
    // Loop through each row of the digit pattern (5 rows)
    for (int row = 0; row < 7; row++) {
        uint8_t rowPattern = digitPatterns[digit][row];

        for (int col = 0; col < 5; col++) {
            // Determine the global x-position in the 44x22 matrix
            int globalX = x + col;

            // Determine the matrix index and the local x-coordinate within that matrix
            matrixIndex = globalX / 22; // Matrix index (0 or 1)

            // Calculate the index within the matrix using the updated get_index
            int index = get_index(matrixIndex, globalX, y + row);

            // Set the LED based on the pattern
            if (rowPattern & (1 << (4 - col))) {
                setLED(index, 255, 255, 255); // Turn on the LED (white color)
            }
        }
    }
}



void displayTime(uint32_t elapsedTime, uint32_t offsetX, uint32_t offsetY) {
    // Calculate hours, minutes, and seconds from elapsedTime (in milliseconds)

	RTC_TimeTypeDef sTime = {0};
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    uint8_t seconds = sTime.Seconds;
    uint8_t minutes = sTime.Minutes;
    uint8_t hours = sTime.Hours % 12;

    // Convert hours, minutes, and seconds to strings
    char timeString[9]; // Format: "HH:MM:SS"
    snprintf(timeString, sizeof(timeString), "%02u:%02u:%02u", hours, minutes, seconds);

    uint32_t seconds_holder = seconds * 44;
    seconds_holder = seconds_holder / 60;
    for (int y = 0; y < 22; y++) {
		for (int x = 0; x < seconds_holder; x++) {
			setLED(get_index(0, x, y), x * 122 / 44, y*122/22, (44-x) * 122 / 44);
		}
	}

    displayDigit(0, 0 + offsetX, 0 + offsetY, timeString[0] - '0');
    displayDigit(0, 6 + offsetX, 0 + offsetY, timeString[1] - '0');
	setLED(get_index(0,12 + offsetX, 1 + offsetY), 255, 255, 255);
	setLED(get_index(0,12 + offsetX, 2 + offsetY), 255, 255, 255);
	setLED(get_index(0,12 + offsetX, 4 + offsetY), 255, 255, 255);
	setLED(get_index(0,12 + offsetX, 5 + offsetY), 255, 255, 255);
	setLED(get_index(0,13 + offsetX, 1 + offsetY), 255, 255, 255);
	setLED(get_index(0,13 + offsetX, 2 + offsetY), 255, 255, 255);
	setLED(get_index(0,13 + offsetX, 4 + offsetY), 255, 255, 255);
	setLED(get_index(0,13 + offsetX, 5 + offsetY), 255, 255, 255);
    displayDigit(0, 15 + offsetX, 0 + offsetY, timeString[3] - '0');
    displayDigit(0, 21 + offsetX, 0 + offsetY, timeString[4] - '0');
    setLED(get_index(0,27 + offsetX, 1 + offsetY), 255, 255, 255);
	setLED(get_index(0,27 + offsetX, 2 + offsetY), 255, 255, 255);
	setLED(get_index(0,27 + offsetX, 4 + offsetY), 255, 255, 255);
	setLED(get_index(0,27 + offsetX, 5 + offsetY), 255, 255, 255);
	setLED(get_index(0,28 + offsetX, 1 + offsetY), 255, 255, 255);
	setLED(get_index(0,28 + offsetX, 2 + offsetY), 255, 255, 255);
	setLED(get_index(0,28 + offsetX, 4 + offsetY), 255, 255, 255);
	setLED(get_index(0,28 + offsetX, 5 + offsetY), 255, 255, 255);
    displayDigit(0, 30 + offsetX, 0 + offsetY, timeString[6] - '0');
    displayDigit(0, 36 + offsetX, 0 + offsetY, timeString[7] - '0');


}

uint32_t convertTimeToMilliseconds(int hours, int minutes, int seconds) {
    // Convert the hours and minutes to milliseconds and add the seconds converted to milliseconds
    return (hours * 3600 + minutes * 60 + seconds) * 1000;
}

void displayRainingEffect() {
    static int positions[44]; // Array to hold current "rain" positions for each column
    static int initialized = 0;

    if (!initialized) {
        // Initialize the positions array with random starting points for each column
        for (int x = 0; x < 44; x++) {
            positions[x] = rand() % 22;
        }
        initialized = 1;
    }

    // Clear the matrix by setting all LEDs to black (off)
    for (int y = 0; y < 22; y++) {
        for (int x = 0; x < 44; x++) {
            setLED(get_index(0, x, y), 0, 0, 0); // Turn off the LED
        }
    }

    // Update the positions for the rain
    for (int x = 0; x < 44; x++) {
        // Randomly decide if we should "fall" the rain or reset it
        if (rand() % 40 > 3 || (positions[x] < 34)) { // Adjust the probability (higher value for less frequent rain)
            positions[x] = (positions[x] + 1);
        } else {
            positions[x] = 0; // Reset to top for a "new rain drop"
        }

        // Set the LED at the current position to green (for the rain)
        int y = positions[x];
        for (int i = y; i >= 0; i--) {
        	setLED(get_index(1, x, i), 0, 255/(y-i+1), 0); // Green color for the rain
        	if ((y-i) == 10) {
        		break;
        	}
        }
    }
}

void clearMatrix() {
    for (int y = 0; y < 22; y++) {
        for (int x = 0; x < 44; x++) {
            // Set each LED to 0 (turn off)
            setLED((y * 44 + x), 0, 0, 0);
        }
    }
}

// Function to display a firework burst with a more dynamic effect
void displayFirework(int centerX, int centerY, int colorR, int colorG, int colorB) {
    // Define the maximum radius for the expansion
    const int maxRadius = 10;

    // Loop to create particles expanding outward
    for (int r = 0; r <= maxRadius; r++) {
        // Loop to create particles at this radius distance
        for (int angle = 0; angle < 360; angle += 10) { // 10-degree increments for a full circle
            // Convert angle to radians
            float angleRad = angle * (3.14159 / 180.0);

            // Calculate new particle position based on the angle and radius
            int newX = centerX + (int)(r * cos(angleRad));
            int newY = centerY + (int)(r * sin(angleRad));

            // Check if the new position is within matrix boundaries
            if (newX >= 0 && newX < 44 && (newY+r) >= 0 && (newY+r) < 22) {
                // Gradually fade the color as particles move outward
                int fadeFactor = (255 * (maxRadius - r)) / maxRadius;

                // Ensure the color is within bounds (0 to 255)
                int finalR = (colorR * fadeFactor) / 255;
                int finalG = (colorG * fadeFactor) / 255;
                int finalB = (colorB * fadeFactor) / 255;

                // Set the LED for this particle with the faded color
                if (newY + r < 22) {
                	setLED(get_index(1, newX, newY+r), finalR, finalG, finalB);
                }
            }
        }
        setBrightness(GLOBAL_BRIGHTNESS);
        WS2812_Update(); // Update the LEDs
        HAL_Delay(DELAY_MS);
        clearMatrix();
    }
}


// Function to simulate a fireworks display
void simulateFireworks() {
    // Randomly generate a position for the firework burst
    int fireworkX = rand() % 44;
    int fireworkY = rand() % 22;

    // Randomly select a color for the firework (e.g., red, green, blue)
    int colorR = rand() % 256;
    int colorG = rand() % 256;
    int colorB = rand() % 256;

    // Clear the matrix before drawing the next frame
    clearMatrix();

    // Display the firework at the generated position with the selected color
    displayFirework(fireworkX, fireworkY, colorR, colorG, colorB);
}

void initializeSparkles() {
    for (int i = 0; i < maxSparkles; i++) {
        sparkles[i].x = rand() % matrixWidth;
        sparkles[i].y = rand() % matrixHeight;
        sparkles[i].r = rand() % 256;
        sparkles[i].g = rand() % 256;
        sparkles[i].b = rand() % 256;
        sparkles[i].intensity = (double)rand()/RAND_MAX; // Start with no intensity
        sparkles[i].fadingOut = rand() % 2;
    }
}

void updateSparkles() {
	for (int i = 0; i < maxSparkles; i++) {
		Sparkle *s = &sparkles[i];

		// Update intensity based on whether it's fading in or out
		if (s->fadingOut) {
			s->intensity -= 0.05; // Decrease intensity
			if (s->intensity <= 0.0) {
				s->intensity = 0.0;
				s->fadingOut = 0; // Start fading in again
				s->x = rand() % matrixWidth;
				s->y = rand() % matrixHeight;
				s->r = rand() % 256;
				s->g = rand() % 256;
				s->b = rand() % 256;
			}
		} else {
			s->intensity += 0.05; // Increase intensity
			if (s->intensity >= 1.0) {
				s->intensity = 1.0;
				s->fadingOut = 1; // Start fading out
			}
		}

		// Set the LED color based on the current intensity
		int r = (int)(s->r * s->intensity);
		int g = (int)(s->g * s->intensity);
		int b = (int)(s->b * s->intensity);

		// Set the LED at the current sparkle position with the calculated color
		setLED(get_index(1, s->x, s->y), r, g, b);
	}
}

void display395() {
	const uint8_t dis395[12][24] = {
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,0},
		{0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0},
		{0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,0},
		{0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,0},
		{0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,0,0},
		{0,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,1,1,1,1,1,1,0},
		{0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0},
		{0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,0},
		{0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0},
		{0,1,1,1,1,1,1,0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
	};

	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 24; j++) {
			if (dis395[i][j] == 1) {
				setLED(get_index(1,j+10,i+5), 255, 255, 255);
			} else {
				setLED(get_index(1,j+10,i+5), 0, 0, 0);
			}
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
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //  uint8_t red[3] = {255, 0, 0};
  //  uint8_t green[3] = {0, 255, 0};
  //  uint8_t blue[3] = {0, 0, 255};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  initializeSparkles();

  RTC_TimeTypeDef sTimeInitt;
  sTimeInitt.Hours = 4;
  sTimeInitt.Minutes = 1;
  sTimeInitt.Seconds = 0;
  sTimeInitt.TimeFormat = RTC_HOURFORMAT12_AM;
  HAL_RTC_SetTime(&hrtc, &sTimeInitt, RTC_FORMAT_BIN);

  RTC_TimeTypeDef sTimeUpd;

  int i = 0;
  int effect = 0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  clearMatrix();
	  switch (effect){
		case 0:
			displayTime(convertTimeToMilliseconds(2,45,50) + i, 1, 3);
			break;
		case 1:
			displayRainingEffect();
			break;
		case 2:
			simulateFireworks();
			break;
		case 3:
			updateSparkles();
			break;
		case 4:
			updateSparkles();
			display395();
	  }
	  setBrightness(GLOBAL_BRIGHTNESS);
	  WS2812_Update(); // Update the LEDs
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_Delay(DELAY_MS);
	  i = i + DELAY_MS * 1.05;

//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//	  HAL_Delay(500);

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
		  HAL_Delay(5);
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
			  effect = (effect+1) % 5;
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
				  HAL_Delay(5);
			  }
		  }
	  }

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
		  HAL_Delay(5);
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
			  HAL_RTC_GetTime(&hrtc, &sTimeUpd, RTC_FORMAT_BIN);
			  sTimeUpd.Minutes = (sTimeUpd.Minutes + 1) % 60;
			  HAL_RTC_SetTime(&hrtc, &sTimeUpd, RTC_FORMAT_BIN);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET){
				  HAL_Delay(5);
			  }
		  }
	  }

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
		  HAL_Delay(5);
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
			  HAL_RTC_GetTime(&hrtc, &sTimeUpd, RTC_FORMAT_BIN);
			  sTimeUpd.Hours = (sTimeUpd.Hours + 1) % 12;
			  HAL_RTC_SetTime(&hrtc, &sTimeUpd, RTC_FORMAT_BIN);
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET){
				  HAL_Delay(5);
			  }
		  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
