/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hexapod mainboard — IMU + ToF readout + dual PCA9685 servo control
  *
  * UART commands (115200 8N1, terminated with \r or \n):
  *   R<n> <angle>   — set servo n (1-9) on Right board to angle (0-180)
  *   L<n> <angle>   — set servo n (1-9) on Left  board to angle (0-180)
  *   STATUS         — print one sensor snapshot
  *   STREAM         — toggle continuous sensor output (~10 Hz)
  *   CENTER         — move all servos to 90° (both boards)
  *
  * Examples:
  *   R3 90    → right servo 3 to 90°
  *   L5 45    → left servo 5 to 45°
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "pca9685.h"
#include "imu.h"
#include "tof.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Servo boards */
static PCA9685_t pca_right;
static PCA9685_t pca_left;

/* IMU */
static stmdev_ctx_t imu_ctx;
static IMU_Data_t   imu_data;

/* ToF */
static TOF_Data_t tof_data;

/* UART receive — polling, byte by byte */
#define CMD_BUF_LEN 32
static char    cmd_buf[CMD_BUF_LEN];
static uint8_t cmd_idx   = 0;
static uint8_t cmd_ready = 0;

/* Sensor streaming */
static uint8_t  stream_on      = 0;
static uint32_t next_stream_ms = 0;
#define STREAM_INTERVAL_MS  100u   /* 10 Hz */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static void process_command(const char *cmd);
static void center_all_servos(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* printf → UART retarget */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ── UART polling receive — call every main loop iteration ─────────────── */
static void uart_poll(void)
{
    uint8_t byte;
    if (HAL_UART_Receive(&huart1, &byte, 1, 1) != HAL_OK)
        return;   /* no byte within 1 ms */

    char c = (char)byte;

    /* Echo the character back so the user sees what they typed */
    HAL_UART_Transmit(&huart1, &byte, 1, 10);

    if (c == '\r' || c == '\n') {
        /* Echo newline */
        uint8_t crlf[2] = {'\r', '\n'};
        HAL_UART_Transmit(&huart1, crlf, 2, 10);

        if (cmd_idx > 0) {
            cmd_buf[cmd_idx] = '\0';
            cmd_idx   = 0;
            cmd_ready = 1;
        }
    } else if (c == 127 || c == '\b') {
        /* Backspace — remove last char */
        if (cmd_idx > 0) cmd_idx--;
    } else if (cmd_idx < CMD_BUF_LEN - 1) {
        cmd_buf[cmd_idx++] = c;
    }
}

/* ── Command processing ─────────────────────────────────────────────────── */

static void process_command(const char *cmd)
{
    char side;
    int  index;
    float angle;

    /* Convert first char to upper for case-insensitive match */
    char first = (char)toupper((unsigned char)cmd[0]);

    if (first == 'R' || first == 'L') {
        /* Servo command: R<n> <angle>  or  L<n> <angle> */
        if (sscanf(cmd + 1, "%d %f", &index, &angle) == 2) {
            if (index < 1 || index > 9) {
                printf("ERR: servo index must be 1-9\r\n");
                return;
            }
            side = first;
            PCA9685_t *board = (side == 'R') ? &pca_right : &pca_left;
            HAL_StatusTypeDef s = PCA9685_SetServoAngle(board,
                                      (uint8_t)(index - 1), angle);
            if (s == HAL_OK)
                printf("OK: %c%d = %.1f deg\r\n", side, index, angle);
            else
                printf("ERR: I2C failed for %c%d\r\n", side, index);
        } else {
            printf("ERR: format is R<1-9> <0-180>  e.g. R3 90\r\n");
        }

    } else if (strncmp(cmd, "STATUS", 6) == 0 ||
               strncmp(cmd, "status", 6) == 0) {
        IMU_Read(&imu_ctx, &imu_data);
        IMU_Print(&imu_data);
        int8_t got = TOF_Read(&tof_data);
        if (got == 1)
            TOF_Print(&tof_data);
        else
            printf("TOF | no data\r\n");

    } else if (strncmp(cmd, "STREAM", 6) == 0 ||
               strncmp(cmd, "stream", 6) == 0) {
        stream_on = !stream_on;
        printf("STREAM %s\r\n", stream_on ? "ON" : "OFF");

    } else if (strncmp(cmd, "CENTER", 6) == 0 ||
               strncmp(cmd, "center", 6) == 0) {
        center_all_servos();
        printf("OK: all servos centered\r\n");

    } else {
        printf("ERR: unknown command '%s'\r\n", cmd);
        printf("     Valid: R<1-9> <deg>, L<1-9> <deg>, STATUS, STREAM, CENTER\r\n");
    }
}

static void center_all_servos(void)
{
    for (uint8_t ch = 0; ch < 9; ch++) {
        PCA9685_SetServoAngle(&pca_right, ch, 90.0f);
        PCA9685_SetServoAngle(&pca_left,  ch, 90.0f);
    }
}

/* USER CODE END 0 */

int main(void)
{
    MPU_Config();
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\n=== Hexapod Mainboard ===\r\n");

    /* ── Enable servo driver OE pins (active-low on your board) ── */
    HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_RESET);

    /* ── PCA9685 Right ────────────────────────────────────────── */
    pca_right.hi2c   = &hi2c1;
    pca_right.addr   = PCA9685_ADDR_RIGHT;
    pca_right.freq_hz = 50.0f;
    pca_right.min_us = PCA9685_SERVO_MIN_US;
    pca_right.max_us = PCA9685_SERVO_MAX_US;

    if (PCA9685_Init(&pca_right) == HAL_OK)
        printf("PCA9685 Right: OK\r\n");
    else
        printf("PCA9685 Right: FAIL\r\n");

    /* ── PCA9685 Left ─────────────────────────────────────────── */
    pca_left.hi2c   = &hi2c1;
    pca_left.addr   = PCA9685_ADDR_LEFT;
    pca_left.freq_hz = 50.0f;
    pca_left.min_us = PCA9685_SERVO_MIN_US;
    pca_left.max_us = PCA9685_SERVO_MAX_US;

    if (PCA9685_Init(&pca_left) == HAL_OK)
        printf("PCA9685 Left:  OK\r\n");
    else
        printf("PCA9685 Left:  FAIL\r\n");

    /* Center all servos on boot */
    center_all_servos();

    /* ── IMU ──────────────────────────────────────────────────── */
    if (IMU_Init(&imu_ctx, &hi2c1) == 0)
        printf("LSM6DSO IMU:   OK\r\n");
    else
        printf("LSM6DSO IMU:   FAIL\r\n");

    /* ── ToF ──────────────────────────────────────────────────── */
    if (TOF_Init(&hi2c1) == 0)
        printf("VL53L1X ToF:   OK\r\n");
    else
        printf("VL53L1X ToF:   FAIL\r\n");

    printf("\r\nReady. Send commands (e.g. R3 90, L5 45, STATUS, STREAM)\r\n> ");
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        /* USER CODE BEGIN WHILE */

        /* ── Poll for incoming UART bytes ──────────────────────── */
        uart_poll();

        /* ── Handle completed command ───────────────────────────── */
        if (cmd_ready) {
            cmd_ready = 0;
            process_command(cmd_buf);
            printf("> ");   /* prompt for next command */
        }

        /* ── Streaming sensor output ───────────────────────────── */
        if (stream_on && HAL_GetTick() >= next_stream_ms) {
            next_stream_ms = HAL_GetTick() + STREAM_INTERVAL_MS;

            IMU_Read(&imu_ctx, &imu_data);
            IMU_Print(&imu_data);

            int8_t got = TOF_Read(&tof_data);
            if (got == 1) TOF_Print(&tof_data);
        }

        /* ── Heartbeat LED ─────────────────────────────────────── */
        static uint32_t last_led = 0;
        if (HAL_GetTick() - last_led >= 500) {
            last_led = HAL_GetTick();
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }

        /* USER CODE END WHILE */
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV2;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                     | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/**
  * @brief Peripherals Common Clock Configuration
  */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M           = 2;
    PeriphClkInitStruct.PLL2.PLL2N           = 10;
    PeriphClkInitStruct.PLL2.PLL2P           = 2;
    PeriphClkInitStruct.PLL2.PLL2Q           = 2;
    PeriphClkInitStruct.PLL2.PLL2R           = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL      = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN       = 0;
    PeriphClkInitStruct.AdcClockSelection    = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) Error_Handler();
}

/**
  * @brief ADC1 Initialization
  */
static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode      = DISABLE;
    hadc1.Init.Oversampling.Ratio    = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

    sConfig.Channel              = ADC_CHANNEL_16;
    sConfig.Rank                 = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime         = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff           = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber         = ADC_OFFSET_NONE;
    sConfig.Offset               = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief ADC2 Initialization
  */
static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait      = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc2.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc2.Init.OversamplingMode      = DISABLE;
    hadc2.Init.Oversampling.Ratio    = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    sConfig.Channel              = ADC_CHANNEL_10;
    sConfig.Rank                 = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime         = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff           = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber         = ADC_OFFSET_NONE;
    sConfig.Offset               = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief ADC3 Initialization
  */
static void MX_ADC3_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
    hadc3.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc3.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait      = DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode      = DISABLE;
    hadc3.Init.Oversampling.Ratio    = 1;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();

    sConfig.Channel              = ADC_CHANNEL_5;
    sConfig.Rank                 = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime         = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff           = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber         = ADC_OFFSET_NONE;
    sConfig.Offset               = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief I2C1 Initialization
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00303D5B;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/**
  * @brief USART1 Initialization
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance            = USART1;
    huart1.Init.BaudRate       = 115200;
    huart1.Init.WordLength     = UART_WORDLENGTH_8B;
    huart1.Init.StopBits       = UART_STOPBITS_1;
    huart1.Init.Parity         = UART_PARITY_NONE;
    huart1.Init.Mode           = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin | LED_RED_Pin | Right_A2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Left_A2_Pin | Left_A1_Pin | Left_A0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, Right_Enable_Pin | Right_A1_Pin | Right_A0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, XSHUT_Pin | LED_CAM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Left_Enable_GPIO_Port, Left_Enable_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = LED_GREEN_Pin | LED_RED_Pin | Right_A2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Left_A2_Pin | Left_A1_Pin | Left_A0_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Right_Enable_Pin | Right_A1_Pin | Right_A0_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = User_Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XSHUT_Pin | LED_CAM_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_1_TOF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_1_TOF_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = Left_Enable_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Left_Enable_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = DIN_WS2812B_1_Pin | DIN_WS2812B_2_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
