/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Buck-Boost DC/DC Converter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Operating mode of the converter
 *
 * Only MODE_BUCK is currently implemented
 */
typedef enum {
    MODE_BUCK = 0,
    MODE_BOOST,
    MODE_BUCK_BOOST
} ConverterMode_t;

/**
 * @brief Internal state machine of the converter
 *
 * STATE_IDLE      : VIN below UVLO threshold — PWM off
 * STATE_SOFTSTART : PWM ramping up toward the target
 * STATE_RUNNING   : PI controller fully active
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_SOFTSTART,
    STATE_RUNNING
} ConverterState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* TIM1 period register (ARR) */
#define TIM1_ARR            566U

/* UVLO thresholds in mV */
#define UVLO_ON_MV          12000U
#define UVLO_OFF_MV         10000U

/* ADC conversion factors */
#define ADC_VREF_MV         3000.0f
#define ADC_FULL_SCALE      4095.0f
#define VIN_DIVIDER_FACTOR  13.4f
#define VOUT_DIVIDER_FACTOR 5.02f

/* VOUT limits in mV */
#define VOUT_MIN_MV         3000U
#define VOUT_MAX_MV         15000U

/* Boost maximum duty */
#define DUTY_BOOST_MAX      550

/* Soft-start: one duty step every N control-loop cycles */
#define SOFTSTART_STEP_PERIOD   4U

/* USB data send interval in ms */
#define USB_SEND_INTERVAL_MS    100U

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* ── Converter configuration ─────────────────────────────────────────────── */
static ConverterMode_t  converter_mode  = MODE_BUCK;
static ConverterState_t converter_state = STATE_IDLE;

/* ── Measurements ────────────────────── */
volatile uint32_t vin_mv  = 0;
volatile uint32_t vout_mv = 0;

/* ── Setpoint (may be updated from USB) ──────────────────────────────────── */
volatile uint32_t vout_target_mv = 5000;

/* ── PI controller state ─────────────────────────────────────────────────── */
volatile float Kp             = 0.1f;
volatile float Ki             = 0.001f;
volatile float pi_integral    = 0.0f;
volatile float pi_integral_max = 100000.0f;

/* ── PWM duty cycle ─────────────────────────────────────── */
volatile int32_t duty = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

/* USER CODE BEGIN PFP */
static void     mode_init(ConverterMode_t mode);
static void     measurements_update(void);
static uint8_t  uvlo_check(void);
static void     setpoint_check(void);
static int32_t  compute_feedforward(void);
static int32_t  pi_update(int32_t error);
static int32_t  duty_clamp(int32_t raw_duty);
static void     softstart_update(int32_t duty_target);
static void     pwm_apply(void);
static void     pwm_stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Configure GPIOs for the selected converter mode
 *
 * Buck  : Q3 (PA9) permanently ON, Q4 (PB14) permanently OFF
 *         Q1/Q2 switch via TIM1 CH1/CH1N
 * Boost : Q1 (PA8) permanently ON, Q2 (PB13) permanently OFF
 *         Q3/Q4 switch via TIM1 CH2/CH2N
 */
static void mode_init(ConverterMode_t mode)
{
    switch (mode)
    {
        case MODE_BUCK:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_SET);   /* Q3 ON  */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); /* Q4 OFF */
            break;

        case MODE_BOOST:
            /* TODO */
            break;

        case MODE_BUCK_BOOST:
            /* TODO */
            break;
    }
}

/**
 * @brief Read ADC dual-simultaneous result and convert to millivolts
 *
 * CDR bits [15:0]  = ADC1 (VIN),  bits [31:16] = ADC2 (VOUT)
 */
static void measurements_update(void)
{
    uint32_t cdr = ADC12_COMMON->CDR;
    uint32_t vin_raw  = cdr & 0xFFFFU;
    uint32_t vout_raw = (cdr >> 16) & 0xFFFFU;

    vin_mv  = (uint32_t)(vin_raw  / ADC_FULL_SCALE * ADC_VREF_MV * VIN_DIVIDER_FACTOR);
    vout_mv = (uint32_t)(vout_raw / ADC_FULL_SCALE * ADC_VREF_MV * VOUT_DIVIDER_FACTOR);
}

/**
 * @brief Evaluate UVLO condition and update converter state accordingly
 *
 * @return 1 if the converter must stay in IDLE
 *         0 if VIN is within operating range
 */
static uint8_t uvlo_check(void)
{
    if (vin_mv >= UVLO_ON_MV)
    {
        if (converter_state == STATE_IDLE)
        {
            /* VIN just crossed the ON threshold — begin soft-start */
            duty         = 0;
            pi_integral  = 0.0f;
            converter_state = STATE_SOFTSTART;
        }
        return 0;
    }

    if (vin_mv < UVLO_OFF_MV)
    {
        converter_state = STATE_IDLE;
        duty            = 0;
        pi_integral     = 0.0f;
        pwm_stop();
        return 1;
    }

    /* Hysteresis band: keep current state */
    return (converter_state == STATE_IDLE) ? 1U : 0U;
}

/**
 * @brief Clamp vout_target_mv to valid range and relaunch soft-start whenever the setpoint changes
 */
static void setpoint_check(void)
{
    if (vout_target_mv < VOUT_MIN_MV) vout_target_mv = VOUT_MIN_MV;
    if (vout_target_mv > VOUT_MAX_MV) vout_target_mv = VOUT_MAX_MV;

    /* Boost: target must stay above VIN */
    if (converter_mode == MODE_BOOST && vout_target_mv < vin_mv + 500U)
        vout_target_mv = vin_mv + 500U;

    static uint32_t prev_target_mv = 0;
    if (vout_target_mv != prev_target_mv)
    {
        prev_target_mv  = vout_target_mv;
        converter_state = STATE_SOFTSTART;
    }
}

/**
 * @brief Compute the open-loop feedforward duty cycle for the active mode
 *
 * The feedforward provides a starting point close to the correct duty so the PI only needs to correct the residual error
 *
 * @return Raw feedforward duty (not yet clamped)
 */
static int32_t compute_feedforward(void)
{
    if (vin_mv == 0) return 0;

    switch (converter_mode)
    {
        case MODE_BUCK:
            return (int32_t)((float)vout_target_mv / (float)vin_mv * TIM1_ARR);

        case MODE_BOOST:
            return (int32_t)(TIM1_ARR - (float)vin_mv / (float)vout_target_mv * TIM1_ARR);

        case MODE_BUCK_BOOST:
            return (int32_t)((float)vout_target_mv / ((float)vin_mv + (float)vout_target_mv) * TIM1_ARR);

        default:
            return 0;
    }
}

/**
 * @brief Run one PI iteration
 *
 * The same PI algorithm is used for all converter modes. Only the feedforward is mode-dependent
 *
 * @return PI correction term (to be added to feedforward)
 */
static int32_t pi_update(int32_t error)
{
    int32_t p_term = (int32_t)(Kp * (float)error);

    pi_integral += Ki * (float)error;
    if (pi_integral >  pi_integral_max) pi_integral =  pi_integral_max;
    if (pi_integral < -pi_integral_max) pi_integral = -pi_integral_max;

    int32_t i_term = (int32_t)pi_integral;

    return p_term + i_term;
}

/**
 * @brief Clamp a raw duty value to the valid range for the active mode
 */
static int32_t duty_clamp(int32_t raw_duty)
{
    int32_t max_duty;

    switch (converter_mode)
    {
        case MODE_BOOST:
            max_duty = DUTY_BOOST_MAX;
            break;
        case MODE_BUCK:
        case MODE_BUCK_BOOST:
        default:
            max_duty = (int32_t)TIM1_ARR;
            break;
    }

    if (raw_duty < 0)        return 0;
    if (raw_duty > max_duty) return max_duty;
    return raw_duty;
}

/**
 * @brief Advance the duty cycle toward duty_target by one soft-start step
 *
 * While in STATE_SOFTSTART, duty moves ±1 every SOFTSTART_STEP_PERIOD control cycles
 * Once duty == duty_target the state transitions to STATE_RUNNING and full PI control takes over
 */
static void softstart_update(int32_t duty_target)
{
    static uint8_t step_counter = 0;
    step_counter++;
    if (step_counter >= SOFTSTART_STEP_PERIOD)
    {
        step_counter = 0;
        if      (duty < duty_target) duty++;
        else if (duty > duty_target) duty--;
        else                         converter_state = STATE_RUNNING;
    }
}

/**
 * @brief Write the current duty value to the appropriate CCR register
 */
static void pwm_apply(void)
{
    switch (converter_mode)
    {
        case MODE_BUCK:
            TIM1->CCR1 = (uint32_t)duty;
            break;

        case MODE_BOOST:
            TIM1->CCR2 = (uint32_t)duty;
            break;

        case MODE_BUCK_BOOST:
            /* TODO */
            break;
    }
}

/**
 * @brief Force all PWM outputs to zero (UVLO or fault condition)
 */
static void pwm_stop(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
}

/**
 * @brief TIM1 period-elapsed ISR — main control loop (150 kHz)
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;

    /* 1. Measurements */
    measurements_update();

    /* 2. UVLO */
    if (uvlo_check()) return;

    /* 3. Clamp setpoint + detect setpoint change */
    setpoint_check();

    /* 4. Feedforward + PI */
    int32_t duty_new = compute_feedforward()
                     + pi_update((int32_t)vout_target_mv - (int32_t)vout_mv);
    duty_new = duty_clamp(duty_new);

    /* 5. Apply duty */
    if (converter_state == STATE_SOFTSTART)
        softstart_update(duty_new);
    else
        duty = duty_new;

    /* 6. Apply */
    pwm_apply();
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_USB_Device_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* USER CODE BEGIN 2 */

    /* Configure GPIOs for the selected mode */
    mode_init(converter_mode);

    /* Start PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    /* Start control-loop interrupt */
    HAL_TIM_Base_Start_IT(&htim1);

    /* Release PF0 (ADC1_IN10 / VIN) from HSE oscillator function */
    RCC->CR &= ~RCC_CR_HSEON;

    /* Calibrate and start ADC in dual simultaneous mode */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc1);

    uint32_t last_usb_send = 0;

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* USB data sent to web dashboard */
        if (HAL_GetTick() - last_usb_send >= USB_SEND_INTERVAL_MS)
        {
            last_usb_send = HAL_GetTick();

            char msg[80];
            sprintf(msg, "VIN:%u VOUT:%u TARGET:%u DUTY:%d\r\n",
                    (unsigned int)vin_mv,
                    (unsigned int)vout_mv,
                    (unsigned int)vout_target_mv,
                    (int)duty);
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        }

        /* USER CODE END 3 */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI
                                          | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State          = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef  multimode = {0};
    ADC_ChannelConfTypeDef sConfig  = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode             = ADC_DUALMODE_REGSIMULT;
    multimode.DMAAccessMode    = ADC_DMAACCESSMODE_DISABLED;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
        Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_10;       /* PF0 — VIN */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation      = 0;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait      = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_15;       /* PB15 — VOUT */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef       sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef      sMasterConfig      = {0};
    TIM_OC_InitTypeDef           sConfigOC          = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = TIM1_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;           /* ISR at 150 kHz */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE; /* ADC trigger */
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 283;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 20; /* ~118 ns @ 170 MHz */
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA4 = status LED (unused for now) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif
