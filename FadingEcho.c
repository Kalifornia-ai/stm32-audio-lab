#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_audio.h"
#include "wm8994.h"

#include <string.h>
#include <stdio.h>

/* ====== Audio config ====== */
#define AUDIO_FREQ 16000u
#define AUDIO_IN_BIT_RES 16u
#define AUDIO_IN_CHANNEL_NBR 2u

/* How many samples per "half-buffer" per channel */
#define BLOCK_SAMPLES_PER_CH 512u

#define DELAY_BUF_SIZE 16000u 
// #define GAIN 0.6f

#define ECHO_GAIN 0.6f

/* Derived sizes */
#define BLOCK_SAMPLES_TOTAL (BLOCK_SAMPLES_PER_CH * AUDIO_IN_CHANNEL_NBR)
#define BUF_SAMPLES (BLOCK_SAMPLES_TOTAL * 2u) // double buffer

/* DMA buffers (source: mic in, sink: headphone out) */
__attribute__((aligned(32))) static uint16_t InBuf[BUF_SAMPLES];
__attribute__((aligned(32))) static uint16_t OutBuf[BUF_SAMPLES];
/* Delay buffer for ~64 ms delay (same size as BUF_SAMPLES works fine) */
// __attribute__((aligned(32))) static uint16_t DelayBuf[BUF_SAMPLES];
// static uint32_t delayPtr = 0;

static int16_t EchoBuf[DELAY_BUF_SIZE];
static uint32_t echoPtr = 0;

/* Flags from DMA callbacks */
static volatile uint8_t InHalfComplete = 0;
static volatile uint8_t InFullComplete = 0;

/* UART for printf */
UART_HandleTypeDef huart1;

/* Prototypes */
static void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void LCD_Backlight_Off(void);
int _write(int file, char *ptr, int len);

/* HAL tick so HAL_Delay works */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Optional: turn off white backlight so it doesn't blind you */
static void LCD_Backlight_Off(void)
{
    __HAL_RCC_GPIOK_CLK_ENABLE();
    GPIO_InitTypeDef bl = {0};
    bl.Pin = GPIO_PIN_3;
    bl.Mode = GPIO_MODE_OUTPUT_PP;
    bl.Pull = GPIO_NOPULL;
    bl.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOK, &bl);
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);
}

/*
 * Override: set SAI2 clock for given sample rate.
 * You already used a version of this in the tone test.
 */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
    RCC_PeriphCLKInitTypeDef clkcfg;
    HAL_RCCEx_GetPeriphCLKConfig(&clkcfg);

    clkcfg.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    clkcfg.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;

    /* crude config: try 16 kHz path like 44.1k family branch */
    if (AudioFreq == AUDIO_FREQ)
    {
        /* We can tune PLLI2S if needed for cleaner 16k, but start with defaults */
        clkcfg.PLLI2S.PLLI2SN = 429;
        clkcfg.PLLI2S.PLLI2SQ = 2;
        clkcfg.PLLI2SDivQ = 19;
    }

    HAL_RCCEx_PeriphCLKConfig(&clkcfg);
}

/* DMA complete callbacks from BSP for AUDIO IN */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void) { InHalfComplete = 1; }
void BSP_AUDIO_IN_TransferComplete_CallBack(void) { InFullComplete = 1; }

/* DMA complete callback from BSP for AUDIO OUT (optional debug) */

void BSP_AUDIO_IN_Error_CallBack(void)
{
    // optional: printf("AUDIO IN ERROR\r\n");
}

/* MSP init for I2C4 (codec control) */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C4)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_I2C4_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        // PF14 = I2C4_SCL, PF15 = I2C4_SDA
        gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF4_I2C4;
        HAL_GPIO_Init(GPIOF, &gpio);
    }
}

void DMA2_Stream7_IRQHandler(void)
{
    extern SAI_HandleTypeDef haudio_in_sai;
    HAL_DMA_IRQHandler(haudio_in_sai.hdmarx);
}

void DMA2_Stream4_IRQHandler(void)
{
    extern SAI_HandleTypeDef haudio_out_sai;
    HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

/* UART retarget for printf */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USART1 -> ST-LINK VCP on PA9/PA10 */
static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

/* 216 MHz system clock from HSI PLL */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableOverDrive();

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 216;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7);
}

// static void ProcessDelayBlock(const uint16_t *in, uint16_t *out, uint32_t len)
// {
//     // len = BLOCK_SAMPLES_TOTAL (so len is "one half-buffer worth of uint16_t samples")
//     for (uint32_t i = 0; i < len; i++)
//     {
//         int16_t delayed = (int16_t)DelayBuf[delayPtr];
//         int16_t current = (int16_t)in[i];

//         int32_t mixed = (int32_t)current + (int32_t)delayed;

//         // simple saturate to int16 range
//         if (mixed > 32767)
//             mixed = 32767;
//         if (mixed < -32768)
//             mixed = -32768;

//         out[i] = (uint16_t)((int16_t)mixed);

//         // store the *current* sample into delay line for future use
//         DelayBuf[delayPtr] = (uint16_t)current;

//         // advance circular index
//         delayPtr++;
//         if (delayPtr >= BUF_SAMPLES)
//         {
//             delayPtr = 0;
//         }
//     }
// }

static void ProcessEchoBlock(const uint16_t *in, uint16_t *out, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        int16_t delayed = EchoBuf[echoPtr]; // old echoed sample
        int16_t current = (int16_t)in[i];

        // mix dry + scaled feedback
        float mixed_f = (float)current + (float)delayed * ECHO_GAIN;

        // clamp to int16
        if (mixed_f > 32767.0f)
            mixed_f = 32767.0f;
        if (mixed_f < -32768.0f)
            mixed_f = -32768.0f;

        int16_t mixed = (int16_t)mixed_f;

        // send to output
        out[i] = (uint16_t)mixed;

        // IMPORTANT: feedback the MIX, not just the dry input
        EchoBuf[echoPtr] = mixed;

        // wrap pointer
        echoPtr++;
        if (echoPtr >= DELAY_BUF_SIZE)
        {
            echoPtr = 0;
        }
    }
}

/* ====== main() ====== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    setvbuf(stdout, NULL, _IONBF, 0);

    LCD_Backlight_Off();

    printf("Init SDRAM...\r\n");
    if (BSP_SDRAM_Init() != HAL_OK)
    {
        printf("SDRAM init failed\r\n");
        while (1)
        {
        }
    }
    printf("SDRAM init OK\r\n");
    // memset(DelayBuf, 0, sizeof(DelayBuf));

    // Clear output buffer just to avoid garbage at startup
    memset(OutBuf, 0, sizeof(OutBuf));

    memset(EchoBuf, 0, sizeof(EchoBuf));

    printf("Init Audio In/Out...\r\n");
    if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                              OUTPUT_DEVICE_HEADPHONE,
                              AUDIO_FREQ,
                              AUDIO_IN_BIT_RES,
                              AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
    {
        printf("Audio IN/OUT init failed\r\n");
        while (1)
        {
        }
    }
    printf("Audio IN/OUT init OK\r\n");
    BSP_AUDIO_OUT_SetVolume(90);

    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    printf("Start OUT DMA...\r\n");
    if (BSP_AUDIO_OUT_Play(OutBuf, sizeof(OutBuf)) != AUDIO_OK)
    {
        printf("OUT play failed\r\n");
        while (1)
        {
        }
    }
    printf("OUT DMA running\r\n");

    printf("Start IN DMA...\r\n");
    if (BSP_AUDIO_IN_Record((uint16_t *)InBuf, BUF_SAMPLES) != AUDIO_OK)
    {
        printf("IN record failed\r\n");
        while (1)
        {
        }
    }
    printf("IN DMA running\r\n");

    printf("Loopback started\r\n");

    while (1)
    {
        if (InHalfComplete)
        {
            InHalfComplete = 0;
            ProcessEchoBlock(InBuf,
                             OutBuf,
                             BLOCK_SAMPLES_TOTAL);
        }

        if (InFullComplete)
        {
            InFullComplete = 0;
            ProcessEchoBlock(InBuf + BLOCK_SAMPLES_TOTAL,
                             OutBuf + BLOCK_SAMPLES_TOTAL,
                             BLOCK_SAMPLES_TOTAL);
        }
    }
}
