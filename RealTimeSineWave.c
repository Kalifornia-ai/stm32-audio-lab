#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_audio.h"
#include "wm8994.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/* ================= Audio Config =================
   For the tone generator we run DAC @ 8 kHz.
   Part 7 of the lab assumes fs = 8000 Hz.
*/
#define AUDIO_FREQ_OUT 8000u // playback sample rate (Hz)
#define AUDIO_BIT_RES 16u
#define AUDIO_CHANNELS 2u // we'll send stereo (L,R same)

/* ========== Tone LUT Settings (500 Hz example) ==========
   Formula: f_tone = fs / LOOPLENGTH
   For 500 Hz at fs=8000 Hz:
       LOOPLENGTH = 8000 / 500 = 16
*/
#define LOOPLENGTH 3u
#define TONE_AMPLITUDE 10000 // +/- amplitude in int16

/* We'll build a stereo buffer from this table and loop it. */

/* UART for debug printf */
UART_HandleTypeDef huart1;

/* Prototypes */
static void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void LCD_Backlight_Off(void);
int _write(int file, char *ptr, int len);

/* HAL tick for HAL_Delay() etc */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Optional: kill LCD backlight so it doesn't shine bright */
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
 * WM8994 / SAI clock config override.
 * BSP calls this internally when setting up audio out.
 * We set PLLI2S so that we can get ~8 kHz frame clock.
 * (These values are cribbed from the lab style: we reuse PLLI2S.)
 */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
    RCC_PeriphCLKInitTypeDef clkcfg;
    HAL_RCCEx_GetPeriphCLKConfig(&clkcfg);

    clkcfg.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    clkcfg.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;

    if (AudioFreq == AUDIO_FREQ_OUT)
    {
        /* These PLLI2S values are chosen to get an SAI clock
         * that divides down to ~8 kHz for the codec.
         * On STM32F7 disco boards this usually works well for
         * the low-rate lab tones.
         *
         * If audio sounds wrong or noisy, the divider math for
         * 8 kHz may need refinement (depends on BSP expectations).
         * But usually it's okay for lab work.
         */
        clkcfg.PLLI2S.PLLI2SN = 429;
        clkcfg.PLLI2S.PLLI2SQ = 2;
        clkcfg.PLLI2SDivQ = 19;
    }

    HAL_RCCEx_PeriphCLKConfig(&clkcfg);
}

/* MSP init for I2C4 (codec control bus) */
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

/* DMA IRQ handlers just in case BSP enables circular DMA on OUT */
void DMA2_Stream4_IRQHandler(void)
{
    extern SAI_HandleTypeDef haudio_out_sai;
    HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

/* Retarget printf to USART1 (ST-LINK VCP) */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

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

/* 216 MHz system clock from HSI PLL (like your echo code) */
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

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7);
}

/* =========================================================
   Build the LUT for one cycle of a 500 Hz sine at 8 kHz fs.
   LOOPLENGTH = 16 so tone = 8000/16 = 500 Hz.

   We'll generate the table at startup instead of hardcoding
   constants, so you can later change LOOPLENGTH easily.
   Then we'll duplicate to stereo and hand that buffer
   to BSP_AUDIO_OUT_Play() in circular mode.
   ========================================================= */

/* Mono, one full cycle */
static int16_t SineLUT[LOOPLENGTH];

/* Interleaved stereo buffer:
   We'll repeat the LUT a few times so DMA always has a chunk.
   For example:  LOOPLENGTH samples mono -> 2*LOOPLENGTH stereo samples.
   We'll tile that maybe 8 times to make a stable ring buffer.
*/
#define STEREO_FRAME_LEN (LOOPLENGTH * AUDIO_CHANNELS)
#define PLAYBACK_TILES 8u
static int16_t StereoOutBuf[STEREO_FRAME_LEN * PLAYBACK_TILES];

/* Helper: fill lookup and stereo buffer */
static void BuildToneBuffer(void)
{
    // 1) build the mono LUT
    for (uint32_t k = 0; k < LOOPLENGTH; k++)
    {
        float theta = 2.0f * 3.14159265359f * (float)k / (float)LOOPLENGTH;
        float s = sinf(theta); // -1..1
        int16_t val = (int16_t)((float)TONE_AMPLITUDE * s);
        SineLUT[k] = val;
    }

    // 2) expand to stereo + tile it
    uint32_t idx = 0;
    for (uint32_t t = 0; t < PLAYBACK_TILES; t++)
    {
        for (uint32_t k = 0; k < LOOPLENGTH; k++)
        {
            int16_t v = SineLUT[k];
            StereoOutBuf[idx++] = v; // Left
            StereoOutBuf[idx++] = v; // Right
        }
    }
}

/* ====================== main() ========================= */

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
    printf("SDRAM OK\r\n");

    // Build the tone data before starting audio output DMA
    BuildToneBuffer();

    printf("Init Audio OUT...\r\n");
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE,
                           90, // volume (0-100)
                           AUDIO_FREQ_OUT) != AUDIO_OK)
    {
        printf("AUDIO OUT init failed\r\n");
        while (1)
        {
        }
    }
    printf("Audio OUT init OK\r\n");

    // Some boards need explicit slot select for stereo on phones jack
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    printf("Start OUT DMA tone...\r\n");
    if (BSP_AUDIO_OUT_Play((uint16_t *)StereoOutBuf,
                           sizeof(StereoOutBuf)) != AUDIO_OK)
    {
        printf("Play failed\r\n");
        while (1)
        {
        }
    }
    printf("Tone playing\r\n");

    // We do NOT start BSP_AUDIO_IN_Record().
    // We just sit here while DMA loops the buffer forever.
    while (1)
    {
        // Idle loop. Could blink LED or print heartbeat.
        HAL_Delay(1000);
        // printf("still alive\r\n");
        for (uint32_t i = 0; i < sizeof(StereoOutBuf) / sizeof(StereoOutBuf[0]); i++)
        {
            printf("%d\r\n", StereoOutBuf[i]);
        }
    }
}
