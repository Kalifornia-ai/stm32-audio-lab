#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_sdram.h"
#include "wm8994.h"
#include <stdio.h>

/* UART for printf */
UART_HandleTypeDef huart1;

/* Tone data */
#define LOOPLENGTH 8
#define AUDIO_FREQ 8000
int16_t sine_table[LOOPLENGTH] = {0, 7071, 10000, 7071, 0, -7071, -10000, -7071};
int16_t stereo_buf[LOOPLENGTH * 2];
volatile uint8_t PlayComplete = 0;

/* Prototypes */
static void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void LCD_Backlight_Off(void);
int _write(int file, char *ptr, int len);

/* HAL tick so HAL_Delay works */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/* Turn off white LCD backlight so it's not painful */
static void LCD_Backlight_Off(void) {
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
 * Override weak clock config in BSP so SAI2 gets a valid MCLK
 * for low sample rates like 8 kHz.
 */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
    RCC_PeriphCLKInitTypeDef clkcfg;
    HAL_RCCEx_GetPeriphCLKConfig(&clkcfg);

    clkcfg.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    clkcfg.Sai2ClockSelection   = RCC_SAI2CLKSOURCE_PLLI2S;

    /* For ~8 kHz playback */
    if (AudioFreq == AUDIO_FREQ || AudioFreq == 8000) {
        clkcfg.PLLI2S.PLLI2SN    = 256;
        clkcfg.PLLI2S.PLLI2SQ    = 5;
        clkcfg.PLLI2SDivQ        = 25;
    } else {
        /* fallback defaults for ~44.1k family */
        clkcfg.PLLI2S.PLLI2SN    = 429;
        clkcfg.PLLI2S.PLLI2SQ    = 2;
        clkcfg.PLLI2SDivQ        = 19;
    }

    HAL_RCCEx_PeriphCLKConfig(&clkcfg);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    setvbuf(stdout, NULL, _IONBF, 0);

    LCD_Backlight_Off();

    printf("Init SDRAM...\r\n");
    if (BSP_SDRAM_Init() != HAL_OK) {
        printf("SDRAM init failed\r\n");
        while (1) { }
    }
    printf("SDRAM init OK\r\n");

    printf("Prep audio buffer...\r\n");
    for (int i = 0; i < LOOPLENGTH; i++) {
        stereo_buf[2*i]     = sine_table[i];
        stereo_buf[2*i + 1] = sine_table[i];
    }

    printf("Init audio...\r\n");
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, AUDIO_FREQ) != AUDIO_OK) {
        printf("Audio init failed\r\n");
        while (1) { }
    }
    printf("Audio init OK\r\n");

    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    printf("Start playback...\r\n");
    if (BSP_AUDIO_OUT_Play((uint16_t*)stereo_buf, sizeof(stereo_buf)) != AUDIO_OK) {
        printf("Audio play failed\r\n");
        while (1) { }
    }
    printf("Playing tone...\r\n");

    while (1) {
        if (PlayComplete) {
            PlayComplete = 0;
            printf("Transfer complete\r\n");
        }
        HAL_Delay(200);
    }
}


/* DMA complete callback from BSP */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    PlayComplete = 1;
}

/* UART redirect for printf */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}


void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C4)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_I2C4_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};

        // On STM32F746G-DISCO:
        // PF14 -> I2C4_SCL
        // PF15 -> I2C4_SDA
        gpio.Pin       = GPIO_PIN_14 | GPIO_PIN_15;
        gpio.Mode      = GPIO_MODE_AF_OD;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF4_I2C4;
        HAL_GPIO_Init(GPIOF, &gpio);
    }
}

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
    static DMA_HandleTypeDef hdma_sai2_a;

    if (hsai->Instance == SAI2_Block_A)
    {
        // 1. Enable clocks
        __HAL_RCC_SAI2_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};

        // Pins used by SAI2 Block A on this board:
        // PI4  = SAI2_MCLK_A
        // PI5  = SAI2_SCK_A
        // PI6  = SAI2_SD_A
        // PG10 = SAI2_FS_A

        // PI4, PI5, PI6 -> AF10_SAI2
        gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF10_SAI2;
        HAL_GPIO_Init(GPIOI, &gpio);

        // PG10 -> AF10_SAI2
        gpio.Pin = GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF10_SAI2;
        HAL_GPIO_Init(GPIOG, &gpio);

        // 2. Configure DMA for SAI2 Block A TX
        // Discovery F746G typical config:
        // DMA2_Stream4, Channel 3 -> SAI2_A transmit
        hdma_sai2_a.Instance = DMA2_Stream4;
        hdma_sai2_a.Init.Channel             = DMA_CHANNEL_3;
        hdma_sai2_a.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_sai2_a.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_sai2_a.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_sai2_a.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_sai2_a.Init.Mode                = DMA_CIRCULAR;
        hdma_sai2_a.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_sai2_a.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK)
        {
            // if DMA init fails, we're in trouble
            while (1) { }
        }

        // Link DMA handle to SAI handle's tx side
        __HAL_LINKDMA(hsai, hdmatx, hdma_sai2_a);

        // 3. Enable NVIC for DMA interrupt
        HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0x0F, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

        // Also set interrupt priority for SAI2 itself (optional but good)
        HAL_NVIC_SetPriority(SAI2_IRQn, 0x0F, 0);
        HAL_NVIC_EnableIRQ(SAI2_IRQn);
    }
}

void DMA2_Stream4_IRQn_Handler(void)
{
    // HAL uses a slightly different default naming: DMA2_Stream4_IRQHandler
}

void DMA2_Stream4_IRQHandler(void)
{
    // The BSP's SAI handle is usually global/static inside stm32746g_discovery_audio.c.
    // We need to extern it so we can call HAL_DMA_IRQHandler on it.

    extern SAI_HandleTypeDef haudio_out_sai; // this symbol is in stm32746g_discovery_audio.c
    HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}




/* USART1 on PA9/PA10 -> ST-LINK VCP */
static void MX_USART1_UART_Init(void) {
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

/* PLL clocks @216 MHz using HSI -> same as before */
static void SystemClock_Config(void) {
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
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7);
}
