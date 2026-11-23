#include <stdint.h>
#include <stdio.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* RCC */
#define RCC_CR         REG32(0x40021000UL)
#define RCC_AHB2ENR    REG32(0x4002104CUL)
#define RCC_APB2ENR    REG32(0x40021060UL)
#define RCC_CCIPR      REG32(0x40021088UL)

/* GPIOB: UART TX on PB6 */
#define GPIOB_MODER    REG32(0x48000400UL)
#define GPIOB_OTYPER   REG32(0x48000404UL)
#define GPIOB_OSPEEDR  REG32(0x48000408UL)
#define GPIOB_PUPDR    REG32(0x4800040CUL)
#define GPIOB_AFRL     REG32(0x48000420UL)

/* GPIOC: potentiometer on PC5 */
#define GPIOC_MODER    REG32(0x48000800UL)
#define GPIOC_PUPDR    REG32(0x4800080CUL)
#define GPIOC_ASCR     REG32(0x4800082CUL)

/* ADC1 */
#define ADC1_ISR       REG32(0x50040000UL)
#define ADC1_CR        REG32(0x50040008UL)
#define ADC1_CFGR      REG32(0x5004000CUL)
#define ADC1_SMPR2     REG32(0x50040018UL)
#define ADC1_SQR1      REG32(0x50040030UL)
#define ADC1_DR        REG32(0x50040040UL)
#define ADC_CCR        REG32(0x50040308UL)

/* USART1 */
#define USART1_CR1     REG32(0x40013800UL)
#define USART1_CR2     REG32(0x40013804UL)
#define USART1_CR3     REG32(0x40013808UL)
#define USART1_BRR     REG32(0x4001380CUL)
#define USART1_ISR     REG32(0x4001381CUL)
#define USART1_TDR     REG32(0x40013828UL)

/* Also available in the debugger */
volatile uint32_t x_raw;
volatile int32_t x_percent;
volatile uint32_t x_millivolts;

static void short_wait(void)
{
    for (volatile uint32_t i = 0; i < 10000U; ++i)
    {
        __asm volatile ("nop");
    }
}

static void uart_init(void)
{
    /* Enable HSI16 oscillator */
    RCC_CR |= (1U << 8);

    while ((RCC_CR & (1U << 10)) == 0)
    {
    }

    /* Enable GPIOB and USART1 clocks */
    RCC_AHB2ENR |= (1U << 1);
    RCC_APB2ENR |= (1U << 14);
    (void)RCC_AHB2ENR;
    (void)RCC_APB2ENR;

    /* Disable USART before configuration */
    USART1_CR1 = 0;

    /* USART1 clock source: HSI16, independent of CPU clock */
    RCC_CCIPR = (RCC_CCIPR & ~3U) | 2U;

    /* PB6: alternate-function mode */
    GPIOB_MODER &= ~(3U << 12);
    GPIOB_MODER |=  (2U << 12);

    /* PB6: AF7 = USART1_TX */
    GPIOB_AFRL &= ~(0xFU << 24);
    GPIOB_AFRL |=  (7U << 24);

    /* Push-pull, no pull-up/down, medium output speed */
    GPIOB_OTYPER &= ~(1U << 6);
    GPIOB_PUPDR &= ~(3U << 12);

    GPIOB_OSPEEDR &= ~(3U << 12);
    GPIOB_OSPEEDR |=  (1U << 12);

    /* 115200 baud, 8 data bits, no parity, 1 stop bit */
    USART1_CR2 = 0;
    USART1_CR3 = 0;
    USART1_BRR = (16000000U + 115200U / 2U) / 115200U;

    /* Enable USART and transmitter */
    USART1_CR1 = (1U << 0) | (1U << 3);

    /* Wait for transmitter ready */
    while ((USART1_ISR & (1U << 21)) == 0)
    {
    }
}

static void uart_write(const char *text)
{
    while (*text != '\0')
    {
        /* Wait until transmit data register is empty */
        while ((USART1_ISR & (1U << 7)) == 0)
        {
        }

        USART1_TDR = (uint8_t)*text++;
    }
}

static void potentiometer_init(void)
{
    /* Enable GPIOC and ADC clocks */
    RCC_AHB2ENR |= (1U << 2) | (1U << 13);
    (void)RCC_AHB2ENR;

    /* A0 / PC5: analog mode, no pull-up/down */
    GPIOC_MODER |= (3U << 10);
    GPIOC_PUPDR &= ~(3U << 10);

    /* Connect PC5 to ADC input */
    GPIOC_ASCR |= (1U << 5);

    /* ADC clock = HCLK / 4 */
    ADC_CCR = (ADC_CCR & ~(3U << 16)) | (3U << 16);

    /* Exit deep power-down and enable ADC regulator */
    ADC1_CR &= ~(1U << 29);
    ADC1_CR |=  (1U << 28);
    short_wait();

    /* Calibrate ADC for single-ended inputs */
    ADC1_CR &= ~(1U << 30);
    ADC1_CR |=  (1U << 31);

    while (ADC1_CR & (1U << 31))
    {
    }

    short_wait();

    /* 12-bit, single conversion, software trigger */
    ADC1_CFGR = 0;

    /* Channel 14 sampling time: 640.5 ADC clock cycles */
    ADC1_SMPR2 &= ~(7U << 12);
    ADC1_SMPR2 |=  (7U << 12);

    /* Clear ready flag and enable ADC */
    ADC1_ISR = (1U << 0);
    ADC1_CR |= (1U << 0);

    while ((ADC1_ISR & (1U << 0)) == 0)
    {
    }
}

static uint32_t adc_read(uint32_t channel)
{
    /* Wait for any previous conversion to finish */
    while (ADC1_CR & (1U << 2))
    {
    }

    /* One conversion, selected channel in first slot */
    ADC1_SQR1 = (channel << 6);

    /* Clear conversion flags */
    ADC1_ISR = (1U << 2) | (1U << 3) | (1U << 4);

    /* Start conversion */
    ADC1_CR |= (1U << 2);

    /* Wait for end of conversion */
    while ((ADC1_ISR & (1U << 2)) == 0)
    {
    }

    return ADC1_DR & 0xFFFU;
}

int main(void)
{
    char line[96];

    uart_init();
    potentiometer_init();

    uart_write("Potentiometer on A0 / PC5\r\n");

    while (1)
    {
        x_raw = adc_read(14);

        /* Position: -100% to +100% */
        x_percent = ((int32_t)x_raw * 200 / 4095) - 100;

        /* Estimated voltage with a 3.3 V ADC reference */
        x_millivolts = (x_raw * 3300U) / 4095U;

        snprintf(line, sizeof(line),
                 "ADC=%lu | Position=%ld%% | Voltage=%lu mV\r\n",
                 (unsigned long)x_raw,
                 (long)x_percent,
                 (unsigned long)x_millivolts);

        uart_write(line);

        /* Approximate pause; duration depends on CPU clock */
        for (uint32_t i = 0; i < 1U; ++i)
        {
            short_wait();
        }
    }
}
