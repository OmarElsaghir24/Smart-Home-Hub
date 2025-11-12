

/**
 * main.c
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "adc0.h"
#include "uart0.h"
#include "wait.h"
#include "i2c0.h"
#include "i2c2.h"
#include "spi0.h"
#include "OLED.h"
#include "LDR.h"
#include "BME280.h"
#include "NRF24L01.h"
#include "kb.h"
#include "Servo.h"
#include "user_created_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/*
#define HWREG(x) (*((volatile unsigned long *)(x)))
#define NVIC_CPAC_R   0xE000ED88  // Coprocessor Access Control Register
#define FPU_FPCCR     0xE000EF34  // Floating-Point Context Control Register

void enableFPU(void)
{
    // Enable full access to CP10 and CP11
    HWREG(NVIC_CPAC_R) = (HWREG(NVIC_CPAC_R) & ~(0xF << 20)) | (0xF << 20);

    // Enable lazy stacking (ASPEN and LSPEN bits in FPCCR)
    HWREG(FPU_FPCCR) |= (1 << 31) | (1 << 30);
}*/

// Enable FPU macro
#define HWREG(x) (*((volatile unsigned long *)(x)))
#define NVIC_CPAC_R 0xE000ED88

void enableFPU(void)
{
    // Enable FPU
    HWREG(NVIC_CPAC_R) = ((HWREG(NVIC_CPAC_R) & ~(0xF << 20)) | (0xF << 20));
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    // Blink LED here to signal overflow
    while(true)
    {
        setPinValue(RED_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        setPinValue(RED_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Timer0_Init(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;        // Enable Timer1 clock
    while ((SYSCTL_PRTIMER_R & 0x01) == 0) {} // wait until ready

    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;         // disable timer
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;   // 32-bit timer
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;  // periodic timer mode
    TIMER0_TAILR_R = 0xFFFFFFFF;             // max value
    TIMER0_TAPR_R = 0;                       // no prescaler
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;       // clear timeout flag
    TIMER0_CTL_R |= TIMER_CTL_TAEN;          // enable timer
}

void initHw(void)
{
    // Enable clocks
    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    _delay_cycles(3);

    // Configure PIR as input
    GPIO_PORTB_DIR_R &= ~PIR_MASK;
    GPIO_PORTB_DEN_R |= PIR_MASK;

    // Configure Blue Led as output
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;

    // Configure Speaker as output
    GPIO_PORTC_DIR_R |= SPEAKER_MASK;
    GPIO_PORTC_DEN_R |= SPEAKER_MASK;

    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(WHITE_LED);
    selectPinPushPullOutput(CS);
    selectPinPushPullOutput(CE);
    //selectPinPushPullOutput(IRQ);

    /*GPIO_PORTB_DIR_R &= ~IRQ_MASK;

    GPIO_PORTB_DEN_R |= IRQ_MASK;

    GPIO_PORTB_IS_R &= ~IRQ_MASK;     // Edge-sensitive
    GPIO_PORTB_IBE_R &= ~IRQ_MASK;    // Not both edges
    GPIO_PORTB_IEV_R &= ~IRQ_MASK;    // Falling edge

    GPIO_PORTB_ICR_R = IRQ_MASK;      // Clear flag
    GPIO_PORTB_IM_R |= IRQ_MASK;      // Unmask PB2

    NVIC_EN0_R |= (1 << (INT_GPIOB - 16));*/
}

int main(void)
{
    // Enable FPU
    enableFPU();

    // Setup hardware
    initHw();

    // Init BME280 sensor
    initBME280();

    // Init LDR
    initLDR();

    // Init keypad
    initKb();

    // Init servo motor
    initServo();

    // Initialize SPI0
    initSpi0(USE_SSI0_RX);
    setSpi0BaudRate(10e6, 40e6);
    setSpi0Mode(0, 0);

    // Init NRF24L01+ module
    nrf24_init();

    // Setup UART0
    UART0_Init(115200, 16000000);

    // Setup I2C2
    initI2c2();

    // Initialize OLED display
    init_ssd1306();
    ssd1306_clearDisplay();

    TimerHandle_t xAutoReloadTimer;
    BaseType_t xTimer1Started;

    // Create queues
    sensor_data_queue = xQueueCreate(5, sizeof(Sensor_Data));
    passwordQueue = xQueueCreate(1, sizeof(char));
    xLogQueue = xQueueCreate(LOG_QUEUE_LEN, LOG_MSG_SIZE);

    // Create event_group
    securityEvent = xEventGroupCreate();

    // Create mutex
    xUartMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreate(vUartLoggerTask, "UART Logger", 512, NULL, 1, NULL);
    xTaskCreate(vOLEDTask, "OLED", 1024, NULL, 1, NULL);
    xTaskCreate(vEnvironmentTask, "Environment Monitor", 512, NULL, 3, NULL);
    xTaskCreate(vKeypadTask, "Keypad", 256, NULL, 4, NULL);
    xTaskCreate(vPasswordTask, "Password Check", 256, NULL, 4, NULL);
    xTaskCreate(vDoorTask, "Door Unlock", 512, NULL, 4, NULL);
    xTaskCreate(vWirelessTask, "Wireless", 256, NULL, 5, NULL);
    xTaskCreate(vAutomationTask, "Automation Task", 256, NULL, 4, NULL);
    xTaskCreate(vShellTask, "Shell", 512, NULL, 5, NULL);

    // Create timer
    xAutoReloadTimer = xTimerCreate("Motion Detection", mainAUTO_RELOAD_TIMER_PERIOD, pdTRUE, 0, prvPirSensorTimerCallback);

    if( ( xAutoReloadTimer != NULL ) )
    {
           xTimer1Started = xTimerStart( xAutoReloadTimer, 0 );
           if( ( xTimer1Started == pdPASS ) )
           {

                 vTaskStartScheduler();
           }
    }

    while(true);
}

