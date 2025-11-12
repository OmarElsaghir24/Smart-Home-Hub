/*
 * user_created_tasks.c
 *
 *  Created on: Sep 17, 2025
 *      Author: Omar Elsaghir
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "adc0.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
#include "i2c0.h"
#include "i2c2.h"
#include "OLED.h"
#include "BME280.h"
#include "NRF24L01.h"
#include "kb.h"
#include "Servo.h"
#include "Display.h"
#include "user_created_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

// Global Variables
char str[100];
char buf[80];
int32_t temp, hum, press;
float pressure_Pa;
bool motion = false;
bool door_status = false;
const char correct_password[] = "8684";
uint8_t index = 0, incorrect_attempts = 0;
uint16_t ldr;
uint16_t light_time = 0;
char buffer[5];
nrf24lo1 pkt;

// Function for activating speaker
void ActivateSpeaker()
{
    int i = 0;
    for(i = 0; i < 10000; i++) {
      SPEAKER = 1;
      waitMicrosecond(185);
      SPEAKER = 0;
      waitMicrosecond(185);
      i++;
    }
}

// Task that receives messages from UART using queue
void vUartLoggerTask(void *pvParameters)
{
    char msg[LOG_MSG_SIZE];

    while(true)
    {
        if (xQueueReceive(xLogQueue, msg, portMAX_DELAY) == pdPASS)
        {
            putsUart0(msg);   // single writer to UART
        }
    }
}

// Task that executes automation (alarm trigger and motion detection) only when the event bits are set
void vAutomationTask(void *pvParameters)
{
    Sensor_Data data;
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (EVENT_MOTION_DETECTED | EVENT_PASSWORD_WRONG );

    while(true)
    {
        xEventGroupValue = xEventGroupWaitBits(securityEvent, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);

        if((xEventGroupValue & EVENT_PASSWORD_WRONG) != 0)
        {
             vTaskDelay(pdMS_TO_TICKS(500));
             setPinValue(RED_LED, 0);
             if(incorrect_attempts >= 3)
             {
                 setPinValue(RED_LED, 1);
                 ActivateSpeaker();
                 incorrect_attempts = 0;
                 setPinValue(RED_LED, 0);
             }
        }

        else if((xEventGroupValue & EVENT_MOTION_DETECTED) != 0)
        {
             if(xQueueReceive(sensor_data_queue, &data, 0))
             {
                 if(data.brightness < 30)
                 {
                     setPinValue(WHITE_LED, 1);
                 }
             }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task that executes door control using a servo motor only when the event bit is set
void vDoorTask(void *pvParameters)
{
    int i;
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (EVENT_DOOR_UNLOCKED | EVENT_PASSWORD_WRONG );
    while(true)
    {
        xEventGroupValue = xEventGroupWaitBits(securityEvent, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);

        if((xEventGroupValue & EVENT_DOOR_UNLOCKED) != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            setPinValue(GREEN_LED, 0);
            incorrect_attempts = 0;
            for(i = 0; i <= 180; i++)
            {
                setServoAngle(i);
                waitMicrosecond(15000);
            }

            waitMicrosecond(3000000);

            for(i = 180; i >= 0; i--)
            {
                setServoAngle(i);
                waitMicrosecond(15000);
            }
            door_status = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Auto Reload timer that clears the interrupt every 0.5 seconds to check if any motion is detected
void prvPirSensorTimerCallback(TimerHandle_t xTimer)
{
    if(PIR)
    {
        BLUE_LED = PIR;
        motion = true;
        xEventGroupSetBits(securityEvent, EVENT_MOTION_DETECTED);
        light_time = 0;
    }
    else
    {
        motion = false;
        BLUE_LED = 0;
        light_time++;
        snprintf(buf, sizeof(buf), "Count = %d\n", light_time);
        putsUart0(buf);
        if(light_time >= 180)
        {
            setPinValue(WHITE_LED, 0);
            light_time = 0;
        }
    }
}

// Task for checking if password stored in buffer is equal to correct password and sets the appropriate event bit based on buffer comparison.
void vPasswordTask(void *pvParameters)
{
    char c;

    while(true)
    {
        if(xQueueReceive(passwordQueue, &c, portMAX_DELAY) == pdPASS)
        {
            if(index < 4)
            {
               buffer[index] = c;
               index++;
            }

            if(index == 4)
            {
               buffer[4] = '\0';
               putsUart0("Stored Password: ");
               putsUart0(buffer);
               putsUart0("\n");
               if(strcmp(buffer, correct_password) == 0)
               {
                   setPinValue(GREEN_LED, 1);
                   ssd1306_printAtSize(7, 8, "Access Granted!", 1);
                   door_status = true;
                   xEventGroupSetBits(securityEvent, EVENT_DOOR_UNLOCKED);
               }
               else
               {
                   setPinValue(RED_LED, 1);
                   ssd1306_printAtSize(7, 8, "Access Denied!", 1);
                   incorrect_attempts++;
                   door_status = false;
                   xEventGroupSetBits(securityEvent, EVENT_PASSWORD_WRONG);
               }
               index = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Task for reading which buttons are pressed on keypad
void vKeypadTask(void *pvParameters)
{
    char c;

    while(true)
    {
        if (kbhit())
        {
            c = getKey();
            if (c != 'D')
            {
                putcUart0(c);
                xQueueSend(passwordQueue, &c, portMAX_DELAY);
            }
            else
            {
                putsUart0("\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Task for transmitting sensor data, motion and door status to ESP32 to publish to MQTT broker
void vWirelessTask(void *pvParameters)
{
    Sensor_Data data;
    while (true)
    {
        if(xQueueReceive(sensor_data_queue, &data, portMAX_DELAY))
        {
           pkt.frame_id = 0;
           pkt.type = 0x0C;
           pkt.dataLength = 6;
           pkt.data[0] = data.temperature;
           pkt.data[1] = data.humidity;
           pkt.data[2] = data.pressure;
           pkt.data[3] = motion;
           pkt.data[4] = data.brightness;
           pkt.data[5] = door_status;

           sendPacket(&pkt);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// Task for displaying sensor measurements on OLED display
void vOLEDTask(void * pvParameters)
{
    Sensor_Data display_data;
    vTaskDelay(pdMS_TO_TICKS(10));
    while(true)
    {
        if(xQueueReceive(sensor_data_queue, &display_data, portMAX_DELAY))
        {
           display_sequence(display_data);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1s
    }
}

// Task for gathering measurements from sensors and send them to other tasks using queue
void vEnvironmentTask(void *pvParameters)
{
    Sensor_Data data;

    readBME280Calibration();
    while(true)
    {
       readBME280Raw(&temp, &press, &hum);

       data.temperature = compensateTemperature(temp);
       pressure_Pa = compensatePressure(press);
       data.humidity = compensateHumidity(hum);
       ldr = readAdc0Ss3();

       data.pressure = pressure_Pa / 100.0f;

       data.brightness = ((4095 - ldr) * 100) / 4095;

       xQueueSend(sensor_data_queue, &data, portMAX_DELAY);

       vTaskDelay(pdMS_TO_TICKS(1000)); // read every 1s
    }
}

// Shell task for monitoring system status
void vShellTask(void *pvParameters)
{
    USER_DATA data;
    int count = 0;
    uint8_t i;

    memset(&data, 0, sizeof(data));
    putsUart0("\r\n> ");   // show initial prompt

    while (true)
    {
        if (kbhitUart0())
        {
            char c = getcUart0();

            if ((c == 8 || c == 127) && count > 0)   // backspace
            {
                count--;
            }
            else if (c == 13) // Enter
            {
                data.buffer[count] = 0;   // null terminate
                parseFields(&data);

                for (i = 0; i < data.fieldCount; i++)
                {
                    putcUart0(data.fieldType[i]);
                    putcUart0('\t');
                    putsUart0(&data.buffer[data.fieldPosition[i]]);
                    putcUart0('\n');
                }

                if (isCommand(&data, "Heap", 0))
                {
                    snprintf(buf, sizeof(buf),
                                    "Free Heap: %u bytes\n"
                                    "Min Ever Free Heap: %u bytes\n"
                                    "Shell Stack Remaining: %u bytes\n\n",
                                    (unsigned) xPortGetFreeHeapSize(),
                                    (unsigned) xPortGetMinimumEverFreeHeapSize(),
                                    (unsigned) (uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t))
                           );
                           xSemaphoreTake(xUartMutex, portMAX_DELAY);
                           putsUart0(buf);
                           putsUart0("\n\n");
                           xSemaphoreGive(xUartMutex);
                }
                else if(isCommand(&data, "Reset", 0))
                {
                    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                }
                else if(isCommand(&data, "Tasks", 0))
                {
                    char buf[LOG_MSG_SIZE];
                    int i;

                    // Section header
                    snprintf(buf, sizeof(buf), "Memory Information\n---------------------\n");
                    xQueueSend(xLogQueue, buf, portMAX_DELAY);

                    // Heap info
                    snprintf(buf, sizeof(buf),
                             "Free Heap: %u bytes\n"
                             "Min Ever Free Heap: %u bytes\n"
                             "Shell Stack Remaining: %u bytes\n\n",
                             (unsigned) xPortGetFreeHeapSize(),
                             (unsigned) xPortGetMinimumEverFreeHeapSize(),
                             (unsigned) (uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
                    xQueueSend(xLogQueue, buf, portMAX_DELAY);

                    // Runtime stats
                    char runtimeBuf[512];
                    vTaskGetRunTimeStats(runtimeBuf);

                    snprintf(buf, sizeof(buf), "Task CPU Usage (%%):\n");
                    xQueueSend(xLogQueue, buf, portMAX_DELAY);

                    // Break runtimeBuf into chunks of 100 chars to avoid queue overflow
                    for (i = 0; i < strlen(runtimeBuf); i += 100)
                    {
                         strncpy(buf, &runtimeBuf[i], 100);
                         buf[100] = '\0';
                         xQueueSend(xLogQueue, buf, portMAX_DELAY);
                    }

                    // Number of tasks
                    snprintf(buf, sizeof(buf), "Number of tasks: %u\n\n", (unsigned) uxTaskGetNumberOfTasks());
                    xQueueSend(xLogQueue, buf, portMAX_DELAY);
                }
                else
                {
                    xSemaphoreTake(xUartMutex, portMAX_DELAY);
                    putsUart0("Invalid Command\r\n");
                    xSemaphoreGive(xUartMutex);
                }

                count = 0;   // reset buffer
                memset(data.buffer, 0, sizeof(data.buffer));
                putsUart0("> "); // show prompt again
            }
            else if (c >= 32 && count < MAX_CHARS-1)
            {
                data.buffer[count++] = c;
            }
        }

        vTaskDelay(1); // yield to other tasks
    }
}
