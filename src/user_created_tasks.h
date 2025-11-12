/*
 * user_created_tasks.h
 *
 *  Created on: Sep 17, 2025
 *      Author: Omar Elsaghir
 */

#ifndef USER_CREATED_TASKS_H_
#define USER_CREATED_TASKS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

// Defines
#define PIR                   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define SPEAKER               (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define BLUE_LED              (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define PIR_MASK       32
#define SPEAKER_MASK   64
#define BLUE_LED_MASK  4
#define RED_LED PORTF, 1
#define GREEN_LED PORTF, 3
#define WHITE_LED PORTC, 4

#define EVENT_MOTION_DETECTED ( 1UL << 0UL )
#define EVENT_DOOR_UNLOCKED   ( 1UL << 1UL )
#define EVENT_PASSWORD_WRONG  ( 1UL << 2UL )

#define LOG_QUEUE_LEN   16   // number of messages in queue
#define LOG_MSG_SIZE    128  // max length of each message

#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )

// Variables
QueueHandle_t sensor_data_queue;
QueueHandle_t passwordQueue;
QueueHandle_t xLogQueue;
SemaphoreHandle_t xUartMutex;
EventGroupHandle_t securityEvent;

// Structures
typedef struct _Sensor_Data
{
    float temperature;
    float humidity;
    float pressure;
    uint8_t brightness;
}Sensor_Data;

// Subroutines
void vEnvironmentTask(void *pvParameters);
void vOLEDTask(void * pvParameters);
void vControllerTask(void *pvParameters);
void vShellTask(void *pvParameters);
void vKeypadTask(void *pvParameters);
void vPasswordTask(void *pvParameters);
void vUartLoggerTask(void *pvParameters);
void vDoorTask(void *pvParameters);
void vAutomationTask(void *pvParameters);
void prvPirSensorTimerCallback(TimerHandle_t xTimer);
void vWirelessTask(void *pvParameters);

#endif /* USER_CREATED_TASKS_H_ */
