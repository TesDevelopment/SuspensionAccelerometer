#include "main.h"

#include <stdbool.h>
#include <stdio.h>

#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "error_handler.h"
#include "core_config.h"
#include "spi.h"
#include "adc.h"
#include "rtt.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>

#define G_CONVERSION_FACTOR 0.0004

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    while(true) {
        core_GPIO_toggle_heartbeat();
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

uint8_t init_board() {
    /*
        Set the Measure Mode to Acceleration

        [W, 0, 0x2D]
        0 0 101101
        0x18AED
        0x2D

        [
            0, Link Bit
            0, Auto-SLeep,
            1, Measurement Mode
            0, Sleep
            00, Wakeup bits
        ]
    */

        /*
        Read the offset of the z-axis

        [R, 0, 0x20]
        1 0 100000
        0xA0
    */

    uint8_t read_request[2] = {
        0xF6,
        0xA0,
    };

    uint8_t *request = read_request;

    core_SPI_start(SPI1);
    core_SPI_read_write(SPI1, request, sizeof(request), request, sizeof(request));
    core_SPI_stop(SPI1);

    return read_request[1];
}

uint8_t build_dbc_messae(uint8_t z){
    return 0;
}

void read_accel_task(void *pvParameters) {
    (uint8_t) pvParameters;
    uint8_t zOffset = init_board();

    /*
        [R, 1, 0x36]
        11110110
        246
        0xF6
    */
    uint8_t read_request[3] = {
        0xF6,
        0x0,
        0x0
    };

    uint8_t *request = read_request;


    while(true) {
        core_SPI_start(SPI1);
        core_SPI_read_write(SPI1, request, sizeof(request), request, sizeof(request));
        core_SPI_stop(SPI1);

        uint16_t z = (read_request[1] << 8) | read_request[2];

        uint8_t zG = (z * G_CONVERSION_FACTOR) - zOffset;

        rprintf("%f", zG);
        
        core_CAN_send_fd_message(FDCAN1, 0xFFBF61E, sizeof(zG), &zG);
    }
}


/*
    PA0 (Solid Light) -> Measuring
    PA0 (OFF) -> Calirating
*/
int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_5);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    core_SPI_init(SPI1, GPIOA, GPIO_PIN_4);
    core_RTT_init();

    //Init pins
    core_GPIO_init(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Left Light
    core_GPIO_init(GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Right Light

    if (!core_clock_init()) error_handler();
    if (!core_CAN_init(CORE_BOOT_FDCAN, 1000000)) error_handler();
    //core_boot_init();

    int err = xTaskCreate(heartbeat_task, "heartbeat", 1000, NULL, 4, NULL);
    if (err != pdPASS) {
        error_handler();
    }

    
    err = xTaskCreate(read_accel_task, "acceleration", 1000, NULL, 4, NULL);
    if(err != pdPASS) {
        error_handler();
    }

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    error_handler();
    return 1;
}

// Called when stack overflows from rtos
// Not needed in header, since included in FreeRTOS-Kernel/include/task.h
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName) {
    (void) xTask;
    (void) pcTaskName;

    error_handler();
}
