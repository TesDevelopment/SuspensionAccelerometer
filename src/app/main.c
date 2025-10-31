#include "main.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

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

#define G_CONVERSION_FACTOR 0.0039

void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    while(true) {
	    //rprintf("Hearbeat!\n");
        core_GPIO_toggle_heartbeat();
        vTaskDelay(1000 * portTICK_PERIOD_MS);
    }
}

uint8_t read_register(uint8_t address) {
    uint8_t request[2] = {
        address + 0b10000000,
        0x0
    };

    uint8_t response[2] = {
        0x0,
        0x0
    };

    core_SPI_start(SPI1);
    core_SPI_read_write(SPI1, request, sizeof(request), response, sizeof(response));
    core_SPI_stop(SPI1);

    uint8_t data = response[1];
    return data;
}

void init_board() {
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
        ] -> 0x08
    */ 

    rprintf("Entering measurement mode\n");
    uint8_t read_request[2] = {
        0x2D,
        0x08,
    };

    uint8_t response_buffer[2] = {
        0x0,
        0x0
    };

    core_SPI_start(SPI1);
    core_SPI_read_write(SPI1, read_request, sizeof(read_request), response_buffer, sizeof(response_buffer));
    core_SPI_stop(SPI1);
}

uint8_t build_dbc_message(uint8_t z){
    return 0;
}

/*

    Unused for now,
    Seems to overcomplicate the process when one poll is enough;
    multiple polls doesn't seem to effect the overall result

*/
float* poll_offsets() {
    rprintf("Entering polling cycle...\n");
    int polling_rate = 500;

    float xSum = 0, ySum = 0, zSum = 0;
    for (int i = 0; i < polling_rate; i++) {
        xSum += ((int)(int8_t)read_register(0x33) << 8) | read_register(0x32);
        ySum +=  ((int)(int8_t)read_register(0x35) << 8) | read_register(0x34);
        zSum += ((int)(int8_t)read_register(0x37) << 8) | read_register(0x36);
    }

    float xAvg = xSum / polling_rate;
    float yAvg = ySum / polling_rate;
    float zAvg = zSum / polling_rate;

    // Converting to int is intentional, float displays dont work with rtt
    rprintf("Finished polling: (X) = %d (Y) = %d (Z) = %d\n", (int) ((xAvg * G_CONVERSION_FACTOR) * 1000), (int) ((yAvg * G_CONVERSION_FACTOR) * 1000), (int) ((zAvg * G_CONVERSION_FACTOR) * 1000));

    float offsets[3] = {
        xAvg,
        yAvg,
        zAvg
    };

    return offsets;
}

void read_accel_task(void *pvParameters) {
    (void) pvParameters;

    rprintf("Initializing Accelerometer...\n"); // VERY IMPORTANT!!!!!!!!!! - breaks code if you remove this
    init_board();

    rprintf("Polling offsets...\n");
    //float* offsets = poll_offsets();

    int xOffset = (((int)(int8_t)read_register(0x33) << 8) | read_register(0x32));  
    int yOffset = (((int)(int8_t)read_register(0x35) << 8) | read_register(0x34));
    int zOffset = (((int)(int8_t)read_register(0x37) << 8) | read_register(0x36));
    
    float mag;

    while(true) {
        int x = (((int)(int8_t)read_register(0x33) << 8) | read_register(0x32));  
        int y = (((int)(int8_t)read_register(0x35) << 8) | read_register(0x34));
        int z = (((int)(int8_t)read_register(0x37) << 8) | read_register(0x36));

        rprintf("Raw Accel: %d %d %d\n", x, y, z);
        mag = sqrtf(x*x + y*y + z*z);
        rprintf("Magnitude: %d\n", (int)(mag));

        float xG = (x - xOffset) * G_CONVERSION_FACTOR;
        float yG = (y - yOffset) * G_CONVERSION_FACTOR;
        float zG = (z - zOffset) * G_CONVERSION_FACTOR;

        int xDisplay = (int) (xG * 1000);
        int yDisplay = (int) (yG * 1000);
        int zDisplay = (int) (zG * 1000);
        

        rprintf("G-Forces (simplified): [%d, %d, %d]\n", (int) xG, (int) yG, (int) zG);
        rprintf("G-Forces * 1000: [%d, %d, %d]\n", xDisplay, yDisplay, zDisplay);
        rprintf("G-Forces (debug): : [%d (%d), %d (%d), %d (%d)]\n",xDisplay, (int) (x * G_CONVERSION_FACTOR * 1000), yDisplay, (int) (y * G_CONVERSION_FACTOR * 1000), zDisplay, (int) (z * G_CONVERSION_FACTOR * 1000));
        rprintf("----------------------------\n");

        uint16_t g_forces[3];
        g_forces[0] = xDisplay;
        g_forces[1] = yDisplay;
        g_forces[2] = zDisplay;
        
        core_CAN_send_message(FDCAN1, 4, 6, *((uint64_t*)g_forces));
        vTaskDelay(100);
    }
}


/*
    PA0 (Solid Light) -> Measuring
    PA0 (OFF) -> Calirating
*/
int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_15);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();

    core_GPIO_init(GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_digital_write(GPIOA, GPIO_PIN_4, true);
    core_SPI_init(SPI1, GPIOA, GPIO_PIN_4);
    // core_GPIO_init(GPIOA, GPIO_PIN_7|GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    // core_GPIO_digital_write(GPIOA, GPIO_PIN_7|GPIO_PIN_5, 0);
    // while (1);
    core_RTT_init(); 

    //Init pins
    core_GPIO_init(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Left Light
    core_GPIO_init(GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Right Light

    if (!core_CAN_init(FDCAN1, 1000000)) error_handler();
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
