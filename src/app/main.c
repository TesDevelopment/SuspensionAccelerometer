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

#define G_CONVERSION_FACTOR 0.004
#define ACCEL_SPI           SPI2
#define NUM_SAMPLES         10

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

    core_SPI_start(ACCEL_SPI);
    core_SPI_read_write(ACCEL_SPI, request, sizeof(request), response, sizeof(response));
    core_SPI_stop(ACCEL_SPI);

    uint8_t data = response[1];
    return data;
}

void write_register(uint8_t address, uint8_t value) {
    uint8_t read_request[2] = {
        address,
        value,
    };

    uint8_t response_buffer[2] = {
        0x0,
        0x0
    };

    core_SPI_start(ACCEL_SPI);
    core_SPI_read_write(ACCEL_SPI, read_request, sizeof(read_request), response_buffer, sizeof(response_buffer));
    core_SPI_stop(ACCEL_SPI);
    
    uint8_t resp = read_register(address);
    if (resp != value) rprintf("ERROR: wrote %02x, received %02x\n", value, resp);
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
    write_register(0x2D, 0x08);


    /*
        Set the refresh rate to 1000hz
        [W, 0, 0x2C]
        1 0 00101101
        1000101101
        0x22D

        1600 hz -> 0x0E
    */

    rprintf("Setting refresh rate to 1600hz...\n");
    write_register(0x2C, 0x0E);

    rprintf("Setting range...\n");
    write_register(0x31, 0x0B);
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

int debugedCanMessage(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data) {
    
    int msg = core_CAN_send_message(can, id, dlc, data);

    if(msg != 1) {
        rprintf("Sending CAN message failed...");
    }
    while ((can->PSR & 0x18) == 0x18);
    for (int i=0; i < 1000; i++);
}

void read_accel_task(void *pvParameters) {
    (void) pvParameters;

    rprintf("Initializing Accelerometer...\n"); // VERY IMPORTANT!!!!!!!!!! - breaks code if you remove this
    init_board();

    rprintf("Polling offsets...\n");
    //float* offsets = poll_offsets();
    
    vTaskDelay(20);
    //int xOffset = (((int)(int8_t)read_register(0x33) << 8) | read_register(0x32));  
    //int yOffset = (((int)(int8_t)read_register(0x35) << 8) | read_register(0x34));
    //int zOffset = (((int)(int8_t)read_register(0x37) << 8) | read_register(0x36));
    int xOffset = 0, yOffset = 0, zOffset = 0;
    rprintf("offsets %d, %d, %d\n", xOffset, yOffset, zOffset);

    float mins[3];
    float maxs[3];
    float temp[3];

    while(true) {
        temp[0] = ((((int)(int8_t)read_register(0x33) << 8) | read_register(0x32)) - xOffset) * G_CONVERSION_FACTOR;  
        temp[1] = ((((int)(int8_t)read_register(0x35) << 8) | read_register(0x34)) - yOffset) * G_CONVERSION_FACTOR;
        temp[2] = ((((int)(int8_t)read_register(0x37) << 8) | read_register(0x36)) - zOffset) * G_CONVERSION_FACTOR;

        // rprintf("G-Forces (simplified): [%d, %d, %d]\n", (int) xG, (int) yG, (int) zG);
        // rprintf("G-Forces * 1000: [%d, %d, %d]\n", xDisplay, yDisplay, zDisplay);
        // rprintf("G-Forces (debug): : [%d (%d), %d (%d), %d (%d)]\n",xDisplay, (int) (x * G_CONVERSION_FACTOR * 1000), yDisplay, (int) (y * G_CONVERSION_FACTOR * 1000), zDisplay, (int) (z * G_CONVERSION_FACTOR * 1000));
        // rprintf("----------------------------\n");

        float sampled_x = temp[0];
        float sampled_y = temp[1];
        float sampled_z = temp[2];

        mins[0] = temp[0];
        mins[1] = temp[1];
        mins[2] = temp[2];

        maxs[0] = temp[0];
        maxs[1] = temp[1];
        maxs[2] = temp[2];

        vTaskDelay(1);

        for(int i = 1; i < NUM_SAMPLES;i++) {
            float temp_x =  ((((int)(int8_t)read_register(0x33) << 8) | read_register(0x32)) - xOffset) * G_CONVERSION_FACTOR;
            float temp_y = ((((int)(int8_t)read_register(0x35) << 8) | read_register(0x34)) - yOffset) * G_CONVERSION_FACTOR;
            float temp_z = ((((int)(int8_t)read_register(0x37) << 8) | read_register(0x36)) - zOffset) * G_CONVERSION_FACTOR;

            if (temp_x > maxs[0]){
                maxs[0] = temp_x;
            } else if(temp_x < mins[0]){
                mins[0] = temp_x;
            }

            if (temp_y > maxs[1]){
                maxs[1] = temp_y;
            } else if(temp_y < mins[0]){
                mins[1] = temp_y;
            }

            if (temp_z > maxs[2]){
                maxs[2] = temp_z;
            } else if(temp_z < mins[2]){
                mins[2] = temp_z;
            }
            
            sampled_x += temp_x;
            sampled_y += temp_y;
            sampled_z +=  temp_z;

            vTaskDelay(portTICK_PERIOD_MS);
        }

        sampled_x /= NUM_SAMPLES;
        sampled_y /= NUM_SAMPLES;
        sampled_z /= NUM_SAMPLES;

        int16_t averages[3];
        averages[0] = (int)(sampled_x * 1000);
        averages[1] = (int)(sampled_y * 1000);
        averages[2] = (int)(sampled_z * 1000);

        int16_t min_out[3];
        min_out[0] = (int) (mins[0] * 1000);
        min_out[1] = (int) (mins[1] * 1000);
        min_out[2] = (int) (mins[2] * 1000);

        int16_t max_out[3];
        max_out[0] = (int) (maxs[0] * 1000);
        max_out[1] = (int) (maxs[1] * 1000);
        max_out[2] = (int) (maxs[2] * 1000);

        rprintf("%d %d %d\n", averages[0], averages[1], averages[2]);

        debugedCanMessage(FDCAN1, 506, 6, *((uint64_t*)averages));
        debugedCanMessage(FDCAN1, 507, 6, *((uint64_t*)min_out));
        debugedCanMessage(FDCAN1, 508, 6, *((uint64_t*)max_out));

        //vTaskDelay(100);
    }
}


int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOA, GPIO_PIN_15);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();

    core_GPIO_init(GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_digital_write(GPIOA, GPIO_PIN_4, true);
    core_SPI_init(ACCEL_SPI, GPIOB, GPIO_PIN_12);
    ACCEL_SPI->I2SCFGR = 0;
    ACCEL_SPI->CR1 = (7 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_CPHA | SPI_CR1_CPOL;
    ACCEL_SPI->CR2 = SPI_CR2_SSOE | (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    ACCEL_SPI->CR1 |= SPI_CR1_SPE;
    core_RTT_init(); 

    //Init pins
    core_GPIO_init(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Left Light
    core_GPIO_init(GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL); // Right Light

    if (!core_CAN_init(FDCAN1, 1000000)) error_handler();
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
