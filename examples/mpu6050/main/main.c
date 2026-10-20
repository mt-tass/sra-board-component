/*
 * MIT License
 *
 * Copyright (c)  2021 Society of Robotics and Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sra_board.h"

static const char *TAG = "mpu_example";
#define G_RAW_VALUE 16384.0f

void app_main()
{    
    // Enable the MPU-6050 Sensor
    enable_mpu6050();
    // Setting up the led matrix
    led_matrix_t matrix;
    ESP_ERROR_CHECK(led_matrix_init(&matrix));
    ESP_ERROR_CHECK(led_matrix_set_brightness(&matrix, 4));
    float dir_x, dir_y, normalizing_factor, normalized_dir_x, normalized_dir_y;
    uint8_t rows[LED_MATRIX_HEIGHT] = {0x00};
    uint8_t prev_rows[LED_MATRIX_HEIGHT] = {0x00};

    // Setting up the result arrays, time difference and initial conditions
    float acce_angle[2], gyro_angle[2], complementary_angle[2], dt;
    float mpu_offset[2] = {0.0f, 0.0f};

    int16_t acce_raw_value[BUFF_SIZE / 2], gyro_raw_value[BUFF_SIZE / 2];
    uint32_t timer = esp_timer_get_time();

    // Read the MPU raw values
    while (read_mpu6050_raw(acce_raw_value, gyro_raw_value) == ESP_OK)
    {
        // Calculate the time difference between two samples
        dt = (float)(esp_timer_get_time() - timer) / 1000000;
        timer = esp_timer_get_time();
        
        // Get the angles as per the accelerometer raw values only
        compute_acce_angle(acce_raw_value[0], acce_raw_value[1], acce_raw_value[2], acce_angle);
        // Get the angles computed as per the gyroscope raw values only
        compute_gyro_angle(gyro_raw_value[0], gyro_raw_value[1], gyro_raw_value[2], dt, gyro_angle);
        // Get fused angles due to both the accelerometer and gyroscope
        complementary_filter(acce_raw_value, gyro_raw_value, complementary_angle, mpu_offset);

        // Plotting the different output plots
        printf("%f \t  %f \t  %f \n ", acce_angle[1], gyro_angle[1], complementary_angle[1]);

        // get g value from raw value
        dir_x = ((float)acce_raw_value[0])/G_RAW_VALUE;
        dir_y = ((float)acce_raw_value[1])/G_RAW_VALUE;

        // normalizes the value to get the direction vector
        normalizing_factor = sqrt(dir_x*dir_x + dir_y*dir_y);
        normalized_dir_x = dir_x/normalizing_factor;
        normalized_dir_y = dir_y/normalizing_factor;
        
        // loops through each pixel and calculates the dot product of the position vector of pixel from origin and the direction vector
        for(int i = 0 ;  i < LED_MATRIX_HEIGHT; i++){
            rows[i] = 0x00;
            for(int j = 0; j < LED_MATRIX_WIDTH; j++){
                float pos_x = 3.5f - i;
                float pos_y = j - 3.5f;
                float score = pos_x*normalized_dir_x + pos_y*normalized_dir_y;
                // limits the dot product to not change the state of led on very small change
                float threshold;
                if(prev_rows[i] & (1 << j)){
                    threshold = -0.30f;
                }
                else{
                    threshold = 0.30f;
                }
                if(score > threshold){
                    rows[i] |= 1 << j;
                }
            }
            // stores the previous state
            prev_rows[i] = rows[i];
        }
        ESP_ERROR_CHECK(led_matrix_draw_bitmap(&matrix,rows));
        ESP_ERROR_CHECK(led_matrix_show(&matrix));
        vTaskDelay(pdMS_TO_TICKS(20));
        
    }

    ESP_LOGE(TAG, "%s", "Error reading values!");
}
