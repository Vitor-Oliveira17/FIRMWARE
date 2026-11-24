/**
 * @file motor_test.c
 * @brief Implementação do teste em malha aberta do motor.
 */

#include "motor_test.h"

#include <stddef.h>

#include "encoder_magnetic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

static const char *TAG = "MOTOR_TEST";

#define MOTOR_TEST_STEP_MS       3000
#define MOTOR_TEST_STOP_MS       1000
#define MOTOR_TEST_SAMPLE_MS      100

typedef struct {
    int16_t duty;
    const char *label;
} motor_test_step_t;

static const motor_test_step_t test_steps[] = {
    {  64, "+25%"},
    { 128, "+50%"},
    { 191, "+75%"},
    { 255, "+100%"},
    { -64, "-25%"},
    {-128, "-50%"},
    {-191, "-75%"},
    {-255, "-100%"},
};

typedef struct {
    i2c_master_bus_handle_t bus_handle;
} motor_test_context_t;

static motor_test_context_t test_context;

static void motor_test_task(void *parameter)
{
    const motor_test_context_t *context = parameter;

    ESP_LOGW(TAG, "Teste iniciado: mantenha a roda suspensa e afastada de pessoas.");

    for (size_t step = 0; step < sizeof(test_steps) / sizeof(test_steps[0]); step++) {
        const motor_test_step_t *current = &test_steps[step];
        motor_set_speed(current->duty);
        ESP_LOGI(TAG, "PWM %s (duty=%d) por %d ms", current->label,
                 current->duty, MOTOR_TEST_STEP_MS);

        for (int elapsed_ms = 0; elapsed_ms < MOTOR_TEST_STEP_MS;
             elapsed_ms += MOTOR_TEST_SAMPLE_MS) {
            vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_SAMPLE_MS));
            const float rpm = encoder_read_rpm(context->bus_handle);
            const float angle_deg = encoder_read_angle(context->bus_handle);
            ESP_LOGI(TAG, "PWM %s | encoder: %.2f deg | %.2f RPM",
                     current->label, angle_deg, rpm);
        }

        motor_set_speed(0);
        ESP_LOGI(TAG, "Motor parado por %d ms", MOTOR_TEST_STOP_MS);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_STOP_MS));
    }

    motor_set_speed(0);
    ESP_LOGW(TAG, "Teste concluido; motor desligado.");
    vTaskDelete(NULL);
}

esp_err_t motor_test_start(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    test_context.bus_handle = bus_handle;
    const BaseType_t task_created = xTaskCreate(
        motor_test_task, "motor_test", 4096, &test_context,
        configMAX_PRIORITIES - 3, NULL);
    return task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
