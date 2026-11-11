/**
 * @file main.c
 * @brief Firmware SEC: Controle em Espaço de Estados Discreto (Pêndulo com Roda de Inércia)
 *
 * Módulos integrados:
 *   - IMU MPU9250 (Giroscópio Z -> theta_dot, theta)
 *   - Encoder AS5600 (RPM -> omega_r)
 *   - Ponte H BTS7960 (PWM LEDC 8 bits -> -255 a +255)
 *   - Controlador em Espaço de Estados (state_controller.h)
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "imu.h"
#include "motor.h"
#include "encoder_magnetic.h"
#include "state_controller.h"

static const char *TAG = "MAIN_SEC";

#define DEG_TO_RAD     (3.14159265358979323846f / 180.0f)
#define RPM_TO_RAD_S   (2.0f * 3.14159265358979323846f / 60.0f)

// Ângulo de corte por segurança (se passar de ±30 graus, desliga o motor)
#define MAX_RECOVERY_ANGLE_RAD (30.0f * DEG_TO_RAD)

// Período da malha de controle em milissegundos
// Ts = 1 ms (1000 Hz) conforme modelagem no MATLAB
#define CONTROL_LOOP_PERIOD_MS 1

static void control_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS) > 0 ? pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS) : 1;
    static int last_time_us = 0; //Current_time é calculado em us
    //const float dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f; //Divide por 1000 para deixar em segundos

    float theta = 0.0f;          // x1: Ângulo do pêndulo em radianos
    float theta_dot = 0.0f;      // x2: Velocidade angular do pêndulo em rad/s
    float omega_r = 0.0f;        // x3: Velocidade angular da roda em rad/s
    uint32_t log_counter = 0;

    ESP_LOGI(TAG, "Malha de controle em espaco de estados ativa (Ts = %d ms)...", CONTROL_LOOP_PERIOD_MS);

    while (1) {
        // 1. Leitura dos sensores
        float gyro_z_deg_s = imu_read_gyro_z();
        theta_dot = gyro_z_deg_s * DEG_TO_RAD;

        //2. Calcula dt em segundos
        int current_time_us = esp_timer_get_time();

        if (last_time_us == 0) {
            last_time_us = current_time_us;
        }

        float dt = (float)(current_time_us - last_time_us)/1000000.0f; //us -> s
        last_time_us = current_time_us;

        // Integração do ângulo do pêndulo
        theta += theta_dot * dt;

        // Leitura do encoder da roda de inércia
        float rpm = encoder_read_rpm(imu_get_i2c_bus_handle());
        omega_r = rpm * RPM_TO_RAD_S;

        // 3. Trava de segurança: Se o robô caiu, desliga o motor
        if (fabsf(theta) > MAX_RECOVERY_ANGLE_RAD) {
            motor_set_speed(0);
            if (++log_counter % 100 == 0) {
                ESP_LOGW(TAG, "EMERGENCIA: Inclinacao excessiva (%.1f deg). Motor desarmado.", theta / DEG_TO_RAD);
            }
            vTaskDelayUntil(&last_wake_time, period_ticks);
            continue;
        }

        // 4 Montagem do vetor de estados x[k] = [theta, theta_dot, omega_r]
        state_vector_t x = {
            .theta = theta,
            .theta_dot = theta_dot,
            .omega_r = omega_r
        };

        // 5. Executa a lei de controle: u = saturate(r - Kd * x)
        state_controller_output_t ctrl_out;
        int16_t duty = state_controller_update(&x, &ctrl_out);

        // 6. Atuação na ponte H (BTS7960)
        motor_set_speed(duty);

        // Telemetria periódica a cada 100 iterações (~10 Hz)
        if (++log_counter >= 100) {
            log_counter = 0;
            ESP_LOGI(TAG, "x=[th: %5.2f deg, w_p: %5.2f rad/s, w_r: %5.1f rad/s] -> Tensao: %5.2f V | Duty: %4d",
                     theta / DEG_TO_RAD, theta_dot, omega_r, ctrl_out.voltage, duty);
        }

        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  FIRMWARE SEC - CONTROLE EM ESPAÇO DE ESTADOS ");
    ESP_LOGI(TAG, "===============================================");

    // 1. Inicializa o atuador (PWM LEDC / BTS7960)
    motor_init();
    motor_set_speed(0);

    // 2. Inicializa o barramento I2C e sensores
    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha critica ao inicializar IMU!");
        return;
    }

    i2c_scanner();
    imu_calibrate();

    // 3. Inicializa o controlador em espaço de estados com os ganhos do modelo
    state_controller_init();

    // 4. Dispara a tarefa de controle em tempo real (prioridade alta)
    xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 2, // Prioridade alta para determinismo
        NULL,
        0
    );
}
