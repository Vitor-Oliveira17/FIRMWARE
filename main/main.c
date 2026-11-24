/**
 * @file main.c
 * @brief Ponto de entrada do firmware de controle em espaço de estados.
 *
 * Modos (SEC_MODE):
 *   SEC_MODE_BALANCE    -> malha de equilibrio em espaco de estados (1 kHz)
 *   SEC_MODE_SENSORS    -> mesma leitura de sensores, motor SEMPRE desligado
 *                          (use para conferir sinais e o zero do angulo)
 *   SEC_MODE_MOTOR_TEST -> teste em malha aberta do motor + encoder
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "encoder_magnetic.h"
#include "imu.h"
#include "motor.h"
#include "motor_test.h"
#include "state_controller.h"

static const char *TAG = "MAIN_SEC";

// ============================================================================
// MODO DE OPERACAO
// ============================================================================
#define SEC_MODE_BALANCE            0
#define SEC_MODE_SENSORS            1
#define SEC_MODE_MOTOR_TEST         2

#define SEC_MODE                    SEC_MODE_BALANCE    

// ============================================================================
// SINAIS E ZERO (ajustar conforme o procedimento de verificacao)
// ============================================================================
// SIGN_THETA: +1/-1. Inverte theta e theta_dot juntos. Girar o CORPO no mesmo
//             sentido em que um giro da RODA da rpm > 0 deve dar theta > 0.
#define SIGN_THETA                  (+1.0f)
// SIGN_MOTOR: +1/-1. Duty positivo deve fazer o encoder medir rpm > 0.
#define SIGN_MOTOR                  (-1)
// THETA_OFFSET_DEG: leitura de theta (graus) no ponto real de equilibrio.
#define THETA_OFFSET_DEG            (0.0f)

// ============================================================================
// FONTE DA VELOCIDADE DA RODA (omega_r)
// ============================================================================
// 0 -> SEM ENCODER: omega_r estimado pelo modelo discreto a partir da tensao
//      aplicada (omega[k+1] = Ad[2]*x[k] + Bd[2]*u[k]). O AS5600 nao e lido.
//      Estavel no modelo enquanto o ganho real do motor for >= ~0.8x o do modelo.
// 1 -> COM ENCODER: omega_r medido pelo AS5600 (I2C).
#define USE_ENCODER                 0

// ============================================================================
// FILTROS, SEGURANCA E TEMPORIZACAO
// ============================================================================
// Filtro passa-baixas (EMA) da velocidade da roda. Com Ts = 1 ms, 1 LSB do
// AS5600 equivale a ~1.5 rad/s; alpha = 0.2 -> fc ~ 36 Hz.
#define OMEGA_EMA_ALPHA             0.2f

#define ARM_ANGLE_DEG               2.0f    // arma so se |theta| < 2 graus...
#define ARM_HOLD_MS                 500     // ...continuamente por 500 ms
#define DISARM_ANGLE_DEG            30.0f   // desarma acima disso (caiu)
#define SAT_TIMEOUT_MS              1000    // desarma se saturar por 1 s seguido
#define CALIB_COUNTDOWN_S           3

#define CONTROL_LOOP_PERIOD_MS      1
#define LOG_PERIOD_MS               100

#define DEG_TO_RAD                  (3.14159265358979323846f / 180.0f)
#define RPM_TO_RAD_S                (2.0f * 3.14159265358979323846f / 60.0f)

// ============================================================================

typedef struct {
    bool armed;
    const char *status;
    float theta_deg;
    float theta_dot;
    float ax, ay, az;
    float enc_deg;
    float rpm;
    float omega_f;
    float voltage;
    int16_t duty;
    int64_t period_us;
    int64_t max_period_us;
    uint32_t imu_errors;
    uint32_t enc_errors;
} sec_snapshot_t;

static QueueHandle_t log_queue;

/**
 * Tarefa de baixa prioridade: imprimir na UART leva ~20 ms por linha a
 * 115200 baud, por isso o log nunca e feito dentro da malha de 1 kHz.
 */
static void log_task(void *pv_parameters)
{
    (void)pv_parameters;
    sec_snapshot_t s;

    while (true) {
        if (xQueueReceive(log_queue, &s, portMAX_DELAY) != pdTRUE) {
            continue;
        }
#if SEC_MODE == SEC_MODE_SENSORS
        ESP_LOGI(TAG,
                 "[SENSORES] th:%+7.2f deg dth:%+6.2f rad/s acc:(%+.2f,%+.2f,%+.2f)g | "
                 "enc:%6.2f deg %+8.1f rpm (wf:%+6.1f rad/s) | errIMU:%lu errENC:%lu",
                 s.theta_deg, s.theta_dot, s.ax, s.ay, s.az, s.enc_deg, s.rpm,
                 s.omega_f, (unsigned long)s.imu_errors, (unsigned long)s.enc_errors);
#elif USE_ENCODER
        ESP_LOGI(TAG,
                 "[%s] th:%+6.2f deg dth:%+6.2f rad/s | roda:%+8.1f rpm (wf:%+6.1f rad/s) | "
                 "u:%+6.2f V duty:%+4d | Ts:%lld us max:%lld us | errIMU:%lu errENC:%lu (%s)",
                 s.status, s.theta_deg, s.theta_dot, s.rpm, s.omega_f, s.voltage, s.duty,
                 (long long)s.period_us, (long long)s.max_period_us,
                 (unsigned long)s.imu_errors, (unsigned long)s.enc_errors,
                 esp_err_to_name(encoder_get_last_error()));
#else
        ESP_LOGI(TAG,
                 "[%s] th:%+6.2f deg dth:%+6.2f rad/s | roda estimada:%+7.1f rad/s | "
                 "u:%+6.2f V duty:%+4d | Ts:%lld us max:%lld us | errIMU:%lu",
                 s.status, s.theta_deg, s.theta_dot, s.omega_f, s.voltage, s.duty,
                 (long long)s.period_us, (long long)s.max_period_us,
                 (unsigned long)s.imu_errors);
#endif
    }
}

static void control_task(void *pv_parameters)
{
    (void)pv_parameters;

    const bool balance_mode = (SEC_MODE == SEC_MODE_BALANCE);
    const TickType_t period_ticks =
        pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS) > 0 ?
        pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS) : 1;
    const float dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
    const uint32_t log_every = LOG_PERIOD_MS / CONTROL_LOOP_PERIOD_MS;
    const uint32_t arm_hold_cycles = ARM_HOLD_MS / CONTROL_LOOP_PERIOD_MS;
    const uint32_t sat_limit_cycles = SAT_TIMEOUT_MS / CONTROL_LOOP_PERIOD_MS;
    const float theta_offset = THETA_OFFSET_DEG * DEG_TO_RAD;
    const float arm_angle = ARM_ANGLE_DEG * DEG_TO_RAD;
    const float disarm_angle = DISARM_ANGLE_DEG * DEG_TO_RAD;
#if USE_ENCODER
    const i2c_master_bus_handle_t bus = imu_get_i2c_bus_handle();
#endif

    bool armed = false;
    const char *status = balance_mode ? "AGUARDANDO" : "SENSORES";
    uint32_t arm_counter = 0;
    uint32_t sat_counter = 0;
    uint32_t log_counter = 0;
    uint32_t imu_errors = 0;
    float omega_f = 0.0f;
    float theta = 0.0f;
    float theta_dot = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float rpm = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();
    int64_t last_cycle_start_us = esp_timer_get_time();
    int64_t max_cycle_period_us = 0;

    ESP_LOGI(TAG, "Malha ativa (Ts = %d ms). Modo: %s | omega_r: %s", CONTROL_LOOP_PERIOD_MS,
             balance_mode ? "EQUILIBRIO" : "SOMENTE SENSORES (motor desligado)",
             USE_ENCODER ? "ENCODER AS5600" : "ESTIMADO PELO MODELO (sem encoder)");
    if (balance_mode) {
        ESP_LOGW(TAG, "Arma quando |theta| < %.1f deg por %d ms.", ARM_ANGLE_DEG, ARM_HOLD_MS);
    }

    while (true) {
        const int64_t cycle_start_us = esp_timer_get_time();
        const int64_t cycle_period_us = cycle_start_us - last_cycle_start_us;
        last_cycle_start_us = cycle_start_us;
        if (cycle_period_us > max_cycle_period_us) {
            max_cycle_period_us = cycle_period_us;
        }

        // ---------------- Leitura dos sensores ----------------
        float theta_raw = 0.0f;
        float gyro_z_deg_s = 0.0f;
        const esp_err_t imu_err = imu_read_state(dt, &theta_raw, &gyro_z_deg_s,
                                                 &ax, &ay, &az);
#if USE_ENCODER
        rpm = encoder_read_rpm(bus);
        omega_f += OMEGA_EMA_ALPHA * (rpm * RPM_TO_RAD_S - omega_f);
#endif

        int16_t duty = 0;
        float voltage = 0.0f;

        if (imu_err != ESP_OK) {
            imu_errors++;
            if (armed) {
                armed = false;
                status = "DESARMADO-IMU";
            }
            arm_counter = 0;
        } else {
            theta = SIGN_THETA * theta_raw - theta_offset;
            theta_dot = SIGN_THETA * gyro_z_deg_s * DEG_TO_RAD;
            const bool valid = isfinite(theta) && isfinite(theta_dot) && isfinite(omega_f);

            if (balance_mode) {
                // ---------------- Logica de armar / desarmar ----------------
                if (armed) {
                    if (!valid || fabsf(theta) > disarm_angle) {
                        armed = false;
                        arm_counter = 0;
                        status = "DESARMADO-QUEDA";
                    }
                } else if (valid && fabsf(theta) < arm_angle) {
                    if (++arm_counter >= arm_hold_cycles) {
                        armed = true;
                        sat_counter = 0;
                        status = "ARMADO";
                    }
                } else {
                    arm_counter = 0;
                }

                // ---------------- Lei de controle ----------------
                if (armed) {
                    const state_vector_t state = {
                        .theta = theta,
                        .theta_dot = theta_dot,
                        .omega_r = omega_f,
                    };
                    state_controller_output_t out;
                    const int16_t ctrl_duty = state_controller_update(&state, &out);

                    if (out.u_raw != out.voltage) {
                        sat_counter++;
                    } else {
                        sat_counter = 0;
                    }

                    if (sat_counter >= sat_limit_cycles) {
                        armed = false;
                        arm_counter = 0;
                        status = "DESARMADO-SATURACAO";
                    } else {
                        duty = (int16_t)(SIGN_MOTOR * ctrl_duty);
                        voltage = out.voltage;
                    }
                }
            }
        }

        if (!armed) {
            duty = 0;
            voltage = 0.0f;
        }
        motor_set_speed(duty);

#if !USE_ENCODER
        // Estimador de omega_r pelo modelo: propaga a 3a linha da planta discreta
        // com a tensao realmente aplicada (0 V quando desarmado).
        {
            const state_vector_t x_now = {
                .theta = theta,
                .theta_dot = theta_dot,
                .omega_r = omega_f,
            };
            state_vector_t x_next;
            state_controller_predict(&x_now, voltage, &x_next, NULL);
            omega_f = isfinite(x_next.omega_r) ? x_next.omega_r : 0.0f;
            if (!armed) {
                // Desarmado: a ponte freia a roda; zera a estimativa.
                omega_f = 0.0f;
            }
        }
#endif

        // ---------------- Log (somente copia; impressao em outra tarefa) ----------------
        if (++log_counter >= log_every) {
            log_counter = 0;
            const sec_snapshot_t snap = {
                .armed = armed,
                .status = status,
                .theta_deg = theta / DEG_TO_RAD,
                .theta_dot = theta_dot,
                .ax = ax, .ay = ay, .az = az,
#if USE_ENCODER
                .enc_deg = encoder_get_last_angle(),
                .enc_errors = encoder_get_error_count(),
#else
                .enc_deg = 0.0f,
                .enc_errors = 0,
#endif
                .rpm = rpm,
                .omega_f = omega_f,
                .voltage = voltage,
                .duty = duty,
                .period_us = cycle_period_us,
                .max_period_us = max_cycle_period_us,
                .imu_errors = imu_errors,
            };
            xQueueOverwrite(log_queue, &snap);
            max_cycle_period_us = 0;
        }

        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Firmware SEC: controle em espaco de estados");

    motor_init();
    motor_set_speed(0);

    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha critica ao inicializar a IMU; motor mantido desligado.");
        return;
    }

#if USE_ENCODER
    // Diagnostico do barramento I2C e do encoder (deve listar 0x36 e 0x68).
    i2c_scanner();
    encoder_diagnose(imu_get_i2c_bus_handle());
#endif

    // O zero de theta e a postura do robo durante a calibracao e no inicio da
    // malha. Segure-o no ponto de equilibrio ate aparecer "Malha ativa".
    for (int s = CALIB_COUNTDOWN_S; s > 0; s--) {
        ESP_LOGW(TAG, "Segure o robo PARADO no ponto de equilibrio... %d", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (imu_calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao calibrar a IMU; motor mantido desligado.");
        return;
    }

#if SEC_MODE == SEC_MODE_MOTOR_TEST
    ESP_LOGW(TAG, "Modo de teste do motor ativo; a malha de equilibrio nao sera iniciada.");
    if (motor_test_start(imu_get_i2c_bus_handle()) != ESP_OK) {
        ESP_LOGE(TAG, "Nao foi possivel iniciar o teste do motor.");
        motor_set_speed(0);
    }
    return;
#endif

    state_controller_init();

    log_queue = xQueueCreate(1, sizeof(sec_snapshot_t));
    if (log_queue == NULL ||
        xTaskCreate(log_task, "sec_log", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Nao foi possivel criar a tarefa de log.");
        return;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        control_task, "control_task", 4096, NULL,
        configMAX_PRIORITIES - 2, NULL, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Nao foi possivel criar a tarefa de controle.");
        motor_set_speed(0);
    }
}
