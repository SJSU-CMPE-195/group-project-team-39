/*
 * Gantry Firmware — Apalis iMX8QM Cortex-M4F Core 0
 *
 * FreeRTOS application controlling two iSV57T BLDC motors via the M4
 * subsystem's own RGPIO peripherals (pads at default ALT0):
 *
 *   Motor 1: CM4_0__RGPIO  pin 0 = DIR, pin 1 = PUL  (Ixora X27 pins 13/14)
 *   Motor 2: CM4_1__RGPIO  pin 0 = DIR, pin 1 = PUL  (Ixora X27 pins 15/16)
 */

extern "C" {
#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "clock_config.h"
}

#include "delay_us.h"
#include "iSV57T.hpp"

/* ---------- Pin assignments ---------------------------------------------- */

#define M1_DIR_PIN       0U
#define M1_PUL_PIN       1U
#define M2_DIR_PIN       0U
#define M2_PUL_PIN       1U

#define PULSE_PER_REV    2000U

/* ---------- FreeRTOS task parameters ------------------------------------- */

#define MOTOR_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
#define MOTOR_PRIORITY   (tskIDLE_PRIORITY + 2)

/* ---------- Task handles ------------------------------------------------- */

static TaskHandle_t s_motor1_handle;
static TaskHandle_t s_motor2_handle;

/* ---------- Motor instances (constructed after board init) ---------------- */

static iSV57T *s_motor1;
static iSV57T *s_motor2;

/* ---------- Motor demo task ---------------------------------------------- */

static void motor_task(void *pvParameters)
{
    iSV57T *motor = static_cast<iSV57T *>(pvParameters);

    for (;;)
    {
        motor->rotate(iSV57T::CW, 360.0f);
        vTaskDelay(pdMS_TO_TICKS(500));

        motor->rotate(iSV57T::CCW, 360.0f);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------- FreeRTOS hooks ----------------------------------------------- */

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                               char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

/* ---------- Main --------------------------------------------------------- */

int main(void)
{
    sc_ipc_t ipc;

    ipc = BOARD_InitRpc();
    BOARD_InitBootClocks();
    BOARD_InitMemory();

    /* Power on both M4 RGPIO peripherals */
    sc_pm_set_resource_power_mode(ipc, SC_R_M4_0_RGPIO, SC_PM_PW_MODE_ON);
    sc_pm_set_resource_power_mode(ipc, SC_R_M4_1_RGPIO, SC_PM_PW_MODE_ON);

    delay_us_init();

    /* Motor 1 on CM4_0 RGPIO (pins 0/1), Motor 2 on CM4_1 RGPIO (pins 0/1) */
    static iSV57T motor1(CM4_0__RGPIO, M1_DIR_PIN, M1_PUL_PIN, PULSE_PER_REV);
    static iSV57T motor2(CM4_1__RGPIO, M2_DIR_PIN, M2_PUL_PIN, PULSE_PER_REV);

    s_motor1 = &motor1;
    s_motor2 = &motor2;

    xTaskCreate(motor_task, "Motor1", MOTOR_STACK_SIZE,
                s_motor1, MOTOR_PRIORITY, &s_motor1_handle);

    xTaskCreate(motor_task, "Motor2", MOTOR_STACK_SIZE,
                s_motor2, MOTOR_PRIORITY, &s_motor2_handle);

    vTaskStartScheduler();

    for (;;) {}
}
