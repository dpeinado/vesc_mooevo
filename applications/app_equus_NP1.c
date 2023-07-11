/*
 * app_equus.c
 *
 *  Created on: 26 jul. 2022
 *      Author: Usuario01
 */


#include "conf_general.h"

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "commands.h"
#include "imu/imu.h"
#include "imu/ahrs.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"

#include <equus_model.h>
#include <mooevo_pid.h>
extern MEQUUSType miEquusModel;
extern LoopManagerType miLoop;
extern balance_config balance_conf;
extern PID_Type freno1, freno2, controlCabeceo, controlAngulo;

#include <math.h>
#include <stdio.h>

// CAN PARAMETERS
#define MAX_CAN_AGE 0.1
#define PERIODO 50


// Balance thread
static THD_FUNCTION(equus_thread, arg);
static THD_WORKING_AREA(equus_thread_wa, 2048); // 2kb stack for this thread

static thread_t *app_thread;


// Function Prototypes
static void set_current(float current);
static void terminal_printData(int argc, const char **argv);
static void set_sensorData(int argc, const char **argv);




void app_custom_configure(app_configuration *conf) {
  initEquusModel(conf);
  initLoopManager(conf);
}

void app_custom_start(void) {
    // First start only, override state to startup
  miEquusModel.estado = BO;
  terminal_register_command_callback(
      "dame",
      "Valores",
      "",
      terminal_printData);
  terminal_register_command_callback(
      "senE",
      "senE btnTR, snsrHM",
      "btnTR, snsrHM",
      set_sensorData);

  app_thread = chThdCreateStatic(equus_thread_wa, sizeof(equus_thread_wa), NORMALPRIO, equus_thread, NULL);
}

void app_custom_stop(void) {
    if(app_thread != NULL){
        chThdTerminate(app_thread);
        chThdWait(app_thread);
    }
    set_current(0);
}


static void set_current(float current){
    // Reset the timeout
    timeout_reset();
    mc_interface_set_current_off_delay(miLoop.motor_timeout);
    mc_interface_set_current(current);
    for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
        can_status_msg *msg = comm_can_get_status_msg_index(i);
        if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
            comm_can_set_current_off_delay(msg->id, current, miLoop.motor_timeout);
        }
    }
}


static THD_FUNCTION(equus_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_EQUUS");
//    palSetPadMode(HW_ADC_EXT_GPIO, HW_ADC_EXT_PIN, PAL_MODE_INPUT_PULLDOWN);
//    palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN, PAL_MODE_INPUT_PULLDOWN);
    while (!chThdShouldTerminateX()) {
      timeout_reset();
      setLoopVariables();
      timeout_reset();
      getEquusSensorValues();
      timeout_reset();
      makeEquusStateTransition();
      timeout_reset();
      makeEquusStateDriving();
      timeout_reset();
      chThdSleep(miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
    }
}


static void terminal_PIDPrintData(PID_Type mpid){
  commands_printf("Tipo: %d\t Dirección: %d\tModo: %d",
                  mpid.tipo, mpid.direccion, mpid.modo);
  commands_printf("Kp = %f\tKi = %f\tKd = %f    *************************  OUTPUT = %f",
                  (double) mpid.kp, (double) mpid.ki, (double) mpid.kd, (double) mpid.output);
  commands_printf("pTerm = %f\tiTerm = %f\tdTerm = %f",
                  (double) mpid.p_term, (double) mpid.i_term, (double) mpid.d_term);
  commands_printf("dt  = %f\tLastInput = %f\tSetPoint = %f\tKd_filter = %f",
                  (double) mpid.dt, (double) mpid.last_input, (double) mpid.set_point, (double) mpid.kd_filter);
  commands_printf("D_filtered = %f\tMinV = %f\tMaxV = %f",
                  (double) mpid.d_filtered, (double) mpid.min_value, (double) mpid.max_value);
}

static void terminal_printData(int argc, const char **argv) {
  (void)argc;
  (void)argv;
  char esta[5];
  switch(miEquusModel.estado) {
    case(BO):
        sprintf(esta, "%s", "BO");
        break;
    case(VP):
        sprintf(esta, "%s", "VP");
        break;
    case(VB):
        sprintf(esta, "%s", "VB");
        break;
    case(TR):
        sprintf(esta, "%s", "TR");
        break;
    case(MW):
        sprintf(esta, "%s", "MW");
        break;
    case(AE):
        sprintf(esta, "%s", "AE");
        break;
    case(FA):
        sprintf(esta, "%s", "FA");
        break;
    case(TZ):
        sprintf(esta, "%s", "TZ");
        break;
  }
  commands_printf("\n\r EQUUS************");
  commands_printf("Estado: %s\t btnTR: %d \t snsrHM: %d \t AngOk: %d \t Abnormal1: %d",
                  esta, miEquusModel.btnTR, miEquusModel.snsrHM, miEquusModel.AngOk, miEquusModel.Abnormal1);
  commands_printf("setpoint: %f \t pitch_angle: %f \t pitch_vel: %f",
                  (double) miEquusModel.setpoint,
                  (double) miEquusModel.pitch_angle,
                  (double) miEquusModel.pitch_vel);
  commands_printf("last_pitch_angle: %f \t roll_angle: %f", (double) miEquusModel.last_pitch_angle, (double) miEquusModel.roll_angle);
  commands_printf("************ FRENO 1 ***************");
  terminal_PIDPrintData(freno1);
  commands_printf("************ FRENO 2 ***************");
  terminal_PIDPrintData(freno2);
  commands_printf("************ CABECEO ***************");
  terminal_PIDPrintData(controlCabeceo);
  commands_printf("************ VELOCIDAD ***************");
  terminal_PIDPrintData(controlAngulo);
  commands_printf("**************************************");
  commands_printf("erpm_M: %f", (double) miEquusModel.erpm_M);
  commands_printf("erpm_S: %f", (double) miEquusModel.erpm_S);

  commands_printf("\n\r LOOP************");
  commands_printf("current_time: %d", miLoop.current_time);
  commands_printf("last_time: %d", miLoop.last_time);
  commands_printf("diff_time: %f", (double) ST2US(miLoop.diff_time)/(double) 1.0e6);
  commands_printf("loop_overshoot: %d", miLoop.loop_overshoot);
  commands_printf("loop_time: %d", miLoop.loop_time);
  commands_printf("loop_time_filter: %d", miLoop.loop_time_filter);
  commands_printf("brake_timeout: %f", (double) miLoop.brake_timeout);
  commands_printf("startup_step_size: %f", (double) miLoop.startup_step_size);
  commands_printf("filtered_loop_overshoot: %f", (double) miLoop.filtered_loop_overshoot);
  commands_printf("loop_overshoot_alpha: %f", (double) miLoop.loop_overshoot_alpha);
  commands_printf("filtered_diff_time: %f", (double) miLoop.filtered_diff_time);

  commands_printf("\n\r CONFIG************");
  commands_printf("MIN_RPM: %d", MIN_RPM);
  commands_printf("MIN_CURR: %f", (double) MIN_CURR);
  commands_printf("MAX_PITCH: %f", (double) MAX_PITCH);
  commands_printf("MAX_ROLL: %f", (double) MAX_ROLL);
  commands_printf("MAX_PITCH_DELTA: %f", (double) MAX_PITCH_DELTA);
  commands_printf("MIN_PITCH_VEL: %f",(double)  MIN_PITCH_VEL);
  commands_printf("MAX_PITCH_VEL: %f", (double) MAX_PITCH_VEL);
  //commands_printf("MAX_ERPM_ABNORMAL1: %f", (double) miEquusModel.max_erpm_AE);
}

static void set_sensorData(int argc, const char **argv) {
  if (argc == 3) {
    int btnTR = 0;
    int snsrHM = 0;
    sscanf(argv[1], "%d", &btnTR);
    sscanf(argv[2], "%d", &snsrHM);
    if (btnTR == 0) miEquusModel.btnTR = false;
    else miEquusModel.btnTR = true;
    if (snsrHM == 0) miEquusModel.snsrHM = false;
    else miEquusModel.snsrHM = true;
  } else {
    commands_printf("\n\r Uso senE btnTR, snsrHM");
  }
}
