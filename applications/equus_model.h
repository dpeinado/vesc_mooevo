/*
 * equus_model.h
 *
 *  Created on: 26 jul. 2022
 *      Author: Usuario01
 */

#ifndef APPLICATIONS_EQUUS_MODEL_H_
#define APPLICATIONS_EQUUS_MODEL_H_

#include "ch.h" // ChibiOS
#include "datatypes.h"
#include "mc_interface.h" // Motor control functions
#include "comm_can.h"
#include "imu/imu.h"
#include "imu/ahrs.h"

#include "mooevo_pid.h"


// CAN PARAMETERS
#define MAX_CAN_AGE             0.1
#define RPM_FILTER_SAMPLES      10
#define MIN_RPM                 balance_conf.fault_adc_half_erpm
#define MIN_CURR                (balance_conf.booster_current/10.0)
#define MAX_PITCH               balance_conf.startup_pitch_tolerance
#define MAX_ROLL                balance_conf.startup_roll_tolerance
#define MAX_PITCH_DELTA         balance_conf.booster_angle
#define MIN_PITCH_VEL           balance_conf.tiltback_lv
#define MAX_PITCH_VEL           (balance_conf.tiltback_hv*5.0)
#define FREQ_VEL_PI             balance_conf.brake_timeout
#define MAX_TIME_COMMAND        1500
#define N_TO_CHANGE_MODE        8


// nivel de prestaciones 1: Booting, Vehículo Parado, Vehículo Bloqueado, Toma de Referencia, Modo Walker
//                          BO, VP, VB, TR, MW, FA
// nivel de prestaciones 2: Booting, Vehículo Parado, Vehículo Bloqueado, Autoequilibrio
//                          Modo Walker, Toma de Referencia, Fallo Autoequilibrio, Toma Zero
//                          BO, VP, VB, AE, MW, TR, FA, TZ
typedef enum {
  BO,
  VP,
  VB,
  TR,
  MW,
  AE,
  FA,
  TZ
} TipoEstado;

typedef struct {
  TipoEstado estado;
  bool btnTR; //botón toma de referencia
  bool snsrHM; // sensor hombre muerto
  bool AngOk;
  bool Abnormal1;
  float setpoint;
  float pitch_angle;
  float pitch_vel;
  float pitch_vel_filtered;
  float last_pitch_angle;
  float roll_angle;
  float kp_ang;
  float kd_ang;
  float kp_vel;
  float ki_vel;
  float kd_vel;
  float kp_brake;
  float ki_brake;
  float kd_brake;
  float erpm_M;
  float erpm_S;
  float erpm_M_last;
  float erpm_S_last;
  float erpm_M_filtered;
  float erpm_S_filtered;
  float abs_max_rpm;
  float avg_rpm;
  float max_erpm_AE;
  bool  np2;
} MEQUUSType;


typedef struct {
  systime_t current_time, last_time, diff_time, loop_overshoot;
  systime_t loop_time;
  uint16_t loop_time_filter;
  float startup_step_size;
  float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
  float motor_timeout;
  systime_t brake_timeout;
} LoopManagerType;

void initLoopManager(app_configuration *conf);
void setLoopVariables(void);
void initEquusModel(app_configuration *conf);
void getEquusSensorValues(void);
void makeEquusStateTransition(void);
void makeEquusStateDriving(void);

#endif /* APPLICATIONS_EQUUS_MODEL_H_ */
