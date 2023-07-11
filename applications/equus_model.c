#include <equus_model.h>
#include "utils.h"
#include "math.h"
#include "commands.h"

MEQUUSType miEquusModel;
//static float gyro[3];
balance_config balance_conf;
static float max_current;
static float dt;

PID_Type freno1, freno2, controlCabeceo, controlAngulo;
//PDPI_Type autobalance;

LoopManagerType miLoop;


uint16_t n_espera_indice = 0;


void initLoopManager(app_configuration *conf){
  balance_conf = conf->app_balance_conf;
  // Set calculated values from config
  miLoop.loop_time = US2ST((int)((1000.0 / balance_conf.hertz) * 1000.0));

  miLoop.motor_timeout = ((1000.0 / balance_conf.hertz)/1000.0) * 20; // Times 20 for a nice long grace period
  miLoop.startup_step_size = balance_conf.startup_speed / balance_conf.hertz;

  // Init Filters
  if(balance_conf.loop_time_filter > 0){
    miLoop.loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter/(2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter+1);
  }
  miLoop.loop_time_filter = balance_conf.loop_time_filter;
  // Reset loop time variables
  miLoop.last_time = 0;
  miLoop.filtered_loop_overshoot = 0;
}

void setLoopVariables(void){

  miLoop.current_time = chVTGetSystemTimeX();
  if(miLoop.last_time == 0){
   miLoop.last_time = miLoop.current_time;
  }
  miLoop.diff_time = miLoop.current_time - miLoop.last_time;
  miLoop.filtered_diff_time = 0.03 * miLoop.diff_time + 0.97 * miLoop.filtered_diff_time; // Purely a metric
  miLoop.last_time = miLoop.current_time;
  if(miLoop.loop_time_filter > 0){
   miLoop.loop_overshoot = miLoop.diff_time - (miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
   miLoop.filtered_loop_overshoot = miLoop.loop_overshoot_alpha * miLoop.loop_overshoot + (1-miLoop.loop_overshoot_alpha)*miLoop.filtered_loop_overshoot;
  }
}

void resetFreno(void){
  reset_PID(&freno1);
  reset_PID(&freno2);
  max_current = 0.0;
}

void resetControlAngulo(void){
  reset_PID(&controlAngulo);
}

void setZeroAngle(void){
  miEquusModel.setpoint = miEquusModel.pitch_angle;
  set_PID_setPoint(&controlCabeceo, miEquusModel.setpoint);
}


void initEquusModel(app_configuration *conf)
{
  balance_conf = conf->app_balance_conf;
  miEquusModel.estado = BO;
  miEquusModel.btnTR = false;
  miEquusModel.snsrHM = false;
  miEquusModel.AngOk = false;
  miEquusModel.Abnormal1 = false;
  miEquusModel.setpoint = 0.0;
  miEquusModel.pitch_angle = miEquusModel.last_pitch_angle = miEquusModel.roll_angle = 0.0;
  miEquusModel.pitch_vel = miEquusModel.pitch_vel_filtered = 0.0;
  miEquusModel.kp_ang = balance_conf.yaw_kp/100.0;
  miEquusModel.kd_ang = balance_conf.yaw_ki/100.0;
  miEquusModel.kp_vel = balance_conf.yaw_kd/100.0;
  miEquusModel.ki_vel = balance_conf.roll_steer_kp/100.0;
  miEquusModel.kd_vel = balance_conf.roll_steer_erpm_kp/100.0;
  miEquusModel.kp_brake = balance_conf.kp/1000.0;
  miEquusModel.ki_brake = balance_conf.ki/1000.0;
  miEquusModel.kd_brake = balance_conf.kd/1000.0;
  miEquusModel.erpm_M = miEquusModel.erpm_S = 0.0;
  miEquusModel.erpm_M_last = miEquusModel.erpm_S_last = 0.0;
  miEquusModel.erpm_M_filtered = miEquusModel.erpm_S_filtered = 0.0;
  miEquusModel.abs_max_rpm = miEquusModel.avg_rpm = 0.0;
  miEquusModel.max_erpm_AE = balance_conf.tiltback_constant_erpm;
  miEquusModel.np2 = (balance_conf.yaw_current_clamp>0.5) ? true : false;

  max_current = 0.0;

  init_PID(&freno1, AWINDUP, DIRECT, miEquusModel.kp_brake, miEquusModel.ki_brake,
           miEquusModel.kd_brake, 0.0, 0.001, 0.2, -1.0, 1.0);
  init_PID(&freno2, AWINDUP, DIRECT, miEquusModel.kp_brake, miEquusModel.ki_brake,
             miEquusModel.kd_brake, 0.0, 0.001, 0.2, -1.0, 1.0);
  init_PID(&controlCabeceo, AWINDUP, REVERSE, miEquusModel.kp_ang, 0.0, miEquusModel.kd_ang,
           0.0, 0.001, 0.2, -1.0, 1.0);
  init_PID(&controlAngulo, SATURATION, DIRECT, miEquusModel.kp_vel, miEquusModel.ki_vel, miEquusModel.kd_vel,
           0.0, 0.001, 0.2, -5.0, 5.0);
  set_mode_PID(&freno2, AUTO);
  set_mode_PID(&freno1, AUTO);
  set_mode_PID(&controlCabeceo, AUTO);
  set_mode_PID(&controlAngulo, AUTO);
//  set_mode_PDPI(&autobalance, AUTO);
}

void getEquusSensorValues(void){
  // aqu� tengo que leer los estados de los sensores externos: btnTR y snsrHM,
  // las ERPM de ambos motores, y los datos procedentes de la IMU


  static float pitch_vel_filtered;
//  static bool previousBTN = false;
//  static systime_t first_in_a_row_time = 0;
//  static uint8_t n_in_a_row = 0;

  dt = ST2US(miLoop.diff_time)/1.0e6;
  if (miLoop.diff_time==0) dt = 0.002;
//  previousBTN = miEquusModel.btnTR;
  miEquusModel.snsrHM = miEquusModel.btnTR = false;
  if (ADC_VOLTS(ADC_IND_EXT)>=2.9) miEquusModel.snsrHM = true;  // leo como entradas anal�gicas. como digitales no funciona
  if (ADC_VOLTS(ADC_IND_EXT2) >=2.9) miEquusModel.btnTR = true; // leo como entradas anal�gicas. como digitales no funciona

 /* if (previousBTN != miEquusModel.btnTR) {
    if (n_in_a_row == 0) {
      first_in_a_row_time = chVTGetSystemTimeX();
      n_in_a_row++;
    } else {
      if (ST2MS(chVTTimeElapsedSinceX(first_in_a_row_time)) > MAX_TIME_COMMAND ) {
        n_in_a_row = 0;
      } else {
        n_in_a_row++;
        if (n_in_a_row == N_TO_CHANGE_MODE){
          miEquusModel.np2 = !miEquusModel.np2;
          n_in_a_row = 0;
        }
      }
    }
  }*/
  // ****************
  // DATOS IMU
  miEquusModel.last_pitch_angle = miEquusModel.pitch_angle;
  miEquusModel.pitch_angle = RAD2DEG_f(imu_get_roll());
  miEquusModel.roll_angle = RAD2DEG_f(imu_get_pitch());
  //imu_get_gyro(gyro);
  miEquusModel.pitch_vel = (miEquusModel.pitch_angle-miEquusModel.last_pitch_angle)/dt;
  UTILS_LP_FAST(pitch_vel_filtered, miEquusModel.pitch_vel, 0.2); // filter D
  //miEquusModel.pitch_vel = -gyro[0];
  //commands_printf("pitch = %f, giro = %f", (double) miEquusModel.pitch_angle, (double) miEquusModel.pitch_vel);
  miEquusModel.AngOk = false;
  miEquusModel.Abnormal1 = false;
  // ************************
  // Check correct attitude, AngOk, Abnormal1
  if (fabsf(miEquusModel.pitch_angle) <  MAX_PITCH &&
      fabsf(miEquusModel.roll_angle)  <  MAX_ROLL &&
      fabsf(miEquusModel.pitch_vel)   <  MIN_PITCH_VEL)
    miEquusModel.AngOk = true;
  if (fabsf(miEquusModel.pitch_angle)>= (MAX_PITCH + MAX_PITCH_DELTA) ||
      fabsf(miEquusModel.pitch_vel) >= MAX_PITCH_VEL ||
      fabsf(miEquusModel.roll_angle) >= MAX_ROLL){
    miEquusModel.Abnormal1 = true;
  }

  // *****************
  // DATOS DE ERPM
  miEquusModel.erpm_M_last = miEquusModel.erpm_M;
  miEquusModel.erpm_S_last = miEquusModel.erpm_S;
  miEquusModel.erpm_M = mc_interface_get_rpm();
  for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
     can_status_msg *msg = comm_can_get_status_msg_index(i);
     if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
       miEquusModel.erpm_S = msg->rpm;
     }
  }
  UTILS_LP_MOVING_AVG_APPROX(miEquusModel.erpm_M_filtered, miEquusModel.erpm_M, RPM_FILTER_SAMPLES);
  UTILS_LP_MOVING_AVG_APPROX(miEquusModel.erpm_S_filtered, miEquusModel.erpm_S, RPM_FILTER_SAMPLES);
  miEquusModel.erpm_M = miEquusModel.erpm_M_filtered;
  miEquusModel.erpm_S = miEquusModel.erpm_S_filtered;
  miEquusModel.abs_max_rpm = (fabsf(miEquusModel.erpm_M) > fabsf(miEquusModel.erpm_S)) ? fabsf(miEquusModel.erpm_M) : fabsf(miEquusModel.erpm_S);
  miEquusModel.avg_rpm = (miEquusModel.erpm_M + miEquusModel.erpm_S)/2.0;
}

bool checkAbnormalEquilibrium(void) {
  //return false;
  if (fabsf(miEquusModel.erpm_M) >  miEquusModel.max_erpm_AE ||
      fabsf(miEquusModel.erpm_S) >  miEquusModel.max_erpm_AE){
    return true;
    commands_printf("Fallo: %f o %f >= %f", fabs(miEquusModel.erpm_M), fabs(miEquusModel.erpm_S), (double) miEquusModel.max_erpm_AE);
  }
  else
    return false;
}

void makeEquusStateTransition(void){
  switch(miEquusModel.estado){
    case(BO): // Se pasa autom�ticamente a veh�culo bloqueado
        miEquusModel.estado = VB;
        commands_printf("A VB");
        resetFreno();
        break;
    case(VP): // Si estando parado se observa que la velocidad aumenta, se pasa a veh�culo bloqueado
        if (miEquusModel.abs_max_rpm > MIN_RPM){
          miEquusModel.estado = VB;
          commands_printf("A VB desde VP");
          resetFreno();
        }
        if (miEquusModel.btnTR){
          miEquusModel.estado = TR;
          commands_printf("A TR desde VP");
        }
        break;
    case(VB): // Si est� parado y no hace casi fuerza, se pasa a veh�culo parado para ahorrar bater�a, si btnTR a TR
        if (miEquusModel.abs_max_rpm < MIN_RPM && max_current < MIN_CURR) {
          miEquusModel.estado = VP;
          commands_printf("A VP desde VB");
        }
        if (miEquusModel.btnTR){
          miEquusModel.estado = TR;
          commands_printf("A TR desde VB");
        }
        break;
    case(TR): // Estando en TR se sale cuando se suelta el bot�n. Si OK, MW, si no, se vuelve a VB
        if (!miEquusModel.btnTR){
          if (!miEquusModel.snsrHM && miEquusModel.AngOk){
            miEquusModel.estado = MW;
            commands_printf("A MW de TR");
          } else {
            miEquusModel.estado = VB;
            commands_printf("A VB desde TR");
            resetFreno();
          }
        }
        break;
    case(MW): // Estando en MW se sale por condici�n anormal, o por que se suelta el asa
        if (miEquusModel.Abnormal1){
          miEquusModel.estado = FA;
          commands_printf("A FA desde MW");
        }
        if (miEquusModel.snsrHM){
          if (miEquusModel.np2) {
            miEquusModel.estado = AE;
            commands_printf("A AE desde MW");
          } else {
            miEquusModel.estado = VB;
            commands_printf("A VB desde MW (hm)");
          }
        }
        if (miEquusModel.btnTR) {
          miEquusModel.estado = TR;
          commands_printf("A TR desde MW");
        }
        break;
    case(AE):
        if (miEquusModel.Abnormal1){
          miEquusModel.estado = FA;
          commands_printf("A FA desde AE");
        }
        if(!miEquusModel.snsrHM){
          miEquusModel.estado = TZ;
          commands_printf("A TZ desde AE");
        }
        if (checkAbnormalEquilibrium()){
          miEquusModel.estado = FA;
          commands_printf("A FA desde AE (EQ. FAILURE)");
        }
        break;
    case(TZ):
        miEquusModel.estado = MW;
        commands_printf("A MW (de TZ)");
        break;
    case(FA):
        miEquusModel.estado = VB;
        commands_printf("A VB desde FA");
        resetFreno();
        resetControlAngulo();
        break;
  }
}

void vehicleBlocking(void){
  if (miEquusModel.abs_max_rpm > MIN_RPM) {
    float out_1, out_2;
    out_1 = compute_PID(&freno1, miEquusModel.erpm_M, dt);
    out_2 = compute_PID(&freno2, miEquusModel.erpm_S, dt);
    max_current = (fabsf(out_1) > fabsf(out_2)) ? fabsf(out_1): fabsf(out_2);
    mc_interface_set_current_rel(out_1);
    for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
      can_status_msg *msg = comm_can_get_status_msg_index(i);
      if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
        comm_can_set_current_rel(msg->id, out_2);
      }
    }
  }
}

void vehicleWalkerMode(void){

  float output;
  output = compute_PID(&controlCabeceo, miEquusModel.pitch_angle, dt);
  mc_interface_set_current_rel(output);
  for (int i= 0; i < CAN_STATUS_MSGS_TO_STORE; i++){
    can_status_msg *msg = comm_can_get_status_msg_index(i);
    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
      comm_can_set_current_rel(msg->id, output);
    }
  }
}

void vehicleSelfBalanceMode(void){
  float output, angulo_cero;
  if (n_espera_indice < FREQ_VEL_PI) {
    n_espera_indice++;
    output = compute_PID(&controlCabeceo, miEquusModel.pitch_angle, dt);
  } else {
    n_espera_indice = 0;
    angulo_cero = compute_PID(&controlAngulo, miEquusModel.avg_rpm, dt);
    miEquusModel.setpoint = angulo_cero;
    set_PID_setPoint(&controlCabeceo, angulo_cero);
    output = compute_PID(&controlCabeceo, miEquusModel.pitch_angle, dt);
  }
  mc_interface_set_current_rel(output);
  for (int i= 0; i < CAN_STATUS_MSGS_TO_STORE; i++){
    can_status_msg *msg = comm_can_get_status_msg_index(i);
    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
      comm_can_set_current_rel(msg->id, output);
    }
  }
}

void makeEquusStateDriving(void){
  switch(miEquusModel.estado){
    case(BO):
        break;
    case(VP):
        break;
    case(VB):
        vehicleBlocking();
        break;
    case(TR):
        vehicleBlocking();
        setZeroAngle();
        break;
    case(MW):
        vehicleWalkerMode();
        break;
    case(AE):
        vehicleSelfBalanceMode();
        break;
    case(TZ):
//        setZeroAngle(); // let's see if this works
        break;
    case(FA):
        break;
  }
}
