/*
 * app_0_Mooevo.c
 *
 *  Created on: Sep 18, 2023
 *      Author: Diego Peinado Martín
 */

#include "app.h"
#include "ch.h"
#include "hal.h"

//#include "stm32f4xx_conf.h"
// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// mooevo includes
#include "mooevo_pid.h"
#include "app_0_Mooevo.h"

#define MAX_CAN_AGE								0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES								5
#define RPM_FILTER_SAMPLES					8

// Threads
static THD_FUNCTION(mooevoThread, arg);
static THD_WORKING_AREA(mooevoThread_wa, 1024);

// Private functions
static void getLoopTimes(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile app_configuration *AppConf;
static const volatile mc_configuration *mc_conf;
static volatile VehicleConfiguration miVehiculo;
static volatile VehicleState miEstado;
static volatile VehicleParameters misParametros;
volatile DisplayCommParameters miDisplayComm = {0, false, 0, 0};
static volatile LoopManagerType miLoop;


// Communications with "Mooevo serial display" Thread
DisplayCommParameters * get_display_parameters(void){
  return (DisplayCommParameters *)(&miDisplayComm);
}

// Utility function to convert from erpm to kph, or kph to erpm.
float get_erpm_from_kph(float kmph){
  return (kmph/(3.6*mc_conf->si_wheel_diameter*M_PI)*(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}

float get_kph_from_erpm(float miErpm){
  return (miErpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}

// Utility function to reset motor state
void resetMotor(volatile MotorParam *miMotor){
	miMotor->current = miMotor->erpm = miMotor->erpm_filtered = miMotor->erpm_last = 0;
}

void resetFrenos(VehicleParameters *misParam){
	reset_PID( &(misParam->frenoMaster));
	reset_PID(&(misParam->frenoSlave));
}

void app_custom_configure(app_configuration *conf) {
	AppConf = conf;
	mc_conf = mc_interface_get_configuration();
	/*
	 * INICIALIZACIÓN CONTROL TIEMPOS DE CICLO: miLoop
	 */
	miLoop.hertz = AppConf->app_balance_conf.hertz;
	miLoop.loop_time = US2ST((int)((1000.0/miLoop.hertz) * 1000.0));
	miLoop.current_time = 0;
	miLoop.last_time = 0;
	miLoop.diff_time = 0;
	miLoop.filtered_loop_overshoot = 0;
	miLoop.brake_timeout = 0;
	miLoop.loop_time_filter = AppConf->app_balance_conf.loop_time_filter;
	miLoop.filtered_loop_overshoot = 0;
	miLoop.filtered_diff_time = 0;
	if(miLoop.loop_time_filter > 0){
	    miLoop.loop_overshoot_alpha = 2*M_PI*((float)1/miLoop.hertz)*miLoop.loop_time_filter/(2*M_PI*((float)1/miLoop.hertz)*miLoop.loop_time_filter+1);
	 }
	miLoop.dt = 1.0/miLoop.hertz;
	/*
	 * INICIALIZACIÓN CONFIGURACIÓN DEL VEHÍCULO: miVehiculo
	 */
	miVehiculo.tipoVehiculo  		= AppConf->app_adc_conf.ctrl_type;
	miVehiculo.erpmM_25kph     = get_erpm_from_kph(25);
	miVehiculo.erpmM_andando = get_erpm_from_kph(AppConf->app_balance_conf.kp);
	miVehiculo.erpmM_tortuga   = get_erpm_from_kph(AppConf->app_balance_conf.ki);
	miVehiculo.erpmM_conejo		= get_erpm_from_kph(AppConf->app_balance_conf.kd);
	miVehiculo.erpmM_m_atras 	= get_erpm_from_kph(1.5);
	miVehiculo.max_omega			= AppConf->app_balance_conf.hertz;
	miVehiculo.k_filter					= AppConf->app_balance_conf.loop_time_filter;
	miVehiculo.omega_cut			= AppConf->app_balance_conf.ki_limit;
	miVehiculo.min_curr				= AppConf->app_balance_conf.kd_pt1_highpass_frequency;
	miVehiculo.min_rpm				= AppConf->app_balance_conf.kd_pt1_lowpass_frequency;
	miVehiculo.brake_current		= AppConf->app_balance_conf.brake_current;
	miVehiculo.brake_timeout		= AppConf->app_balance_conf.brake_timeout;
	/*
	 * INICIALIZACIÓN DEL ESTADO LÓGICO DEL VEHÍCULO
	 */
	miEstado.tipoVehiculo = miVehiculo.tipoVehiculo;
	miEstado.modoVehiculo = Sin_limites;
	miEstado.estadoHombreMuerto = HM_FREE;
	miEstado.ms_without_power = 0;
	miEstado.pwr = 0;
	miEstado.reversa = false;
	miEstado.freno = false;
	miEstado.sensorHombreMuerto = false;
	miEstado.max_rpm_conf = miVehiculo.erpmM_25kph;
	miEstado.min_rpm_conf = - miVehiculo.erpmM_25kph;
	/*
	 * INICIALIZACIÓN DE LOS PARÁMETROS DE ESTADO DEL VEHÍCULO
	 */
	misParametros.timeout = 0;
	resetMotor(&misParametros.motorMaster);
	resetMotor(&misParametros.motorSlave);
	init_PID((PID_Type *)&(misParametros.frenoMaster), AWINDUP, DIRECT,
			AppConf->app_balance_conf.kp2,
			AppConf->app_balance_conf.ki2,
			AppConf->app_balance_conf.kd2,
			0.0, miLoop.dt, 0.2, -1.0, 1.0);
	init_PID((PID_Type *)&(misParametros.frenoSlave), AWINDUP, DIRECT,
			AppConf->app_balance_conf.kp2,
			AppConf->app_balance_conf.ki2,
			AppConf->app_balance_conf.kd2,
			0.0, miLoop.dt, 0.2, -1.0, 1.0);
	set_mode_PID((PID_Type *)&misParametros.frenoMaster, AUTO);
	set_mode_PID((PID_Type *)&misParametros.frenoSlave, AUTO);
	misParametros.current_max = 0;
	misParametros.abs_max_rpm = 0;
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {

	stop_now = false;
	chThdCreateStatic(mooevoThread_wa, sizeof(mooevoThread_wa), NORMALPRIO, mooevoThread, NULL);
	commands_printf("Realizado app_custom_START");

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"get_loop",
			"Print the Loop Time, and Sleep Time",
			"",
			getLoopTimes);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(getLoopTimes);
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("Realizado app_custom_STOP");
}

void setLoopVariables(void){
	// set current time, last time, diff time, overshoot compensation, etc.
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
	  miLoop.dt = ST2S(miLoop.diff_time);
}

static void updateExternalVariables(void){
	// For safe start when fault codes occur
	if (mc_interface_get_fault() != FAULT_CODE_NONE && AppConf->app_adc_conf.safe_start != SAFE_START_NO_FAULT) {
		miEstado.ms_without_power = 0;
	}
	// State parameters coming from Display
	miEstado.reversa = miDisplayComm.reversa;

	// State parameters coming from external devices
	// Throttle
	miEstado.pwr = ADC_VOLTS(ADC_IND_EXT);
	// Brake. The HW has a pulldown in this analog input to use a switch
	miEstado.freno = (ADC_VOLTS(ADC_IND_EXT2)>=0.5) ? true : false;
	// Dead Man. The HW as a pulldown in this analog input to use a switch
	miEstado.estadoHombreMuerto = (ADC_VOLTS(ADC_IND_HM)>=0.5) ? true : false;
}
static void updateInternalVariables(void){
	misParametros.motorMaster.erpm_last = misParametros.motorMaster.erpm;
	misParametros.motorSlave.erpm_last = misParametros.motorSlave.erpm;
	misParametros.motorMaster.erpm = mc_interface_get_rpm();
	for (int i=0; i < CAN_STATUS_MSGS_TO_STORE; i++){
		can_status_msg *msg = comm_can_get_status_msg_index(i);
		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE){
			misParametros.motorSlave.erpm = msg->rpm;
		}
	}
    UTILS_LP_MOVING_AVG_APPROX(
    		misParametros.motorMaster.erpm_filtered,
    		misParametros.motorMaster.erpm,
			RPM_FILTER_SAMPLES);
    UTILS_LP_MOVING_AVG_APPROX(
    		misParametros.motorSlave.erpm_filtered,
    		misParametros.motorSlave.erpm,
    		RPM_FILTER_SAMPLES);
    misParametros.rpm_avg_last = misParametros.rpm_avg;
    misParametros.motorMaster.erpm = misParametros.motorMaster.erpm_filtered;
    misParametros.motorSlave.erpm = misParametros.motorSlave.erpm_filtered;
    misParametros.rpm_avg = 0.5*(misParametros.motorMaster.erpm+misParametros.motorSlave.erpm);
    static float omega_filtered = 0.0;
    misParametros.omega = misParametros.rpm_avg - misParametros.rpm_avg_last;
    UTILS_LP_FAST(omega_filtered, misParametros.omega, miVehiculo.k_filter);
    misParametros.omega = omega_filtered;

    misParametros.abs_max_rpm = (fabsf(misParametros.motorMaster.erpm) > fabsf(misParametros.motorSlave.erpm)) ?
    		fabsf(misParametros.motorMaster.erpm) : fabsf(misParametros.motorSlave.erpm);
    miDisplayComm.velocidad = 10*get_kph_from_erpm(misParametros.rpm_avg);

}



static void stateTransition(void){
	if (!miEstado.sensorHombreMuerto) {
		if( (miEstado.tipoVehiculo == Walker_Clean) ||
				(miEstado.tipoVehiculo==Carro_26 && miEstado.modoVehiculo==Andarin)){
			switch(miEstado.estadoHombreMuerto){
			case HM_FREE:
				miEstado.estadoHombreMuerto = HM_BRAKING;
				misParametros.timeout = miLoop.current_time + S2ST(miVehiculo.brake_timeout);
				break;
			case HM_BRAKING:
				if (miLoop.current_time > misParametros.timeout){
					if (misParametros.abs_max_rpm >= miVehiculo.min_rpm){
						miEstado.estadoHombreMuerto = HM_CONTROL_PID;
					} else {
						miEstado.estadoHombreMuerto = HM_STOPPED;
						resetFrenos((VehicleParameters *)&misParametros);
					}
				} else {
					if (misParametros.abs_max_rpm < miVehiculo.min_rpm){
						miEstado.estadoHombreMuerto = HM_STOPPED;
						resetFrenos((VehicleParameters *)&misParametros);
					}
				}
				break;
			case HM_STOPPED:
				if (misParametros.abs_max_rpm > miVehiculo.min_rpm){
					miEstado.estadoHombreMuerto = HM_CONTROL_PID;
				}
				break;
			case HM_CONTROL_PID:
				if (misParametros.abs_max_rpm <  miVehiculo.min_rpm && misParametros.current_max < miVehiculo.min_curr){
					miEstado.estadoHombreMuerto = HM_STOPPED;
					resetFrenos((VehicleParameters *)&misParametros);
				}
				break;
			}
		} else {
			miEstado.estadoHombreMuerto = HM_FREE;
		}
	} else {
		switch(miEstado.tipoVehiculo){
		case Vehiculo_sin_limites:
		case Vehiculo_a_25kph:
			miEstado.modoVehiculo = Sin_limites;
			miDisplayComm.modo = miEstado.tipoVehiculo;
			break;
		case Walker_Clean:
			miEstado.modoVehiculo = Andarin;
			miDisplayComm.modo = miEstado.tipoVehiculo;
			break;
		case Carro_26:
		case Yawer:
			miEstado.modoVehiculo = miDisplayComm.modo;
			break;
		}

		if (miEstado.reversa){
			switch(miEstado.tipoVehiculo){
			case Vehiculo_sin_limites:
			case Vehiculo_a_25kph:
			case Walker_Clean:
				miEstado.max_rpm_conf = 0;
				miEstado.min_rpm_conf = 0;
				break;
			case Carro_26:
			case Yawer:
				if (miEstado.modoVehiculo >= Vehiculo_tortuga){
					miEstado.max_rpm_conf = 0;
					miEstado.min_rpm_conf = - miVehiculo.erpmM_m_atras;
				}
			}
		} else {
			miEstado.min_rpm_conf = 0;
			switch(miEstado.tipoVehiculo){
			case Vehiculo_sin_limites:
				miEstado.max_rpm_conf = 100000;
				break;
			case Vehiculo_a_25kph:
				miEstado.max_rpm_conf = miVehiculo.erpmM_25kph;
				break;
			case Walker_Clean:
				miEstado.max_rpm_conf = miVehiculo.erpmM_andando;
				break;
			case Carro_26:
			case Yawer:
				switch(miEstado.modoVehiculo){
				case Sin_limites:
					miEstado.max_rpm_conf = 0;
					break;
				case Andarin:
					miEstado.max_rpm_conf = miVehiculo.erpmM_andando;
					break;
				case Vehiculo_tortuga:
					miEstado.max_rpm_conf = miVehiculo.erpmM_tortuga;
					break;
				case Vehiculo_conejo:
					miEstado.max_rpm_conf = miVehiculo.erpmM_conejo;
					break;
				}
			}
		}
	}
}

static void driveVehicule(void){

}
static THD_FUNCTION(mooevoThread, arg) {
	(void)arg;
	chRegSetThreadName("APP MOOEVO MONOLITHIC");
	while (!chThdShouldTerminateX()) {
	      timeout_reset();
	      setLoopVariables();
	      timeout_reset();
	      updateExternalVariables();
	      timeout_reset();
	      updateInternalVariables();
	      timeout_reset();
	      stateTransition();
	      timeout_reset();
	      driveVehicule();
	      timeout_reset();
	      chThdSleep(miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
	    }
}

// Callback function for the terminal command with arguments.
static void getLoopTimes(int argc, const char **argv) {
	  (void)argc;
	  (void)argv;
	  float diff = miLoop.diff_time;
	  float sleep_time = miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot);
	  commands_printf("\n%f\t%f", (double)diff, (double)sleep_time);
}
