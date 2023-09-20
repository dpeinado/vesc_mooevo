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

// Threads
static THD_FUNCTION(mooevoThread, arg);
static THD_WORKING_AREA(mooevoThread_wa, 1024);

// Private functions
static void terminal_test(int argc, const char **argv);

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

void resetMotor(volatile MotorParam *miMotor){
	miMotor->current = miMotor->erpm = miMotor->erpm_filtered = miMotor->erpm_last = 0;
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
	miEstado.pwr = 0;
	miEstado.reversa = false;
	miEstado.freno = false;
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
			"custom_cmd",
			"Print the number d",
			"[d]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(terminal_test);
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("Realizado app_custom_STOP");
}

static THD_FUNCTION(mooevoThread, arg) {
	(void)arg;
	chRegSetThreadName("APP MOOEVO MONOLITHIC");
	while (!chThdShouldTerminateX()) {
	      timeout_reset();
	      setLoopVariables();

	      chThdSleep(miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
	    }

}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
