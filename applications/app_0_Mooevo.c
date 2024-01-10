/*
 * app_0_Mooevo.c
 *
 *  Created on: Sep 18, 2023
 *      Author: Diego Peinado Martín
 */

/*!
 *  \brief Módulo que contiene el FW monolítico de Mooevo, a excepción del módulo de cálculo de los
 *  PIDs, y de la definición del HW necesario para las comunicaciones con el Display
 */

#include "app.h"
#include "ch.h"
#include "hal.h"

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

#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TRUE_LEVEL_ADC					1.5
#define ERPM_INFINITE					100000
#define N_RESET_ITERM					200

// Threads
static THD_FUNCTION(mooevoThread, arg);
static THD_WORKING_AREA(mooevoThread_wa, 1024);

// Private functions
static void getLoopTimes(int argc, const char **argv);
static void getInfo1(int argc, const char **argv);
float getPWR(float input_pwr);

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
static volatile mc_fault_code miErrorCode;


/*!
 * \brief Communications with "Mooevo serial display" Thread
 */
DisplayCommParameters * get_display_parameters(void){
  return ((DisplayCommParameters *)(&miDisplayComm));
}

/*!
 * \brief Utility function to convert from erpm to kph..
 */
float get_erpm_from_kph(float kmph){
  return (kmph/(3.6*mc_conf->si_wheel_diameter*M_PI)*(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}
/*!
 * \brief Utility function to convert from kph to erpm.
 */
float get_kph_from_erpm(float miErpm){
  return (miErpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}

/*!
 * \brief Utility function to reset motor state
 */
void resetMotor(volatile MotorParam *miMotor){
	miMotor->current = miMotor->erpm = miMotor->erpm_filtered = miMotor->erpm_last = 0;
}

/*!
 * \brief Utility function to reset brakes state
 */
void resetFrenos(VehicleParameters *misParam){
	reset_PID( &(misParam->frenoMaster));
	reset_PID(&(misParam->frenoSlave));
}

/*!
 * \brief Configuration of the application
 *
 * It gets the pointer to the configuration structure and assigns it to a module variable \c AppConf,
 * and also gets the master motor configuration and assigns it to a module variable \c mc_conf
 *
 * \param app_configuration *conf: pointer to the configuration structure
 */
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
	miVehiculo.tipoVehiculo  = AppConf->app_adc_conf.ctrl_type;
	miVehiculo.erpmM_25kph   = get_erpm_from_kph(25);
	miVehiculo.erpmM_andando = get_erpm_from_kph(AppConf->app_balance_conf.kp);
	miVehiculo.erpmM_tortuga = get_erpm_from_kph(AppConf->app_balance_conf.ki);
	miVehiculo.erpmM_conejo	 = get_erpm_from_kph(AppConf->app_balance_conf.kd);
	miVehiculo.erpmM_m_atras = get_erpm_from_kph(1.5);
	miVehiculo.max_omega	 = AppConf->app_balance_conf.tiltback_constant_erpm;
	miVehiculo.k_filter		 = AppConf->app_balance_conf.tiltback_constant;
	miVehiculo.omega_cut	 = AppConf->app_balance_conf.tiltback_variable* miVehiculo.max_omega;
	miVehiculo.min_curr		 = AppConf->app_balance_conf.kd_pt1_highpass_frequency/10.0;
	miVehiculo.min_rpm		 = AppConf->app_balance_conf.kd_pt1_lowpass_frequency;
	miVehiculo.brake_current = AppConf->app_balance_conf.brake_current;
	miVehiculo.brake_timeout = AppConf->app_balance_conf.brake_timeout;
	/*
	 * INICIALIZACIÓN DEL ESTADO LÓGICO DEL VEHÍCULO
	 */
	miEstado.tipoVehiculo 		= miVehiculo.tipoVehiculo;
	miEstado.modoVehiculo 		= Sin_limites;
	miEstado.estadoHombreMuerto = HM_FREE;
	miEstado.ms_without_power 	= 0;
	miEstado.pwr 				= 0;
	miEstado.decoded_pwr		= 0;
	miEstado.reversa 			= false;
	miEstado.freno 				= false;
	miEstado.sensorHombreMuerto = false;
	miEstado.estoyMuerto		= false;
	miEstado.max_rpm_conf 		= miVehiculo.erpmM_25kph;
	miEstado.min_rpm_conf 		= - miVehiculo.erpmM_25kph;
	/*
	 * INICIALIZACIÓN DE LOS PARÁMETROS DE ESTADO DEL VEHÍCULO
	 */
	misParametros.timeout = 0;
	resetMotor(&misParametros.motorMaster);
	resetMotor(&misParametros.motorSlave);
	init_PID((PID_Type *)&(misParametros.frenoMaster), AWINDUP, DIRECT,
			AppConf->app_balance_conf.kp2/1000.0,
			AppConf->app_balance_conf.ki2/1000.0,
			AppConf->app_balance_conf.kd2/1000.0,
			0.0, miLoop.dt, 0.2, -1.0, 1.0);
	init_PID((PID_Type *)&(misParametros.frenoSlave), AWINDUP, DIRECT,
			AppConf->app_balance_conf.kp2/1000.0,
			AppConf->app_balance_conf.ki2/1000.0,
			AppConf->app_balance_conf.kd2/1000.0,
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
	terminal_register_command_callback(
			"get_info1",
			"Print configuration",
			"",
			getInfo1);
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
	miErrorCode = mc_interface_get_fault();
	if (miErrorCode != FAULT_CODE_NONE){
		commands_printf("\nERROR: %s", mc_interface_fault_to_string(miErrorCode));
	}
	// State parameters coming from Display
	miEstado.reversa = miDisplayComm.reversa;

	// State parameters coming from external devices
	// Throttle
	miEstado.pwr = ADC_VOLTS(ADC_IND_EXT);
	// Brake. The HW has a pulldown in this analog input to use a switch
	miEstado.freno = (ADC_VOLTS(ADC_IND_EXT2)>=TRUE_LEVEL_ADC) ? true : false;
	// Dead Man. The HW as a pulldown in this analog input to use a switch
	miEstado.sensorHombreMuerto = (ADC_VOLTS(ADC_IND_HM)>=TRUE_LEVEL_ADC) ? true : false;
	//miEstado.sensorHombreMuerto = ADC_VOLTS(ADC_IND_HM);
	miEstado.modoVehiculo = miDisplayComm.modo;
	miEstado.decoded_pwr = getPWR(miEstado.pwr);

	if( (miEstado.tipoVehiculo == Walker_Clean) ||
					(miEstado.tipoVehiculo==Carro_26 && miEstado.modoVehiculo==Andarin)){
		/*if (miEstado.decoded_pwr>0){
			commands_printf("\nTipo Vehiculo correcto, pwr = %.2f \t decoded_pwr = %.2f",
					(double) miEstado.pwr,
					(double) miEstado.decoded_pwr);
			commands_printf("\nVol_start = %.2f\t Vol_center = %.2f\t Vol_end = %.2f",
					(double) AppConf->app_adc_conf.voltage_start,
					(double) AppConf->app_adc_conf.voltage_center,
					(double) AppConf->app_adc_conf.voltage_end);
		}*/
		if (miEstado.decoded_pwr < 0.0) {
			miEstado.estoyMuerto = true;
			if (miEstado.sensorHombreMuerto) {
				miEstado.estoyMuerto = false;
				miEstado.decoded_pwr = 0.0;
			}
		} else {
			miEstado.estoyMuerto = false;
		}
	}
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
	if (miEstado.estoyMuerto) {
		if( (miEstado.tipoVehiculo == Walker_Clean) ||
				(miEstado.tipoVehiculo==Carro_26 && miEstado.modoVehiculo==Andarin)){
			switch(miEstado.estadoHombreMuerto){
			case HM_FREE:
				miEstado.estadoHombreMuerto = HM_BRAKING;
				misParametros.timeout = miLoop.current_time + S2ST(miVehiculo.brake_timeout);
				misParametros.motorMaster.current = 0;
				misParametros.motorSlave.current = 0;
				break;
			case HM_BRAKING:
				if (miLoop.current_time > misParametros.timeout){
					if (misParametros.abs_max_rpm >= miVehiculo.min_rpm){
						miEstado.estadoHombreMuerto = HM_CONTROL_PID;
						resetFrenos((VehicleParameters *)&misParametros);
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
					resetFrenos((VehicleParameters *)&misParametros);
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
			//resetFrenos((VehicleParameters *)&misParametros);
		}
	} else {
		if (miEstado.estadoHombreMuerto) {
			miEstado.estadoHombreMuerto = HM_FREE;
			resetFrenos((VehicleParameters *)&misParametros);
			miEstado.ms_without_power = 0;
			commands_printf("ME RESETEEEOOOO");
		}
	}
	if (miEstado.reversa){
		miEstado.max_rpm_conf = 0;
		switch(miEstado.tipoVehiculo){
		case Vehiculo_sin_limites:
		case Vehiculo_a_25kph:
		case Walker_Clean:
			miEstado.min_rpm_conf = 0;
			break;
		case Carro_26:
		case Yawer:
			if (miEstado.modoVehiculo >= Vehiculo_tortuga){
				miEstado.min_rpm_conf = - miVehiculo.erpmM_m_atras;
			}
		}
	} else {
		miEstado.min_rpm_conf = 0;
		switch(miEstado.tipoVehiculo){
		case Vehiculo_sin_limites:
			miEstado.max_rpm_conf = ERPM_INFINITE;
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

float getPWR(float input_pwr){
	float pwr = input_pwr;
	static float filter_val = 0;
	if (miEstado.freno) {
		return (0);
	}
	//if (pwr >0) commands_printf("\nPWR 1 = %f", (double) pwr);
	// se promedian los 5 últimos valores (aproximadamente)
	UTILS_LP_MOVING_AVG_APPROX(filter_val, pwr, FILTER_SAMPLES);
	if (AppConf->app_adc_conf.use_filter) {
		pwr = filter_val;
	}
	// Mapear respecto del centro del acelerador
    if (pwr < AppConf->app_adc_conf.voltage_center) {
        pwr = utils_map(pwr, AppConf->app_adc_conf.voltage_start,
        		AppConf->app_adc_conf.voltage_center, 0.0, 0.5);
    } else {
        pwr = utils_map(pwr, AppConf->app_adc_conf.voltage_center,
        		AppConf->app_adc_conf.voltage_end, 0.5, 1.0);
    }
	utils_truncate_number(&pwr, 0.0, 1.0);

	if (AppConf->app_adc_conf.voltage_inverted) {
		pwr = 1.0 - pwr;
	}
	// si modo reversa, se invierte la salida
	if (miEstado.reversa && miEstado.min_rpm_conf) {
	  pwr = -pwr;
	  palSetPad(HW_ICU_GPIO, HW_ICU_PIN); // salida alarma marcha atrás
	} else {
	  palClearPad(HW_ICU_GPIO, HW_ICU_PIN); // apaga alarma marcha atrás
	}
	// TODO AQUI HAY QUE DISCRIMINAR QUIÉN TIENE HOMBRE MUERTO Y *=2 y -=1
	if( ( (miEstado.tipoVehiculo == Walker_Clean) ||
						(miEstado.tipoVehiculo==Carro_26 && miEstado.modoVehiculo==Andarin)) &&
								!miEstado.sensorHombreMuerto) {
		// Escalar el voltaje y poner posición central en 0
		pwr *=2;
		pwr -=1;
	}
	// se aplican la banda muerta y la curva de aceleraci�n/deceleraci�n
	utils_deadband(&pwr, AppConf->app_adc_conf.hyst, 1.0);
	pwr = utils_throttle_curve(pwr,
			AppConf->app_adc_conf.throttle_exp,
			AppConf->app_adc_conf.throttle_exp_brake,
			AppConf->app_adc_conf.throttle_exp_mode);
	timeout_reset();

	static float pwr_ramp = 0.0;
	float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? AppConf->app_adc_conf.ramp_time_pos : AppConf->app_adc_conf.ramp_time_neg;

	if (ramp_time > 0.01) {
		const float ramp_step = (float)ST2MS(miLoop.diff_time) / (ramp_time * 1000.0);
		utils_step_towards(&pwr_ramp, pwr, ramp_step);
		pwr = pwr_ramp;
	}

	/*************************************************************/
    // Algoritmo para un arranque seguro
	if (fabsf(pwr) < 0.001) {
        miEstado.ms_without_power += (1000.0 * (float)miLoop.diff_time) / (float)CH_CFG_ST_FREQUENCY;
    }
	timeout_reset();
    // If safe start is enabled and the output has not been zero for long enough
    /*if (miEstado.ms_without_power < MIN_MS_WITHOUT_POWER && AppConf->app_adc_conf.safe_start) {
    	static int pulses_without_power_before = 0;
    	if (miEstado.ms_without_power == pulses_without_power_before) {
    		miEstado.ms_without_power = 0;
    	}
        pulses_without_power_before = miEstado.ms_without_power;
        pwr = 0;
    }*/
    if (miEstado.tipoVehiculo == Yawer && !miEstado.sensorHombreMuerto) {
    	pwr = 0;
    }
	return (pwr);
}

void set_curr_rel_both_brakes(float rel_curr){
	mc_interface_set_brake_current_rel(rel_curr);
	for (int i=0; i < CAN_STATUS_MSGS_TO_STORE; i++){
		can_status_msg *msg = comm_can_get_status_msg_index(i);
		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE){
			comm_can_set_current_brake_rel(msg->id,  rel_curr);
		}
	}
}

void set_curr_rel_both_motors(float rel_curr1, float rel_curr2){
	mc_interface_set_current_rel(rel_curr1);
	for (int i=0; i < CAN_STATUS_MSGS_TO_STORE; i++){
		can_status_msg *msg = comm_can_get_status_msg_index(i);
		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE){
			comm_can_set_current_rel(msg->id,  rel_curr2);
		}
	}
}

static void driveVehicule(void){
	switch(miEstado.estadoHombreMuerto){
		case HM_FREE: {
			float miPwr = miEstado.decoded_pwr;
			float lo_max_rpm = 0.0;
			float lo_min_rpm = 0.0;
			// aplico los máximos a la corriente de salida en función de los límites de velocidad del modo seleccionado
			if (misParametros.rpm_avg >= 0) {
			  const float rpm_pos_cut_start = miEstado.max_rpm_conf * mc_conf->l_erpm_start;
			  const float rpm_pos_cut_end = miEstado.max_rpm_conf;
			  if (misParametros.rpm_avg < (rpm_pos_cut_start + 0.1)) {
				  lo_max_rpm = miPwr;
			  } else if (misParametros.rpm_avg > (rpm_pos_cut_end - 0.1)) {
				  lo_max_rpm = 0.0;
			  } else {
				  lo_max_rpm = utils_map(misParametros.rpm_avg, rpm_pos_cut_start, rpm_pos_cut_end, miPwr, 0.0);
			  }
			  miPwr = lo_max_rpm;
			} else {
			  const float rpm_neg_cut_start = miEstado.min_rpm_conf * mc_conf->l_erpm_start;
			  const float rpm_neg_cut_end = miEstado.min_rpm_conf;
			  if (misParametros.rpm_avg > (rpm_neg_cut_start - 0.1)) {
				  lo_min_rpm = miPwr;
			  } else if (misParametros.rpm_avg < (rpm_neg_cut_end + 0.1)) {
				  lo_min_rpm = 0.0;
			  } else {
				  lo_min_rpm = utils_map(misParametros.rpm_avg, rpm_neg_cut_start, rpm_neg_cut_end, miPwr, 0.0);
			  }
			  miPwr = lo_min_rpm;
			}

			// controlo que la aceleración no supere un valor máximo
		   if (misParametros.omega >= 0){
			 if (misParametros.omega < (miVehiculo.omega_cut)){
			   lo_max_rpm = miPwr;
			 } else if (misParametros.omega > miVehiculo.max_omega){
			   lo_max_rpm = 0.0;
			 } else {
			   lo_max_rpm = utils_map(misParametros.omega, miVehiculo.omega_cut, miVehiculo.max_omega, miPwr, 0.0);
			 }
			 miPwr = lo_max_rpm;
		   } // En principio no hay límite a la máxima deceleración
		   misParametros.motorMaster.current = miPwr;
		   misParametros.motorSlave.current = miPwr;
		   misParametros.current_max = miPwr;
		   miDisplayComm.intensidad = 10*miPwr*mc_conf->lo_current_motor_max_now;
		   set_curr_rel_both_motors(misParametros.motorMaster.current, misParametros.motorSlave.current);
		   break;
		}
		case HM_BRAKING:
			set_curr_rel_both_brakes(miVehiculo.brake_current );
			timeout_reset();
			break;
		case HM_STOPPED:
			break;
		case HM_CONTROL_PID:
			set_curr_rel_both_brakes(0 );
			misParametros.motorMaster.current = compute_PID(
					(PID_Type *)&(misParametros.frenoMaster),
					misParametros.motorMaster.erpm,
					miLoop.dt);
			misParametros.motorSlave.current = compute_PID(
					(PID_Type *)&(misParametros.frenoSlave),
								misParametros.motorSlave.erpm,
								miLoop.dt);
			misParametros.current_max = ( fabsf(misParametros.motorMaster.current)> \
					fabsf(misParametros.motorSlave.current)) ? \
							fabsf(misParametros.motorMaster.current) : fabsf(misParametros.motorSlave.current);

			static uint contadorM = 0;
			static uint contadorS = 0;
			if ( (misParametros.motorMaster.erpm < -miVehiculo.min_rpm && misParametros.frenoMaster.i_term < 0) ||  (misParametros.motorMaster.erpm > miVehiculo.min_rpm && misParametros.frenoMaster.i_term > 0) ) {
				contadorM++;
				if (contadorM >= N_RESET_ITERM){
					contadorM = 0;
					misParametros.frenoMaster.i_term = 0;
					misParametros.motorMaster.current = 0;
				}
			} else contadorM = 0;

			if ( (misParametros.motorSlave.erpm < -miVehiculo.min_rpm && misParametros.frenoSlave.i_term < 0) ||  (misParametros.motorSlave.erpm > miVehiculo.min_rpm && misParametros.frenoSlave.i_term > 0) ) {
				contadorS++;
				if (contadorS >=N_RESET_ITERM){
					misParametros.frenoSlave.i_term = 0;
					misParametros.motorSlave.current = 0;
				}
			} else contadorS = 0;

			set_curr_rel_both_motors(misParametros.motorMaster.current, misParametros.motorSlave.current);
			break;
	}
}

static THD_FUNCTION(mooevoThread, arg) {
	(void)arg;
	chRegSetThreadName("Mooevo Monolithic");
	// Config output for reverse alarm as an output
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(HW_ICU_GPIO, HW_ICU_PIN); // apaga alarma marcha atrás
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

static void getInfo1(int argc, const char **argv) {
	  (void)argc;
	  (void)argv;
	  char mensaje1[100];
	  char mensaje2[100];
	  uint miTipoV = miEstado.tipoVehiculo;
	  uint miModoV = miEstado.modoVehiculo;
	  switch (miTipoV) {
	  case Vehiculo_sin_limites:
		  sprintf(mensaje1, "%s", "Vehiculo sin Límites");
		  break;
	  case Vehiculo_a_25kph:
		  sprintf(mensaje1 , "%s",  "Vehículo a 25 kpm");
		  break;
	  case Walker_Clean:
		  sprintf(mensaje1 , "%s",  "Walker / Clean");
		  break;
	  case Carro_26:
		  sprintf(mensaje1 , "%s",  "Carro 26");
		  break;
	  case Yawer:
		  sprintf(mensaje1 , "%s",  "Yawer");
		  break;
	  }
	  switch(miModoV){
	  case Sin_limites:
		  sprintf(mensaje2 , "%s",  "Sin limites");
		  break;
	  case Andarin:
		  sprintf(mensaje2 , "%s",  "Andarín");
		  break;
	  case Vehiculo_tortuga:
		  sprintf(mensaje2 , "%s",  "Tortuga");
		  break;
	  case Vehiculo_conejo:
		  sprintf(mensaje2 , "%s",  "Conejo");
		  break;
	  }
	  commands_printf("\nTipo vehículo = %s\t Modo Vehículo = %s", mensaje1, mensaje2);
	  float mswp = miEstado.ms_without_power;
	  float miPwr = miEstado.decoded_pwr;
	  bool mireversa = miEstado.reversa;
	  bool mifreno = miEstado.freno;
	  bool miSHM = miEstado.sensorHombreMuerto;
	  float miMRC = miEstado.max_rpm_conf;
	  float mimRC = miEstado.min_rpm_conf;
	  uint estadoHM = miEstado.estadoHombreMuerto;
	  switch(estadoHM){
	  case HM_FREE:
		  sprintf(mensaje1 , "%s",  "Free");
		  break;
	  case HM_BRAKING:
		  sprintf(mensaje1 , "%s",   "Braking");
		  break;
	  case HM_STOPPED:
		  sprintf(mensaje1 , "%s",   "Stopped");
		  break;
	  case HM_CONTROL_PID:
		  sprintf(mensaje1 , "%s",  "Control PID");
		  break;
	  }
	  commands_printf("\nHM: %s\t Milliseconds without power: %f", mensaje1, (double) mswp);
	  commands_printf("\nMSWP = %s\tPWR = %f\tRev: %u\tFreno: %u\tSHM: %u", mensaje1, (double)miPwr, mireversa, mifreno, miSHM);
	  commands_printf("\nMaxRpm: %f\t MinRpm: %f", (double)miMRC, (double)mimRC);


	  commands_printf("\nCurrent: %f\t max_omega: %f\t omega_cut: %f",
			  (double)misParametros.current_max,
			  (double)miVehiculo.max_omega,
			  (double)miVehiculo.omega_cut);
	  commands_printf("\n MINCurrent: %f\tMINRpm: %f\tbrake_current: %f\tbrake_timeout: %f",
			  (double) miVehiculo.min_curr,
			  (double) miVehiculo.min_rpm,
			  (double) miVehiculo.brake_current,
			  (double) miVehiculo.brake_timeout);
	  commands_printf("\nvoltage_start: %f\tvoltage_end: %f",
			  (double)  AppConf->app_adc_conf.voltage_start,
			  (double) AppConf->app_adc_conf.voltage_end);

	  commands_printf("\n*********************************   MASTER   ***********************************************");
	  commands_printf("\nkp = %f\tki = %f\tkd = %f",
			  (double) misParametros.frenoMaster.kp,
			  (double) misParametros.frenoMaster.ki,
			  (double) misParametros.frenoMaster.kd);

	  commands_printf("\nP = %f\tkI = %f\tkD = %f",
			  (double) misParametros.frenoMaster.p_term,
			  (double) misParametros.frenoMaster.i_term,
			  (double) misParametros.frenoMaster.d_term);

	  commands_printf("\n*********************************   SLAVE   ***********************************************");
	  commands_printf("\nkp = %f\tki = %f\tkd = %f",
			  (double) misParametros.frenoSlave.kp,
			  (double) misParametros.frenoSlave.ki,
			  (double) misParametros.frenoSlave.kd);

	  commands_printf("\nP = %f\tkI = %f\tkD = %f",
			  (double) misParametros.frenoSlave.p_term,
			  (double) misParametros.frenoSlave.i_term,
			  (double) misParametros.frenoSlave.d_term);
}
