/*
 	Copyright 2023 Diego Peinado dpeinad@gmail.com
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  App derived from app_adc.c
  Vehicle: URBAN, Correos, duales.
  Differences: It only considers Intensity control type with no rev and center brake
  Additional characteristics:
		Reverse with limited velocity
		Two forward limits for max velocity
		Display ebike for interaction
  It has to be used with new hardware: 60_mooevo
 */

//#include "hwconf/mooevo/hw_60_mooevo.h" // only for eclipse
#include "app_URBAN_DISPLAY.h"

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "commands.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

#include "mooevo_pid.h"


// Settings ADC
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf

// Settings Espec�ficos

// Threads
static THD_FUNCTION(urban_thread, arg);
static THD_WORKING_AREA(urban_thread_wa, 1024);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float read_voltage = 0.0;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private variables espec�ficas

static const volatile mc_configuration *mc_conf;
mc_configuration *mc_conf_mod;


urban_config uconf;
tipo_estado_vehiculo miEstado = {1, false, 0, 0};


tipo_estado_vehiculo * get_estado_vehiculo(void){
  return &miEstado;
}

float get_erpm_from_kph(float kmph){
  return kmph/(3.6*mc_conf->si_wheel_diameter*M_PI)*(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio);
}

float get_kph_from_erpm(float miErpm){
  return miErpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio);
}

void app_custom_configure(app_configuration *conf) {
	config = conf->app_adc_conf;
	mc_conf = mc_interface_get_configuration();
	uconf.erpmM_marchaAtras = get_erpm_from_kph(conf->app_balance_conf.kp);
	uconf.erpmM_andando     = get_erpm_from_kph(conf->app_balance_conf.ki);
	uconf.erpmM_vehiculo    = get_erpm_from_kph(conf->app_balance_conf.kd);
	uconf.vel_proporcion    = conf->app_balance_conf.kp2;
	uconf.max_omega         = conf->app_balance_conf.ki2;
	uconf.k_filter          = conf->app_balance_conf.kd2;
	uconf.omega_cut         = uconf.max_omega*conf->app_balance_conf.ki_limit;
	ms_without_power = 0.0;
	commands_printf("Marcha Atras: %f", (double) uconf.erpmM_marchaAtras);
	commands_printf("Andando: %f", (double) uconf.erpmM_andando);
	commands_printf("Veh�culo: %f", (double) uconf.erpmM_vehiculo);
}

void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(urban_thread_wa, sizeof(urban_thread_wa), NORMALPRIO, urban_thread, NULL);
	commands_printf("Realizado app_custom_START");
}

void app_custom_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
	//terminal_unregister_callback(terminal_test);
	commands_printf("Realizado app_custom_STOP");
}

static THD_FUNCTION(urban_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_URBAN");

	volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();
	is_running = true;

	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	for(;;) {
	    // control de la frecuencia de repetici�n de la tarea
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
        if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT) {
            ms_without_power = 0;
        }

		timeout_reset();

		// lectura del voltaje del acelerador
		float pwr = ADC_VOLTS(ADC_IND_EXT);
		// lectura del sensor de freno. Tengo un pull down en la PCB para poder leer entrada digital en esta analógica
		float brk = ADC_VOLTS(ADC_IND_EXT2);
		read_voltage = pwr;
		static float filter_val = 0.0;
		if (brk >= 0.5) {
		  pwr = 0;
		  read_voltage = 0;
		  //commands_printf("BRK: %f", brk);
		}

		// se promedian los 5 �ltimos valores (aproximadamente)
		UTILS_LP_MOVING_AVG_APPROX(filter_val, pwr, FILTER_SAMPLES);
		if (config.use_filter) {
			pwr = filter_val;
		}

		// se mapea la entrada desde voltios al intervalo 0 - 1
		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
		utils_truncate_number(&pwr, 0.0, 1.0);
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}

        // si modo reversa, se invierte la salida
        if (miEstado.reversa && miEstado.modo > 1) {
          pwr = -pwr;
          palSetPad(HW_ICU_GPIO, HW_ICU_PIN); // salida alarma marcha atrás
        } else {
          palClearPad(HW_ICU_GPIO, HW_ICU_PIN); // apaga alarma marcha atrás
        }

        // se aplican la banda muerta y la curva de aceleraci�n/deceleraci�n
		utils_deadband(&pwr, config.hyst, 1.0);
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		timeout_reset();

        // Se aplica rampa en el valor de la salida. Esto no es bueno para el control, por lo que se reducir� al m�nimo
        static systime_t last_time = 0;
        static float pwr_ramp = 0.0;
        float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

        if (ramp_time > 0.01) {
            const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
            utils_step_towards(&pwr_ramp, pwr, ramp_step);
            last_time = chVTGetSystemTimeX();
            pwr = pwr_ramp;
        }

        // Algoritmo para un arranque seguro
		float current_rel = 0.0;
		current_rel = pwr;
		if (fabsf(pwr) < 0.001) {
            ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
        }
		timeout_reset();
        // If safe start is enabled and the output has not been zero for long enough
        if (ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) {
            static int pulses_without_power_before = 0;
            if (ms_without_power == pulses_without_power_before) {
                ms_without_power = 0;
            }
            pulses_without_power_before = ms_without_power;
            mc_interface_set_brake_current(timeout_get_brake_current());

            if (config.multi_esc) {
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current_brake(msg->id, timeout_get_brake_current());
                    }
                }
            }
        }

        // Find lowest RPM. En realidad tengo las rpm locales y en lowest tendr� las rpm del otro controlador
        float rpm_local = mc_interface_get_rpm();
        float rpm_lowest = rpm_local;
        float rpm_avg = 0.0;
        static float rpm_avg_prev = 0.0;

        if (config.multi_esc) {
            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);
                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                  rpm_lowest = msg->rpm;
                }
            }
        }
        rpm_avg = (rpm_local+rpm_lowest)/2.0;

        miEstado.velocidad = 10*get_kph_from_erpm(rpm_avg);
        float lo_max_rpm = 0.0;
        float lo_min_rpm = 0.0;
        float max_rpm_conf = 0.0;
        float min_rpm_conf = 0.0;

        // Se calculan los valores m�nimos (marcha atr�s) y m�ximos (marcha alante) de rpms en funci�n del modo elegido
        if (miEstado.reversa) {
          if (miEstado.modo>1){ // marcha atr�s y modo veh�culo
            max_rpm_conf = 0.0;
            min_rpm_conf = - uconf.erpmM_marchaAtras;
          } else { // marcha atr�s y modo andar�n
            max_rpm_conf = 0.0;
            min_rpm_conf = 0.0;
          }
        } else{
          if (miEstado.modo>=3){ // marcha adelante y modo veh�culo
            max_rpm_conf = uconf.erpmM_vehiculo;
            min_rpm_conf = 0.0;
          } else if (miEstado.modo == 2) { // marcha adelante y modo andar�n
            max_rpm_conf = uconf.erpmM_vehiculo*uconf.vel_proporcion;
            min_rpm_conf = 0.0;
          } else if (miEstado.modo == 1) {
            max_rpm_conf = uconf.erpmM_andando;
            min_rpm_conf = 0.0;
          } else {
            max_rpm_conf = 0.0;
            min_rpm_conf = 0.0;
          }
        }
        // aplico los m�ximos a la corriente de salida en funci�n del modo seleccionado
        if (rpm_avg >= 0) {
          const float rpm_pos_cut_start = max_rpm_conf * mc_conf->l_erpm_start;
          const float rpm_pos_cut_end = max_rpm_conf;
          if (rpm_avg < (rpm_pos_cut_start + 0.1)) {
              lo_max_rpm = current_rel;
          } else if (rpm_avg > (rpm_pos_cut_end - 0.1)) {
              lo_max_rpm = 0.0;
          } else {
              lo_max_rpm = utils_map(rpm_avg, rpm_pos_cut_start, rpm_pos_cut_end, current_rel, 0.0);
          }
          current_rel = lo_max_rpm;
        } else {
          const float rpm_neg_cut_start = min_rpm_conf * mc_conf->l_erpm_start;
          const float rpm_neg_cut_end = min_rpm_conf;
          if (rpm_avg > (rpm_neg_cut_start - 0.1)) {
              lo_min_rpm = current_rel;
          } else if (rpm_avg < (rpm_neg_cut_end + 0.1)) {
              lo_min_rpm = 0.0;
          } else {
              lo_min_rpm = utils_map(rpm_avg, rpm_neg_cut_start, rpm_neg_cut_end, current_rel, 0.0);
          }
          current_rel = lo_min_rpm;
        }

        // controlo que la aceleraci�n no supere un valor m�ximo almacenado en balace_conf.noseangle_speed
        float omega = 0.0;
        static float omega_filtered = 0.0;
        omega = rpm_avg - rpm_avg_prev;
        UTILS_LP_FAST(omega_filtered, omega, uconf.k_filter);
        omega = omega_filtered;
        rpm_avg_prev = rpm_avg;

        if (omega >= 0){
          if (omega < (uconf.omega_cut)){
            lo_max_rpm = current_rel;
          } else if (omega > uconf.max_omega){
            lo_max_rpm = 0.0;
          } else {
            lo_max_rpm = utils_map(omega, uconf.omega_cut, uconf.max_omega, current_rel, 0.0);
          }
          //commands_printf("rpm = %f: omega = %f: omega_cut = %f: intensity_cut = %f: intensity_raw = %f", (double) rpm_avg, (double) omega, (double) uconf.omega_cut,
          //                (double) lo_max_rpm, (double) current_rel);
          current_rel = lo_max_rpm;
        } // En principio no hay l�mite a la m�xima deceleraci�n

        for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
            can_status_msg *msg = comm_can_get_status_msg_index(i);
            if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
              comm_can_set_current_rel(msg->id, current_rel);
            }
        }
        miEstado.intensidad = 10*current_rel*mcconf->lo_current_motor_max_now;
        mc_interface_set_current_rel(current_rel);
	}
}
