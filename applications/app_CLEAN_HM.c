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
  Vehicle: Clean, Walkers with HM
  Differences: It only considers Intensity control type with no rev and center brake
  Additional characteristics: Dead Man brake (HM)
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "commands.h"

// Settings ADC remaining
#define MAX_CAN_AGE                     0.1
#define MIN_MS_WITHOUT_POWER            500
#define FILTER_SAMPLES                  5
#define RPM_FILTER_SAMPLES              8

// Settings Specific for this app
#define MIN_RPM                         balance_conf.fault_adc_half_erpm
#define MIN_CURR                        balance_conf.booster_current/10.0

// Threads
static THD_FUNCTION(clean_thread, arg);
static THD_WORKING_AREA(clean_thread_wa, 1024);

// Private variables ADC
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private variables HM
static volatile balance_config balance_conf;
static volatile bool brakeSignal = false;
float dt;
static systime_t current_time, previous_time;

// HM states
typedef enum {
  FREE = 0,
  BRAKING,
  STOPPED,
  BRAKE_FAULT
} BrakeStateType;

typedef enum {
  FORWARD = 0,
  NODIRECTION,
  BACKWARD
} BrakeFaultDirectionType;

//Brake definition includes a PID Controller for stop the cart
typedef struct {
  BrakeStateType state;
  systime_t timeout;
  float erpm_M;
  float erpm_S;
  float erpm_M_last;
  float erpm_S_last;
  float erpm_M_filtered;
  float erpm_S_filtered;
  float abs_max_rpm;
  float p_term_M;
  float p_term_S;
  float i_term_M;
  float i_term_S;
  float d_term_M;
  float d_term_S;
  float d_term_M_filtered;
  float d_term_S_filtered;
  float current_M;
  float current_S;
  float current_max;
} BrakeType;

static BrakeType brake;

// reset PID settings
void reset_brake(void){
  brake.timeout = 0;
  brake.current_M = brake.p_term_M = brake.i_term_M = brake.d_term_M = 0;
  brake.current_S = brake.p_term_S = brake.i_term_S = brake.d_term_S = 0;
  brake.erpm_M_filtered = brake.erpm_S_filtered = 0;
  brake.d_term_M_filtered = brake.d_term_S_filtered = 0;
  brake.abs_max_rpm = 0;
  brake.current_max = 0;
}

// get the
void app_custom_configure(app_configuration *conf) {
    config = conf->app_adc_conf;
    balance_conf = conf->app_balance_conf;
    balance_conf.kp /= 1000.;
    balance_conf.ki /= 1000.;
    balance_conf.kd /= 1000.;
    ms_without_power = 0.0;
    reset_brake();
}

void app_custom_start(void) {
    stop_now = false;
    chThdCreateStatic(clean_thread_wa, sizeof(clean_thread_wa), NORMALPRIO, clean_thread, NULL);
}

void app_custom_stop(void) {
    stop_now = true;
    while (is_running) {
        chThdSleepMilliseconds(1);
    }
    //terminal_unregister_callback(terminal_test);
}


void app_set_brake_current(float current){
  mc_interface_set_brake_current(current);
  if (config.multi_esc) {
    for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
       can_status_msg *msg = comm_can_get_status_msg_index(i);
       if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
         comm_can_set_current_brake(msg->id, current);
       }
    }
  }
}


static THD_FUNCTION(clean_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_SIMPLE_WALKER");

	is_running = true;
	reset_brake();
	brake.state = FREE;
	current_time = previous_time = chVTGetSystemTimeX();
	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);
		// current_time para temporizador del freno
        previous_time = current_time;
        current_time = chVTGetSystemTimeX();
        dt = (float) ST2S(current_time-previous_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT) {
			ms_without_power = 0;
		}

		// Read the external ADC pin voltage
		float pwr = ADC_VOLTS(ADC_IND_EXT);
		read_voltage = pwr;
		static float filter_val = 0.0;
		// filter
		UTILS_LP_MOVING_AVG_APPROX(filter_val, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			pwr = filter_val;
		}
		// Mapping with respect to center voltage
		if (pwr < config.voltage_center) {
			pwr = utils_map(pwr, config.voltage_start,
					config.voltage_center, 0.0, 0.5);
		} else {
			pwr = utils_map(pwr, config.voltage_center,
					config.voltage_end, 0.5, 1.0);
		}

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);
		decoded_level = pwr;
		pwr *= 2.0;
		pwr -= 1.0;
		// throttle 0 -> pwr = -1
		// throttle mid -> pwr = 0
		// Throttle max -> pwr = 1


		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		float current_rel = 0.0;
		bool current_mode_brake = false;

		// Use the filtered and mapped voltage for control according to current norev center brake

		if (pwr >= 0.0) {
			current_rel = pwr;
		} else {
			current_rel = fabsf(pwr);
			current_mode_brake = true;
		}

		if (pwr < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

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
			continue;
		}

		// Reset timeout
		timeout_reset();

        // para frenado leo las rpm de ambas controladoras y saco la media;
        brake.erpm_M_last = brake.erpm_M;
        brake.erpm_S_last = brake.erpm_S;
        brake.erpm_M = mc_interface_get_rpm();
        if(config.multi_esc){
            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);
                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    brake.erpm_S = msg->rpm;
                }
            }
        }
        UTILS_LP_MOVING_AVG_APPROX(brake.erpm_M_filtered, brake.erpm_M, RPM_FILTER_SAMPLES);
        UTILS_LP_MOVING_AVG_APPROX(brake.erpm_S_filtered, brake.erpm_S, RPM_FILTER_SAMPLES);
        brake.erpm_M = brake.erpm_M_filtered;
        brake.erpm_S = brake.erpm_S_filtered;
        brake.abs_max_rpm = (fabsf(brake.erpm_M) > fabsf(brake.erpm_S)) ? fabsf(brake.erpm_M) : fabsf(brake.erpm_S);
		if (current_mode_brake) {
		  switch (brake.state){
		  case FREE:
			brake.state = BRAKING;
			brake.timeout = current_time + S2ST(balance_conf.brake_timeout);
			mc_interface_set_brake_current_rel(current_rel);
			// Send brake command to all ESCs seen recently on the CAN bus
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake_rel(msg->id, current_rel);
					}
				}
			}
			commands_printf("\n\r Current_rel = %f", (double) current_rel);
			// Reset timeout
			timeout_reset();
			break;
		  case BRAKING:
			mc_interface_set_brake_current_rel(current_rel);
			// Send brake command to all ESCs seen recently on the CAN bus
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake_rel(msg->id, current_rel);
					}
				}
			}
			commands_printf("\n\r Current_rel 2 = %f", (double) current_rel);
			if (current_time > brake.timeout) {
			  if (brake.abs_max_rpm > MIN_RPM) {
				brake.state = BRAKE_FAULT;
			  } else {
				app_set_brake_current(0);
				reset_brake();
				brake.state = STOPPED;
			  }
			} else {
			  if (brake.abs_max_rpm < MIN_RPM){
				app_set_brake_current(0);
				reset_brake();
				brake.state = STOPPED;
				commands_printf("\n\r PARAOOO");
			  }
			}
			//commands_printf("\n\r Current_rel 2 = %f", (double) current_rel);
			// Reset timeout
			timeout_reset();
			break;
		  case STOPPED:
			if (brake.abs_max_rpm > MIN_RPM) {
			  brake.state = BRAKE_FAULT;
			  commands_printf("\n\r FALLO FRENO UNA VEZ PARADO");
			}
			break;
		  case BRAKE_FAULT:
			if (brake.abs_max_rpm < MIN_RPM && brake.current_max < MIN_CURR) { //estoy parado y no tengo que hacer casi fuerza
			  reset_brake();
			  brake.state = STOPPED;
			  commands_printf("\n\r PARAOOO222222");
			} else if (brake.abs_max_rpm < MIN_RPM){ // parado, pero en cuesta
			} else { // todav�a no estoy parado
			  commands_printf("\n\r FRENANDO EN FALLO");
			  brake.p_term_M = -balance_conf.kp*brake.erpm_M;
			  brake.p_term_S = -balance_conf.kp*brake.erpm_S;
			  brake.d_term_M =  balance_conf.kd*(brake.erpm_M-brake.erpm_M_last)/dt;
			  brake.d_term_S =  balance_conf.kd*(brake.erpm_S-brake.erpm_S_last)/dt;
			  brake.i_term_M -= balance_conf.ki*brake.erpm_M*dt;
			  brake.i_term_S -= balance_conf.ki*brake.erpm_S*dt;
			  // Filter D
			  UTILS_LP_FAST(brake.d_term_M_filtered, brake.d_term_M, balance_conf.kd_pt1_lowpass_frequency);
			  brake.d_term_M = brake.d_term_M_filtered;
			  UTILS_LP_FAST(brake.d_term_S_filtered, brake.d_term_S, balance_conf.kd_pt1_lowpass_frequency);
			  brake.d_term_S = brake.d_term_S_filtered;

			  float output_M = brake.p_term_M + brake.i_term_M + brake.d_term_M;
			  float output_S = brake.p_term_S + brake.i_term_S + brake.d_term_S;
			  utils_truncate_number_abs(&output_M, 1.0);
			  utils_truncate_number_abs(&output_S, 1.0);

			  //brake.current_M = output_M*balance_conf.brake_current;
			  //brake.current_S = output_S*balance_conf.brake_current;
			  brake.current_M = output_M;
			  brake.current_S = output_S;
			  brake.current_max = (fabsf(brake.current_M) > fabsf(brake.current_S)) ? fabsf(brake.current_M): fabsf(brake.current_S);
			}
			mc_interface_set_current_rel(brake.current_M);
			if (config.multi_esc) {
			  for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				 can_status_msg *msg = comm_can_get_status_msg_index(i);
				 if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				   comm_can_set_current_rel(msg->id, brake.current_S);
				 }
			  }
			}
			// Reset timeout
			timeout_reset();
			break;
		  }
		} else {
			if (brake.state != FREE){
			  brake.state = FREE;
			  reset_brake();
			}
			float current_out = current_rel;
			bool is_reverse = false;
			if (current_out < 0.0) {
				is_reverse = true;
				current_out = -current_out;
				current_rel = -current_rel;
			}

			// Traction control
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						if (config.tc && config.tc_max_diff > 1.0) {
							float rpm_tmp = msg->rpm;
							if (is_reverse) {
								rpm_tmp = -rpm_tmp;
							}
						}

						if (is_reverse) {
							comm_can_set_current_rel(msg->id, -current_out);
						} else {
							comm_can_set_current_rel(msg->id, current_out);
						}
					}
				}
			}
			// Reset timeout
			timeout_reset();

			if (is_reverse) {
				mc_interface_set_current_rel(-current_out);
			} else {
				mc_interface_set_current_rel(current_out);
			}
		}
	}
}
