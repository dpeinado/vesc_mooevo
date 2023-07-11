/*
 * mooevo_pid.c
 *
 *  Created on: 29 sept. 2022
 *      Author: Diego Peinado Mart�n
 */


#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "utils.h"

#include "mooevo_pid.h"


void set_mode_PID(PID_Type *mpid, bool modo){
  mpid->modo = modo;
}

void reset_PID(PID_Type *mpid){
  mpid->p_term = mpid->i_term = mpid->d_term = mpid->d_filtered = 0.0;
}


void reset_Iterm(PID_Type *mpid){
  mpid->i_term = 0.0;
  mpid->last_input = 0.0;
  mpid->output = 0.0;
}
void set_PID_direction(PID_Type *mpid, DireccionControlador mdireccion){
  if (mpid->direccion != mdireccion){
    mpid->kp *= (-1);
    mpid->ki *= (-1);
    mpid->kd *= (-1);
    mpid->direccion = mdireccion;
  }
}

void set_PID_setPoint(PID_Type *mpid, float mset_point){
  mpid->set_point = mset_point;
}

void init_PID(PID_Type *mpid, TipoPID mtipo, DireccionControlador mdireccion,
              float mkp, float mki, float mkd,
              float mset_point, float mdt, float mkd_filter,
              float mmin_value, float mmax_value){
  mpid->tipo = mtipo;
  mpid->direccion = mdireccion;
  mpid->modo = MANUAL;
  mpid->kp = mkp;
  mpid->ki = mki;
  mpid->kd = mkd;
  mpid->p_term = mpid->i_term = mpid->d_term = mpid->d_filtered = 0.0;
  mpid->set_point = mset_point;
  mpid->dt = mdt;
  mpid->kd_filter = mkd_filter;
  mpid->min_value = mmin_value;
  mpid->max_value = mmax_value;
  mpid->output = 0.0;
  if (mpid->direccion == REVERSE) {
    mpid->kp *= (-1);
    mpid->ki *= (-1);
    mpid->kd *= (-1);
  }
}


float compute_PID_antiWindup(PID_Type *mpid, float input) {
  float error = mpid->set_point - input;
  float dInput = input - mpid->last_input;
  mpid->p_term = error*mpid->kp*0.050;
  mpid->i_term += error*mpid->ki*mpid->dt*0.050;
  mpid->d_term = - dInput*mpid->kd/mpid->dt*0.050; // Derivative on Measurement
  UTILS_LP_FAST(mpid->d_filtered, mpid->d_term, mpid->kd_filter); // filter D
  mpid->d_term = mpid->d_filtered;
  utils_truncate_number(&mpid->i_term, -1.0, 1.0); // I wind-up protection
  mpid->last_input = input;
  float output = mpid->p_term+mpid->i_term+mpid->d_term;
  utils_truncate_number(&output, -1.0, 1.0);
  return output;
}

float compute_PID_antiSaturation(PID_Type *mpid, float input){
  float error = mpid->set_point - input;
  float dInput = input - mpid->last_input;
  mpid->p_term = error*mpid->kp*0.050;
  mpid->d_term = -dInput*mpid->kd/mpid->dt*0.050;
  UTILS_LP_FAST(mpid->d_filtered, mpid->d_term, mpid->kd_filter);
  mpid->d_term = mpid->d_filtered;
  mpid->last_input = input;
  utils_truncate_number_abs(&mpid->p_term, 1.0);
  utils_truncate_number_abs(&mpid->d_term, 1.0);
  float output = mpid->p_term+mpid->i_term+mpid->d_term;
  float pre_output = output;
  utils_truncate_number_abs(&output, 1.0);
  float output_saturation = output - pre_output;
  mpid->i_term += error*mpid->ki*mpid->dt*0.050+output_saturation;
  return output;
}


float compute_PID(PID_Type *mpid, float input, float dt){
  if (!mpid->modo) return 0.0;
  if (isnan(dt)) dt = 0.001;
  mpid->dt = dt;
  float output;
  if(mpid->tipo == AWINDUP)
    output=compute_PID_antiWindup(mpid, input);
  else
    output=compute_PID_antiSaturation(mpid, input);
  output = (output+1.0)/2.0*(mpid->max_value-mpid->min_value)+mpid->min_value;
  mpid->output = output;
  return output;
}
