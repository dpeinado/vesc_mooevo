/*
 * mooevo_pid.h
 *
 *  Created on: 29 sept. 2022
 *      Author: Diego Peinado Martín
 */

#ifndef APPLICATIONS_MOOEVO_PID_H_
#define APPLICATIONS_MOOEVO_PID_H_

typedef enum {
  MANUAL,
  AUTO
} TipoModo;

typedef enum {
  AWINDUP,
  SATURATION
} TipoPID;

typedef enum {
  DIRECT,
  REVERSE
} DireccionControlador;


typedef struct {
  TipoPID tipo; // controla qué algoritmo pid se ejecuta
  DireccionControlador direccion; // hay controladores directos: más señal de error, más señal de control, o inversos
  bool modo;
  float kp, ki, kd;
  float p_term, i_term, d_term;
  float last_input;
  float set_point;
  float dt;
  float kd_filter;
  float d_filtered;
  float min_value;
  float max_value;
  float output;
} PID_Type;


typedef struct {
  TipoPID tipo; // controla qué algoritmo pid se ejecuta
  DireccionControlador direccion; // hay controladores directos: más señal de error, más señal de control, o inversos
  bool modo;
  float kp1, kd1, kp2, ki2;
  float p_term1, d_term1, p_term2, i_term2;
  float last_input1;
  float last_input2;
  float set_point1;
  float set_point2;
  float dt;
  float kd_filter;
  float d_filtered;
  float min_value;
  float max_value;
  float output;
} PDPI_Type;



void init_PID(PID_Type *mpid, TipoPID mtipo, DireccionControlador mdireccion,
              float mkp, float mki, float mkd,
              float mset_point, float mdt, float mkd_filter,
              float mmin_value, float mmax_value);
float compute_PID(PID_Type *mpid, float input, float dt);
void set_mode_PID(PID_Type *mpid, bool modo);
void reset_PID(PID_Type *mpid);
void set_PID_setPoint(PID_Type *mpid, float mset_point);


void init_PDPI(PDPI_Type *mpid, TipoPID mtipo, DireccionControlador mdireccion,
               float mkp1, float mkd1, float mkp2, float mki2,
              float mset_point1, float mset_point2, float mdt, float mkd_filter,
              float mmin_value, float mmax_value);
float compute_PDPI(PDPI_Type *, float input1, float input2, float dt);
void reset_PDPI(PDPI_Type *mpid);
void set_PDPI_setPoint(PDPI_Type *mpid, float mset_point1, float mset_point2);
void set_mode_PDPI(PDPI_Type *mpid, bool modo);
void transferPID(PID_Type *mpid, PDPI_Type *mpipd);

#endif /* APPLICATIONS_MOOEVO_PID_H_ */
