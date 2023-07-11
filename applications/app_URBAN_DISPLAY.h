/*
 * app_adc_URBAN_DISPLAY.h
 *
 *  Created on: 21 oct. 2022
 *      Author: Usuario01
 */

#ifndef APPLICATIONS_APP_URBAN_DISPLAY_H_
#define APPLICATIONS_APP_URBAN_DISPLAY_H_

typedef struct {
  float erpmM_marchaAtras;  // m�ximas erpms en marcha atr�s
  float erpmM_andando;      // m�ximas erpm andando
  float erpmM_vehiculo;     // m�ximas erpm como veh�culo
  float vel_proporcion;     // multiplicador de erpmM_vehiculo para modo intermedio entre 0 y 1
  float max_omega;          // m�ximo valor de la aceleraci�n angular
  float omega_cut;          // cuando empieza a recortar aceleraci�n
  float k_filter;           // valor filtro para aceleraci�n (0 superfiltrado 1 sin filtrar)
} urban_config;


typedef struct {
  uint8_t   modo; // modo 0 parado, modo 1 andarin, modo 2 vehiculo lento, modo 3 vehiculo r�pido
  bool      reversa; // marcha atr�s o no
  uint16_t  velocidad; // velocidad multiplicado por 10, ejemplo 16.4 kmph -> 164
  uint16_t  intensidad; // intensidad multiplicada por 10, ejemplo 2,67 -> 27 // 28.5 -> 285
//  bool      frenado; // Se ha actuado el freno y se ha recibido la se�al de frenando
//  char      lbv;
//  char      hbv;
//  char      error;
} tipo_estado_vehiculo;


tipo_estado_vehiculo *get_estado_vehiculo(void);

#endif /* APPLICATIONS_APP_URBAN_DISPLAY_H_ */
