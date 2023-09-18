/*
 * app_0_Mooevo.h
 *
 *  Created on: Sep 18, 2023
 *      Author: bicho
 */

#ifndef APPLICATIONS_APP_0_MOOEVO_H_
#define APPLICATIONS_APP_0_MOOEVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	Vehiculo_sin_limites = 0,
	Vehiculo_a_25kph,
	Walker_Clean,
	Carro_26,
	Yawer
} VehicleType;


typedef enum{
	Sin_limites = 0,
	Andarin,
	Vehiculo_tortuga,
	Vehiculo_conejo
} VehicleMode;

typedef enum{
	HM_FREE = 0,
	HM_BRAKING,
	HM_STOPPED,
	HM_CONTROL_PID
} HombreMuertoState;

typedef struct{
	VehicleType tipoVehiculo;
	VehicleMode modoVehiculo;
	HombreMuertoState estadoHombreMuerto;
	bool reversa;
	bool freno;
	float pwr;
} VehicleState;

typedef struct {
  uint8_t   	modo; // modo 0 sin límites, modo 1 andarin, modo 2 vehiculo lento, modo 3 vehiculo rápido
  bool      	reversa; // marcha atrás o no
  uint16_t  	velocidad; // velocidad multiplicado por 10, ejemplo 16.4 kmph -> 164
  uint16_t  	intensidad; // intensidad multiplicada por 10, ejemplo 2,67 -> 27 // 28.5 -> 285
} DisplayCommParameters;



#endif /* APPLICATIONS_APP_0_MOOEVO_H_ */
