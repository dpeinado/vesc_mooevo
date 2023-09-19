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
#include "mooevo_pid.h"

/*
 *  Tipos de vehículo: uno sin límite de velocidad, otro limitado a 25 kph, un andarín, uno dual (3 velocidades) con marcha atrás, y uno
 *  que podría ser igual, pero con comportamiento diferente en la seguridad, si es que tiene.
 *
 */


typedef struct {
	systime_t current_time;
	systime_t last_time;
	systime_t diff_time;
	systime_t loop_overshoot;
	systime_t loop_time;
	systime_t brake_timeout;
	uint16_t loop_time_filter;
	float startup_step_size;
	float filtered_loop_overshoot;
	float loop_overshoot_alpha;
	float filtered_diff_time;
	float motor_timeout;
} LoopManagerType;

typedef enum {
	Vehiculo_sin_limites = 0,
	Vehiculo_a_25kph,
	Walker_Clean,
	Carro_26,
	Yawer
} VehicleType;

/*
 * El modo de vehículo sin límites (se selecciona mediante el display, o en caso de que no exista display, por defecto está en valor 0), tendrá
 * un límite de 25 kph si el tipo (seleccionable por vesc_tool) es Vehiculo_a_25kph, o ninguno si es Vehículo_sin_límites
 */

typedef enum {
	Sin_limites = 0,
	Andarin,
	Vehiculo_tortuga,
	Vehiculo_conejo
} VehicleMode;

/*
 * Los estados del freno del hombre muerto. El que el HM_FREE tenga el valor 0, permite utilizarlo como booleano para ver si está activado o
 * no ( if (HombreMuerto > HM_FREE) .... ). HM_BRAKING en el intervalo definido por brake_timeout se frena con la intensidad determinada
 * por brake intensity. Si consigue detenerse y no tiene que hacer nada para mantenerse detenido, estará en HM_STOPPED. Si no, entrará en el
 * estado de HM_CONTROL_PID, y se producirá un lazo de control para asegurarse que el vehículo se queda detenido
 */

typedef enum {
	HM_FREE = 0,
	HM_BRAKING,
	HM_STOPPED,
	HM_CONTROL_PID
} HombreMuertoState;

/*
 * Este es el estado del vehículo. Tiene el tipo de vehículo (dado por el vesc_tool), el modo de vehículo (dado por el display),
 * el estado del HM, dado por la entrada de éste (a definir en cada vehículo), la entrada de acelerador - float pwr-, si está o no activada
 * la marcha atrás -bool reversa-, y por último si está activado el freno de servicio -bool freno-.
 */

typedef struct {
	VehicleType tipoVehiculo;
	VehicleMode modoVehiculo;
	HombreMuertoState estadoHombreMuerto;
	float pwr;
	bool reversa;
	bool freno;
	float max_rpm_conf;
	float min_rpm_conf;
} VehicleState;

/*
 * paquete de datos intercambiado con el display. se selecciona el modo de vehículo, la marcha atrás, se muestra velocidad e intensidad
 */
typedef struct {
  uint8_t   	modo; // modo 0 sin límites, modo 1 andarin, modo 2 vehiculo lento, modo 3 vehiculo rápido
  bool      	reversa; // marcha atrás o no
  uint16_t  	velocidad; // velocidad multiplicado por 10, ejemplo 16.4 kmph -> 164
  uint16_t  	intensidad; // intensidad multiplicada por 10, ejemplo 2,67 -> 27 // 28.5 -> 285
} DisplayCommParameters;

typedef struct {
	float erpm;
	float erpm_last;
	float erpm_filtered;
	float current;
}MotorParam;

typedef struct {
	systime_t timeout;
	MotorParam motorMaster;
	MotorParam motorSlave;
	PID_Type frenoMaster;
	PID_Type frenoSlave;
	float current_max;
	float abs_max_rpm;
} VehicleParameters;

typedef struct {
	VehicleType tipoVehiculo;
	  float erpmM_25kph;  // maximas erpms en marcha atras
	  float erpmM_andando;      // maximas erpm andando
	  float erpmM_tortuga;     // maximas erpm como vehiculo tortuga
	  float erpmM_conejo;     // maximas erpm como vehiculo conejo
	  float erpmM_marcha_atras;     // multiplicador de erpmM_vehiculo para modo intermedio entre 0 y 1
	  float max_omega;          // maximo valor de la aceleracion angular
	  float omega_cut;          // cuando empieza a recortar aceleracion
	  float k_filter;           // valor filtro para aceleracion (0 superfiltrado 1 sin filtrar)
	  float min_curr;		// valor corriente mínima para parar el control pid del hm
	  float min_rpm;		// valor revoluciones mínimas para parar el control pid del hm
	  float brake_current;
	  float brake_timeout;
}VehicleConfiguration;




DisplayCommParameters * get_display_parameters(void);

#endif /* APPLICATIONS_APP_0_MOOEVO_H_ */
