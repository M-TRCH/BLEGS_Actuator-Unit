
#ifndef MOTOR_CONF_H
#define MOTOR_CONF_H

#define MOTOR_TYPE                  BLDC_MOTOR      // Motor type: BLDC_MOTOR, PMSM_MOTOR, or STEPPER_MOTOR
#define NOMINAL_VOLTAGE             24.0f           // Nominal voltage in Volts
#define POWER                       65.0f           // Rated power in Watts
#define NOMINAL_TORQUE              5.0f            // Nominal torque in Nm
#define STALL_TORQUE                11.0f           // Stall torque in Nm
#define NOMINAL_SPEED_REDUCE        120.0f          // Nominal speed after reduction in RPM
#define NOMINAL_CURRENT             10.5f           // Nominal current in Amperes
#define STALL_CURRENT               25.0f           // Stall current in Amperes
#define PHASE_TO_PHASE_RESISTANCE   0.214f          // Phase to phase resistance in Ohms
#define PHASE_TO_PHASE_INDUCTANCE   0.000000124f    // Phase to phase inductance in Henrys
#define SPEED_CONSTANT              12.3f           // Speed constant in rpm/V
#define TORQUE_CONSTANT             0.47f           // Torque constant in Nm/A
#define ROTOR_INERTIA               26.3            // Rotor inertia in g.cm^2
#define POLE_PAIRS                  14.0f           // Number of pole pairs   
#define GEAR_RATIO                  8.0f            // Gear ratio (8:1 for direct drive)

#endif // MOTOR_CONF_H