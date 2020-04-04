# include "MultiControl.h"

MultiControl::MultiControl(/* args */)
{
}

MultiControl::~MultiControl()
{
}

const AP_Param::GroupInfo MultiControl::var_info[] = {
    ////////////////////////////////
    /* Multicopter configuration */

    // @Param: MASS
    // @DisplayName: Multicopter mass
    // @Description: The mass of the multicopter to be controlled
    // @Range: 0.0 3.4E+38
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("MASS", 0, MultiControl, _mass, MASS_DEFAULT),

    // @Param: M_INERTIA
    // @DisplayName: Moments of Inertia
    // @Description: The moments of inertia of the multicopter to be controlled
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("M_INERTIA", 1, MultiControl, _momentsOfInertia, MOMENTS_OF_INERTIA_DEFAULT),

    // @Param: P_INERTIA
    // @DisplayName: Products of Inertia
    // @Description: The products of inertia of the multicopter to be controlled
    // @Range: -3.4E+38 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("P_INERTIA", 2, MultiControl, _productsOfInertia, PRODUCTS_OF_INERTIA_DEFAULT),

    // @Param: FRICTION
    // @DisplayName: Air friction coefficients
    // @Description: The air friction coefficients for the main X, Y and Z body axis
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("FRICTION", 3, MultiControl, _mainTranslationalFriction, MAIN_TRANSLATIONAL_FRICTION_DEFAULT),

    // @Param: NUM_ROT
    // @DisplayName: Number of rotors
    // @Description: Number of rotors in the entire aircraft. Ex.: 8, for an octacopter
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("NUM_ROT", 4, MultiControl, _numberOfRotors, NUMBER_OF_ROTORS_DEFAULT),

    // @Param: ROT_INERT
    // @DisplayName: Rotor inertia
    // @Description: The inertia for a rotor component, including propeller
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("ROT_INERT", 5, MultiControl, _rotorInertia, ROTOR_INERTIA_DEFAULT),

    // @Param: ROT_LIFT
    // @DisplayName: Rotor lift/thrust coefficient
    // @Description: Lift/thrust coefficient for the propeller in relation to square of rotor speed 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2
    // @User: Advanced
    AP_GROUPINFO("ROT_LIFT", 6, MultiControl, _rotorLiftCoeff, ROTOR_LIFT_COEFF_DEFAULT),

    // @Param: ROT_DRAG
    // @DisplayName: Rotor drag coefficient
    // @Description: Drag coefficient for the propeller in relation to square of rotor speed 
    // @Range: 0.0 3.4E+38
    // @Units: N.m.s^2
    // @User: Advanced
    AP_GROUPINFO("ROT_DRAG", 7, MultiControl, _rotorDragCoeff, ROTOR_DRAG_COEFF_DEFAULT),

    // @Param: ROT_MAX
    // @DisplayName: Rotor maximum speed
    // @Description: Maximum rotational speed one rotor can achieve 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_MAX", 8, MultiControl, _rotorMaxSpeed, ROTOR_MAX_SPEED_DEFAULT),

    // @Param: ROT_OP
    // @DisplayName: Rotor speed operating point
    // @Description: Nominal rotor speed necessary to achieve hover flight 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_OP", 9, MultiControl, _rotorOperatingPoint, ROTOR_OPERATING_POINT_DEFAULT),

    // @Param: ROT_MIN
    // @DisplayName: Rotor minimum speed
    // @Description: Minimum rotational speed one rotor has to have 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_MIN", 10, MultiControl, _rotorMinSpeed, ROTOR_MIN_SPEED_DEFAULT),

    // @Param: ROT_RM
    // @DisplayName: Motor electrical resistance
    // @Description: Estimated electrical resistance of the motor spinning the propellers 
    // @Range: 0.0 3.4E+38
    // @Units: Ohms
    // @User: Advanced
    AP_GROUPINFO("ROT_RM", 11, MultiControl, _rotorRm, ROTOR_RM_DEFAULT),

    // @Param: ROT_L
    // @DisplayName: Motor inductance
    // @Description: Estimated inductance of the motor spinning the propellers 
    // @Range: 0.0 3.4E+38
    // @Units: Henry
    // @User: Advanced
    AP_GROUPINFO("ROT_L", 12, MultiControl, _rotorL, ROTOR_L_DEFAULT),

    // @Param: ROT_KT
    // @DisplayName: Motor torque coefficient
    // @Description: Estimated torque coefficient for the motor. Torque/Current relation
    // @Range: 0.0 3.4E+38
    // @Units: N.m/A
    // @User: Advanced
    AP_GROUPINFO("ROT_KT", 13, MultiControl, _rotorKt, ROTOR_KT_DEFAULT),

    // @Param: ROT_KV
    // @DisplayName: Motor speed coefficient
    // @Description: Estimated speed coefficient for the motor. Speed/Voltage relation
    // @Range: 0.0 3.4E+38
    // @Units: RPM/V
    // @User: Advanced
    AP_GROUPINFO("ROT_KV", 14, MultiControl, _rotorKv, ROTOR_KV_DEFAULT),

    // @Param: ROT_IO
    // @DisplayName: Motor idle current rate
    // @Description: Idle current rate for the motor. Amperes/Volts 
    // @Range: 0.0 3.4E+38
    // @Units: A/V
    // @User: Advanced
    AP_GROUPINFO("ROT_IO", 15, MultiControl, _rotorIo, ROTOR_IO_DEFAULT),

    // @Param: ROT_MAX_V
    // @DisplayName: Motor maximum voltage
    // @Description: Maximum voltage allowed on the motors 
    // @Range: 0.0 3.4E+38
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("ROT_MAX_V", 16, MultiControl, _rotorMaxVoltage, ROTOR_MAX_VOLTAGE_DEFAULT),

    // @Param: ROT1_POS
    // @DisplayName: Rotor 1 position
    // @Description: Position of rotor 1 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_POS", 17, MultiControl, _rotor1Position, ROTOR1_POSITION_DEFAULT),

    // @Param: ROT2_POS
    // @DisplayName: Rotor 2 position
    // @Description: Position of rotor 2 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_POS", 18, MultiControl, _rotor2Position, ROTOR2_POSITION_DEFAULT),

    // @Param: ROT3_POS
    // @DisplayName: Rotor 3 position
    // @Description: Position of rotor 3 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_POS", 19, MultiControl, _rotor3Position, ROTOR3_POSITION_DEFAULT),

    // @Param: ROT4_POS
    // @DisplayName: Rotor 4 position
    // @Description: Position of rotor 4 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_POS", 20, MultiControl, _rotor4Position, ROTOR4_POSITION_DEFAULT),

    // @Param: ROT5_POS
    // @DisplayName: Rotor 5 position
    // @Description: Position of rotor 5 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_POS", 21, MultiControl, _rotor5Position, ROTOR5_POSITION_DEFAULT),

    // @Param: ROT6_POS
    // @DisplayName: Rotor 6 position
    // @Description: Position of rotor 6 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_POS", 22, MultiControl, _rotor6Position, ROTOR6_POSITION_DEFAULT),

    // @Param: ROT7_POS
    // @DisplayName: Rotor 7 position
    // @Description: Position of rotor 7 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_POS", 23, MultiControl, _rotor7Position, ROTOR7_POSITION_DEFAULT),

    // @Param: ROT8_POS
    // @DisplayName: Rotor 8 position
    // @Description: Position of rotor 8 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_POS", 24, MultiControl, _rotor8Position, ROTOR8_POSITION_DEFAULT),

    // @Param: ROT1_ORI
    // @DisplayName: Rotor 1 orientation
    // @Description: Orientation of rotor 1 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_ORI", 25, MultiControl, _rotor1Orientation, ROTOR1_ORIENTATION_DEFAULT),

    // @Param: ROT2_ORI
    // @DisplayName: Rotor 2 orientation
    // @Description: Orientation of rotor 2 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_ORI", 26, MultiControl, _rotor2Orientation, ROTOR2_ORIENTATION_DEFAULT),

    // @Param: ROT3_ORI
    // @DisplayName: Rotor 3 orientation
    // @Description: Orientation of rotor 3 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_ORI", 27, MultiControl, _rotor3Orientation, ROTOR3_ORIENTATION_DEFAULT),

    // @Param: ROT4_ORI
    // @DisplayName: Rotor 4 orientation
    // @Description: Orientation of rotor 4 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_ORI", 28, MultiControl, _rotor4Orientation, ROTOR4_ORIENTATION_DEFAULT),

    // @Param: ROT5_ORI
    // @DisplayName: Rotor 5 orientation
    // @Description: Orientation of rotor 5 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_ORI", 29, MultiControl, _rotor5Orientation, ROTOR5_ORIENTATION_DEFAULT),

    // @Param: ROT6_ORI
    // @DisplayName: Rotor 6 orientation
    // @Description: Orientation of rotor 6 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_ORI", 30, MultiControl, _rotor6Orientation, ROTOR6_ORIENTATION_DEFAULT),

    // @Param: ROT7_ORI
    // @DisplayName: Rotor 7 orientation
    // @Description: Orientation of rotor 7 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_ORI", 31, MultiControl, _rotor7Orientation, ROTOR7_ORIENTATION_DEFAULT),

    // @Param: ROT8_ORI
    // @DisplayName: Rotor 8 orientation
    // @Description: Orientation of rotor 8 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_ORI", 32, MultiControl, _rotor8Orientation, ROTOR8_ORIENTATION_DEFAULT),

    // @Param: ROT1_DIR
    // @DisplayName: Rotor 1 rotation direction
    // @Description: Direction of rotation of rotor 1 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_DIR", 33, MultiControl, _rotor1Direction, ROTOR1_DIRECTION_DEFAULT),

    // @Param: ROT2_DIR
    // @DisplayName: Rotor 2 rotation direction
    // @Description: Direction of rotation of rotor 2 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_DIR", 34, MultiControl, _rotor2Direction, ROTOR2_DIRECTION_DEFAULT),

    // @Param: ROT3_DIR
    // @DisplayName: Rotor 3 rotation direction
    // @Description: Direction of rotation of rotor 3 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_DIR", 35, MultiControl, _rotor3Direction, ROTOR3_DIRECTION_DEFAULT),

    // @Param: ROT4_DIR
    // @DisplayName: Rotor 4 rotation direction
    // @Description: Direction of rotation of rotor 4 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_DIR", 36, MultiControl, _rotor4Direction, ROTOR4_DIRECTION_DEFAULT),

    // @Param: ROT5_DIR
    // @DisplayName: Rotor 5 rotation direction
    // @Description: Direction of rotation of rotor 5 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_DIR", 37, MultiControl, _rotor5Direction, ROTOR5_DIRECTION_DEFAULT),

    // @Param: ROT6_DIR
    // @DisplayName: Rotor 6 rotation direction
    // @Description: Direction of rotation of rotor 6 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_DIR", 38, MultiControl, _rotor6Direction, ROTOR6_DIRECTION_DEFAULT),

    // @Param: ROT7_DIR
    // @DisplayName: Rotor 7 rotation direction
    // @Description: Direction of rotation of rotor 7 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_DIR", 39, MultiControl, _rotor7Direction, ROTOR7_DIRECTION_DEFAULT),

    // @Param: ROT8_DIR
    // @DisplayName: Rotor 8 rotation direction
    // @Description: Direction of rotation of rotor 8 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_DIR", 40, MultiControl, _rotor8Direction, ROTOR8_DIRECTION_DEFAULT),

    ////////////////////////////////
    /* Position PIDD configuration */

    // @Param: PIDD_KP
    // @DisplayName: Position PIDD Kp value
    // @Description: Position error gain for the position control PIDD 
    // @Range: 0.0 3.4E+38
    // @Units: N/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KP", 41, MultiControl, _piddKp, PIDD_KP_DEFAULT),

    // @Param: PIDD_KI
    // @DisplayName: Position PIDD Ki value
    // @Description: Position error integral gain for the position control PIDD 
    // @Range: 0.0 3.4E+38
    // @Units: N/m.s
    // @User: Advanced
    AP_GROUPINFO("PIDD_KI", 42, MultiControl, _piddKi, PIDD_KI_DEFAULT),

    // @Param: PIDD_KD
    // @DisplayName: Position PIDD Kd value
    // @Description: Velocity error gain for the position control PIDD 
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KD", 43, MultiControl, _piddKd, PIDD_KD_DEFAULT),

    // @Param: PIDD_KDD
    // @DisplayName: Position PIDD Kdd value
    // @Description: Acceleration error gain for the position control PIDD 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KDD", 44, MultiControl, _piddKdd, PIDD_KDD_DEFAULT),

    // @Param: FLTR_GAIN
    // @DisplayName: Angular velocity filter gain
    // @Description: Gain for the filter of the angular velocity 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FLTR_GAIN", 45, MultiControl, _velocityFilterGain, VELOCITY_FILTER_GAIN_DEFAULT),

    AP_GROUPEND
};