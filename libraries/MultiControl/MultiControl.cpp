# include "MultiControl.h"

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

    //--------------------------
    // @Param: M_INER_XX
    // @DisplayName: Moment of Inertia XX
    // @Description: The moment of inertia around X axis of the multicopter to be controlled
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("M_INER_XX", 1, MultiControl, _momentsOfInertia[0], MOMENTS_OF_INERTIA_XX_DEFAULT),

    // @Param: M_INER_YY
    // @DisplayName: Moment of Inertia YY
    // @Description: The moment of inertia around Y axis of the multicopter to be controlled
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("M_INER_YY", 2, MultiControl, _momentsOfInertia[1], MOMENTS_OF_INERTIA_YY_DEFAULT),

    // @Param: M_INER_ZZ
    // @DisplayName: Moment of Inertia ZZ
    // @Description: The moment of inertia around Z axis of the multicopter to be controlled
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("M_INER_ZZ", 3, MultiControl, _momentsOfInertia[2], MOMENTS_OF_INERTIA_ZZ_DEFAULT),
    //--------------------------
    //--------------------------
    // @Param: P_INER_XY
    // @DisplayName: Product of Inertia XY
    // @Description: The product of inertia on plane XY of the multicopter to be controlled
    // @Range: -3.4E+38 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("P_INER_XY", 4, MultiControl, _productsOfInertia[0], PRODUCTS_OF_INERTIA_XY_DEFAULT),

    // @Param: P_INER_XZ
    // @DisplayName: Product of Inertia XZ
    // @Description: The product of inertia on plane XZ of the multicopter to be controlled
    // @Range: -3.4E+38 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("P_INER_XZ", 5, MultiControl, _productsOfInertia[1], PRODUCTS_OF_INERTIA_XZ_DEFAULT),
    
    // @Param: P_INER_YZ
    // @DisplayName: Product of Inertia YZ
    // @Description: The product of inertia on plane YZ of the multicopter to be controlled
    // @Range: -3.4E+38 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("P_INER_YZ", 6, MultiControl, _productsOfInertia[2], PRODUCTS_OF_INERTIA_YZ_DEFAULT),
    //--------------------------
    //--------------------------
    // @Param: FRICT_XX
    // @DisplayName: Air friction coefficient on X axis direction
    // @Description: The air friction coefficients for the main X body axis direction
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("FRICT_XX", 7, MultiControl, _mainTranslationalFriction[0], MAIN_TRANSLATIONAL_FRICTION_X_DEFAULT),

    // @Param: FRICT_YY
    // @DisplayName: Air friction coefficient on Y axis direction
    // @Description: The air friction coefficients for the main Y body axis direction
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("FRICT_YY", 8, MultiControl, _mainTranslationalFriction[1], MAIN_TRANSLATIONAL_FRICTION_Y_DEFAULT),

    // @Param: FRICT_ZZ
    // @DisplayName: Air friction coefficient on Z axis direction
    // @Description: The air friction coefficients for the main Z body axis direction
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("FRICT_ZZ", 9, MultiControl, _mainTranslationalFriction[2], MAIN_TRANSLATIONAL_FRICTION_Z_DEFAULT),
    //--------------------------
    //--------------------------
    // @Param: NUM_ROT
    // @DisplayName: Number of rotors
    // @Description: Number of rotors in the entire aircraft. Ex.: 8, for an octacopter
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("NUM_ROT", 10, MultiControl, _numberOfRotors, NUMBER_OF_ROTORS_DEFAULT),

    // @Param: ROT_INERT
    // @DisplayName: Rotor inertia
    // @Description: The inertia for a rotor component, including propeller
    // @Range: 0.0 3.4E+38
    // @Units: kg.m^2
    // @User: Advanced
    AP_GROUPINFO("ROT_INERT", 11, MultiControl, _rotorInertia, ROTOR_INERTIA_DEFAULT),

    // @Param: ROT_LIFT
    // @DisplayName: Rotor lift/thrust coefficient
    // @Description: Lift/thrust coefficient for the propeller in relation to square of rotor speed 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2
    // @User: Advanced
    AP_GROUPINFO("ROT_LIFT", 12, MultiControl, _rotorLiftCoeff, ROTOR_LIFT_COEFF_DEFAULT),

    // @Param: ROT_DRAG
    // @DisplayName: Rotor drag coefficient
    // @Description: Drag coefficient for the propeller in relation to square of rotor speed 
    // @Range: 0.0 3.4E+38
    // @Units: N.m.s^2
    // @User: Advanced
    AP_GROUPINFO("ROT_DRAG", 13, MultiControl, _rotorDragCoeff, ROTOR_DRAG_COEFF_DEFAULT),

    // @Param: ROT_MAX
    // @DisplayName: Rotor maximum speed
    // @Description: Maximum rotational speed one rotor can achieve 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_MAX", 14, MultiControl, _rotorMaxSpeed, ROTOR_MAX_SPEED_DEFAULT),

    // @Param: ROT_OP
    // @DisplayName: Rotor speed operating point
    // @Description: Nominal rotor speed necessary to achieve hover flight 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_OP", 15, MultiControl, _rotorOperatingPoint, ROTOR_OPERATING_POINT_DEFAULT),

    // @Param: ROT_MIN
    // @DisplayName: Rotor minimum speed
    // @Description: Minimum rotational speed one rotor has to have 
    // @Range: 0.0 3.4E+38
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("ROT_MIN", 16, MultiControl, _rotorMinSpeed, ROTOR_MIN_SPEED_DEFAULT),

    // @Param: ROT_RM
    // @DisplayName: Motor electrical resistance
    // @Description: Estimated electrical resistance of the motor spinning the propellers 
    // @Range: 0.0 3.4E+38
    // @Units: Ohms
    // @User: Advanced
    AP_GROUPINFO("ROT_RM", 17, MultiControl, _rotorRm, ROTOR_RM_DEFAULT),

    // @Param: ROT_L
    // @DisplayName: Motor inductance
    // @Description: Estimated inductance of the motor spinning the propellers 
    // @Range: 0.0 3.4E+38
    // @Units: Henry
    // @User: Advanced
    AP_GROUPINFO("ROT_L", 18, MultiControl, _rotorL, ROTOR_L_DEFAULT),

    // @Param: ROT_KT
    // @DisplayName: Motor torque coefficient
    // @Description: Estimated torque coefficient for the motor. Torque/Current relation
    // @Range: 0.0 3.4E+38
    // @Units: N.m/A
    // @User: Advanced
    AP_GROUPINFO("ROT_KT", 19, MultiControl, _rotorKt, ROTOR_KT_DEFAULT),

    // @Param: ROT_KV
    // @DisplayName: Motor speed coefficient
    // @Description: Estimated speed coefficient for the motor. Speed/Voltage relation
    // @Range: 0.0 3.4E+38
    // @Units: RPM/V
    // @User: Advanced
    AP_GROUPINFO("ROT_KV", 20, MultiControl, _rotorKv, ROTOR_KV_DEFAULT),

    // @Param: ROT_IO
    // @DisplayName: Motor idle current rate
    // @Description: Idle current rate for the motor. Amperes/Volts 
    // @Range: 0.0 3.4E+38
    // @Units: A/V
    // @User: Advanced
    AP_GROUPINFO("ROT_IO", 21, MultiControl, _rotorIo, ROTOR_IO_DEFAULT),

    // @Param: ROT_MAX_V
    // @DisplayName: Motor maximum voltage
    // @Description: Maximum voltage allowed on the motors 
    // @Range: 0.0 3.4E+38
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("ROT_MAX_V", 22, MultiControl, _rotorMaxVoltage, ROTOR_MAX_VOLTAGE_DEFAULT),

    //--------------------------
    // @Param: RT1_POS_X
    // @DisplayName: Rotor 1 position X
    // @Description: Position of rotor 1 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_POS_X", 23, MultiControl, _rotorPosition[0][0], ROTOR1_POSITION_X_DEFAULT),

    // @Param: RT1_POS_Y
    // @DisplayName: Rotor 1 position Y
    // @Description: Position of rotor 1 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_POS_Y", 24, MultiControl, _rotorPosition[0][1], ROTOR1_POSITION_Y_DEFAULT),

    // @Param: RT1_POS_Z
    // @DisplayName: Rotor 1 position Z
    // @Description: Position of rotor 1 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_POS_Z", 25, MultiControl, _rotorPosition[0][2], ROTOR1_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT2_POS_X
    // @DisplayName: Rotor 2 position X
    // @Description: Position of rotor 2 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_POS_X", 26, MultiControl, _rotorPosition[1][0], ROTOR2_POSITION_X_DEFAULT),

    // @Param: RT2_POS_Y
    // @DisplayName: Rotor 2 position Y
    // @Description: Position of rotor 2 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_POS_Y", 27, MultiControl, _rotorPosition[1][1], ROTOR2_POSITION_Y_DEFAULT),

    // @Param: RT2_POS_Z
    // @DisplayName: Rotor 2 position Z
    // @Description: Position of rotor 2 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_POS_Z", 28, MultiControl, _rotorPosition[1][2], ROTOR2_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT3_POS_X
    // @DisplayName: Rotor 3 position X
    // @Description: Position of rotor 3 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_POS_X", 29, MultiControl, _rotorPosition[2][0], ROTOR3_POSITION_X_DEFAULT),

    // @Param: RT3_POS_Y
    // @DisplayName: Rotor 3 position Y
    // @Description: Position of rotor 3 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_POS_Y", 30, MultiControl, _rotorPosition[2][1], ROTOR3_POSITION_Y_DEFAULT),

    // @Param: RT3_POS_Z
    // @DisplayName: Rotor 3 position Z
    // @Description: Position of rotor 3 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_POS_Z", 31, MultiControl, _rotorPosition[2][2], ROTOR3_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT4_POS_X
    // @DisplayName: Rotor 4 position X
    // @Description: Position of rotor 4 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_POS_X", 32, MultiControl, _rotorPosition[3][0], ROTOR4_POSITION_X_DEFAULT),

    // @Param: RT4_POS_Y
    // @DisplayName: Rotor 4 position Y
    // @Description: Position of rotor 4 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_POS_Y", 33, MultiControl, _rotorPosition[3][1], ROTOR4_POSITION_Y_DEFAULT),

    // @Param: RT4_POS_Z
    // @DisplayName: Rotor 4 position Z
    // @Description: Position of rotor 4 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_POS_Z", 34, MultiControl, _rotorPosition[3][2], ROTOR4_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT5_POS_X
    // @DisplayName: Rotor 5 position X
    // @Description: Position of rotor 5 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_POS_X", 35, MultiControl, _rotorPosition[4][0], ROTOR5_POSITION_X_DEFAULT),

    // @Param: RT5_POS_Y
    // @DisplayName: Rotor 5 position Y
    // @Description: Position of rotor 5 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_POS_Y", 36, MultiControl, _rotorPosition[4][1], ROTOR5_POSITION_Y_DEFAULT),

    // @Param: RT5_POS_Z
    // @DisplayName: Rotor 5 position Z
    // @Description: Position of rotor 5 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_POS_Z", 37, MultiControl, _rotorPosition[4][2], ROTOR5_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT6_POS_X
    // @DisplayName: Rotor 6 position X
    // @Description: Position of rotor 6 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_POS_X", 38, MultiControl, _rotorPosition[5][0], ROTOR6_POSITION_X_DEFAULT),

    // @Param: RT6_POS_Y
    // @DisplayName: Rotor 6 position Y
    // @Description: Position of rotor 6 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_POS_Y", 39, MultiControl, _rotorPosition[5][1], ROTOR6_POSITION_Y_DEFAULT),

    // @Param: RT6_POS_Z
    // @DisplayName: Rotor 6 position Z
    // @Description: Position of rotor 6 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_POS_Z", 40, MultiControl, _rotorPosition[5][2], ROTOR6_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT7_POS_X
    // @DisplayName: Rotor 7 position X
    // @Description: Position of rotor 7 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_POS_X", 41, MultiControl, _rotorPosition[6][0], ROTOR7_POSITION_X_DEFAULT),

    // @Param: RT7_POS_Y
    // @DisplayName: Rotor 7 position Y
    // @Description: Position of rotor 7 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_POS_Y", 42, MultiControl, _rotorPosition[6][1], ROTOR7_POSITION_Y_DEFAULT),

    // @Param: RT7_POS_Z
    // @DisplayName: Rotor 7 position Z
    // @Description: Position of rotor 7 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_POS_Z", 43, MultiControl, _rotorPosition[6][2], ROTOR7_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT8_POS_X
    // @DisplayName: Rotor 8 position X
    // @Description: Position of rotor 8 in relation to CG over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_POS_X", 44, MultiControl, _rotorPosition[7][0], ROTOR8_POSITION_X_DEFAULT),

    // @Param: RT8_POS_Y
    // @DisplayName: Rotor 8 position Y
    // @Description: Position of rotor 8 in relation to CG over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_POS_Y", 45, MultiControl, _rotorPosition[7][1], ROTOR8_POSITION_Y_DEFAULT),

    // @Param: RT8_POS_Z
    // @DisplayName: Rotor 8 position Z
    // @Description: Position of rotor 8 in relation to CG over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_POS_Z", 46, MultiControl, _rotorPosition[7][2], ROTOR8_POSITION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT1_ORI_X
    // @DisplayName: Rotor 1 orientation X
    // @Description: Orientation of rotor 1 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_ORI_X", 47, MultiControl, _rotorOrientation[0][0], ROTOR1_ORIENTATION_X_DEFAULT),

    // @Param: RT1_ORI_Y
    // @DisplayName: Rotor 1 orientation Y
    // @Description: Orientation of rotor 1 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_ORI_Y", 48, MultiControl, _rotorOrientation[0][1], ROTOR1_ORIENTATION_Y_DEFAULT),

    // @Param: RT1_ORI_Z
    // @DisplayName: Rotor 1 orientation Z
    // @Description: Orientation of rotor 1 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT1_ORI_Z", 49, MultiControl, _rotorOrientation[0][2], ROTOR1_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT2_ORI_X
    // @DisplayName: Rotor 2 orientation X
    // @Description: Orientation of rotor 2 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_ORI_X", 50, MultiControl, _rotorOrientation[1][0], ROTOR2_ORIENTATION_X_DEFAULT),

    // @Param: RT2_ORI_Y
    // @DisplayName: Rotor 2 orientation Y
    // @Description: Orientation of rotor 2 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_ORI_Y", 51, MultiControl, _rotorOrientation[1][1], ROTOR2_ORIENTATION_Y_DEFAULT),

    // @Param: RT2_ORI_Z
    // @DisplayName: Rotor 2 orientation Z
    // @Description: Orientation of rotor 2 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT2_ORI_Z", 52, MultiControl, _rotorOrientation[1][2], ROTOR2_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT3_ORI_X
    // @DisplayName: Rotor 3 orientation X
    // @Description: Orientation of rotor 3 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_ORI_X", 53, MultiControl, _rotorOrientation[2][0], ROTOR3_ORIENTATION_X_DEFAULT),

    // @Param: RT3_ORI_Y
    // @DisplayName: Rotor 3 orientation Y
    // @Description: Orientation of rotor 3 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_ORI_Y", 54, MultiControl, _rotorOrientation[2][1], ROTOR3_ORIENTATION_Y_DEFAULT),

    // @Param: RT3_ORI_Z
    // @DisplayName: Rotor 3 orientation Z
    // @Description: Orientation of rotor 3 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT3_ORI_Z", 55, MultiControl, _rotorOrientation[2][2], ROTOR3_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT4_ORI_X
    // @DisplayName: Rotor 4 orientation X
    // @Description: Orientation of rotor 4 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_ORI_X", 56, MultiControl, _rotorOrientation[3][0], ROTOR4_ORIENTATION_X_DEFAULT),

    // @Param: RT4_ORI_Y
    // @DisplayName: Rotor 4 orientation Y
    // @Description: Orientation of rotor 4 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_ORI_Y", 57, MultiControl, _rotorOrientation[3][1], ROTOR4_ORIENTATION_Y_DEFAULT),

    // @Param: RT4_ORI_Z
    // @DisplayName: Rotor 4 orientation Z
    // @Description: Orientation of rotor 4 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT4_ORI_Z", 58, MultiControl, _rotorOrientation[3][2], ROTOR4_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT5_ORI_X
    // @DisplayName: Rotor 5 orientation X
    // @Description: Orientation of rotor 5 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_ORI_X", 59, MultiControl, _rotorOrientation[4][0], ROTOR5_ORIENTATION_X_DEFAULT),

    // @Param: RT5_ORI_Y
    // @DisplayName: Rotor 5 orientation Y
    // @Description: Orientation of rotor 5 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_ORI_Y", 60, MultiControl, _rotorOrientation[4][1], ROTOR5_ORIENTATION_Y_DEFAULT),

    // @Param: RT5_ORI_Z
    // @DisplayName: Rotor 5 orientation Z
    // @Description: Orientation of rotor 5 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT5_ORI_Z", 61, MultiControl, _rotorOrientation[4][2], ROTOR5_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT6_ORI_X
    // @DisplayName: Rotor 6 orientation X
    // @Description: Orientation of rotor 6 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_ORI_X", 62, MultiControl, _rotorOrientation[5][0], ROTOR6_ORIENTATION_X_DEFAULT),

    // @Param: RT6_ORI_Y
    // @DisplayName: Rotor 6 orientation Y
    // @Description: Orientation of rotor 6 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_ORI_Y", 63, MultiControl, _rotorOrientation[5][1], ROTOR6_ORIENTATION_Y_DEFAULT),

    // @Param: RT6_ORI_Z
    // @DisplayName: Rotor 6 orientation Z
    // @Description: Orientation of rotor 6 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT6_ORI_Z", 64, MultiControl, _rotorOrientation[5][2], ROTOR6_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT7_ORI_X
    // @DisplayName: Rotor 7 orientation X
    // @Description: Orientation of rotor 7 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_ORI_X", 65, MultiControl, _rotorOrientation[6][0], ROTOR7_ORIENTATION_X_DEFAULT),

    // @Param: RT7_ORI_Y
    // @DisplayName: Rotor 7 orientation Y
    // @Description: Orientation of rotor 7 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_ORI_Y", 66, MultiControl, _rotorOrientation[6][1], ROTOR7_ORIENTATION_Y_DEFAULT),

    // @Param: RT7_ORI_Z
    // @DisplayName: Rotor 7 orientation Z
    // @Description: Orientation of rotor 7 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT7_ORI_Z", 67, MultiControl, _rotorOrientation[6][2], ROTOR7_ORIENTATION_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: RT8_ORI_X
    // @DisplayName: Rotor 8 orientation X
    // @Description: Orientation of rotor 8 in relation to body system over X axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_ORI_X", 68, MultiControl, _rotorOrientation[7][0], ROTOR8_ORIENTATION_X_DEFAULT),

    // @Param: RT8_ORI_Y
    // @DisplayName: Rotor 8 orientation Y
    // @Description: Orientation of rotor 8 in relation to body system over Y axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_ORI_Y", 69, MultiControl, _rotorOrientation[7][1], ROTOR8_ORIENTATION_Y_DEFAULT),

    // @Param: RT8_ORI_Z
    // @DisplayName: Rotor 8 orientation Z
    // @Description: Orientation of rotor 8 in relation to body system over Z axis. 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RT8_ORI_Z", 70, MultiControl, _rotorOrientation[7][2], ROTOR8_ORIENTATION_Z_DEFAULT),
    //--------------------------

    // @Param: ROT1_DIR
    // @DisplayName: Rotor 1 rotation direction
    // @Description: Direction of rotation of rotor 1 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_DIR", 71, MultiControl, _rotorDirection[0], ROTOR1_DIRECTION_DEFAULT),

    // @Param: ROT2_DIR
    // @DisplayName: Rotor 2 rotation direction
    // @Description: Direction of rotation of rotor 2 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_DIR", 72, MultiControl, _rotorDirection[1], ROTOR2_DIRECTION_DEFAULT),

    // @Param: ROT3_DIR
    // @DisplayName: Rotor 3 rotation direction
    // @Description: Direction of rotation of rotor 3 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_DIR", 73, MultiControl, _rotorDirection[2], ROTOR3_DIRECTION_DEFAULT),

    // @Param: ROT4_DIR
    // @DisplayName: Rotor 4 rotation direction
    // @Description: Direction of rotation of rotor 4 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_DIR", 74, MultiControl, _rotorDirection[3], ROTOR4_DIRECTION_DEFAULT),

    // @Param: ROT5_DIR
    // @DisplayName: Rotor 5 rotation direction
    // @Description: Direction of rotation of rotor 5 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_DIR", 75, MultiControl, _rotorDirection[4], ROTOR5_DIRECTION_DEFAULT),

    // @Param: ROT6_DIR
    // @DisplayName: Rotor 6 rotation direction
    // @Description: Direction of rotation of rotor 6 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_DIR", 76, MultiControl, _rotorDirection[5], ROTOR6_DIRECTION_DEFAULT),

    // @Param: ROT7_DIR
    // @DisplayName: Rotor 7 rotation direction
    // @Description: Direction of rotation of rotor 7 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_DIR", 77, MultiControl, _rotorDirection[6], ROTOR7_DIRECTION_DEFAULT),

    // @Param: ROT8_DIR
    // @DisplayName: Rotor 8 rotation direction
    // @Description: Direction of rotation of rotor 8 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_DIR", 78, MultiControl, _rotorDirection[7], ROTOR8_DIRECTION_DEFAULT),

    ////////////////////////////////
    /* Position PIDD configuration */

    //--------------------------
    // @Param: PIDD_KP_X
    // @DisplayName: Position X PIDD Kp value
    // @Description: Position error gain for the position control PIDD on X axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KP_X", 79, MultiControl, _piddKp[0], PIDD_KP_X_DEFAULT),

    // @Param: PIDD_KP_Y
    // @DisplayName: Position Y PIDD Kp value
    // @Description: Position error gain for the position control PIDD on Y axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KP_Y", 80, MultiControl, _piddKp[1], PIDD_KP_Y_DEFAULT),

    // @Param: PIDD_KP_Z
    // @DisplayName: Position Z PIDD Kp value
    // @Description: Position error gain for the position control PIDD on Z axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KP_Z", 81, MultiControl, _piddKp[2], PIDD_KP_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: PIDD_KI_X
    // @DisplayName: Position X PIDD Ki value
    // @Description: Position error integral gain for the position control PIDD on X axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m.s
    // @User: Advanced
    AP_GROUPINFO("PIDD_KI_X", 82, MultiControl, _piddKi[0], PIDD_KI_X_DEFAULT),

    // @Param: PIDD_KI_Y
    // @DisplayName: Position Y PIDD Ki value
    // @Description: Position error integral gain for the position control PIDD on Y axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m.s
    // @User: Advanced
    AP_GROUPINFO("PIDD_KI_Y", 83, MultiControl, _piddKi[1], PIDD_KI_Y_DEFAULT),

    // @Param: PIDD_KI_Z
    // @DisplayName: Position Z PIDD Ki value
    // @Description: Position error integral gain for the position control PIDD on Z axis 
    // @Range: 0.0 3.4E+38
    // @Units: N/m.s
    // @User: Advanced
    AP_GROUPINFO("PIDD_KI_Z", 84, MultiControl, _piddKi[2], PIDD_KI_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: PIDD_KD_X
    // @DisplayName: Position X PIDD Kd value
    // @Description: Velocity error gain for the position control PIDD on X axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KD_X", 85, MultiControl, _piddKd[0], PIDD_KD_X_DEFAULT),

    // @Param: PIDD_KD_Y
    // @DisplayName: Position Y PIDD Kd value
    // @Description: Velocity error gain for the position control PIDD on Y axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KD_Y", 86, MultiControl, _piddKd[1], PIDD_KD_Y_DEFAULT),

    // @Param: PIDD_KD_Z
    // @DisplayName: Position Z PIDD Kd value
    // @Description: Velocity error gain for the position control PIDD on Z axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KD_Z", 87, MultiControl, _piddKd[2], PIDD_KD_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: PIDD_KDDX
    // @DisplayName: Position X PIDD Kdd value
    // @Description: Acceleration error gain for the position control PIDD on X axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KDDX", 88, MultiControl, _piddKdd[0], PIDD_KDD_X_DEFAULT),

    // @Param: PIDD_KDDY
    // @DisplayName: Position Y PIDD Kdd value
    // @Description: Acceleration error gain for the position control PIDD on Y axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KDDY", 89, MultiControl, _piddKdd[1], PIDD_KDD_Y_DEFAULT),

    // @Param: PIDD_KDDZ
    // @DisplayName: Position Z PIDD Kdd value
    // @Description: Acceleration error gain for the position control PIDD on Z axis 
    // @Range: 0.0 3.4E+38
    // @Units: N.s^2/m
    // @User: Advanced
    AP_GROUPINFO("PIDD_KDDZ", 90, MultiControl, _piddKdd[2], PIDD_KDD_Z_DEFAULT),
    //--------------------------

    //--------------------------
    // @Param: FLTR_GN_X
    // @DisplayName: X axis angular velocity filter gain
    // @Description: Gain for the filter of the angular velocity around X axis 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FLTR_GN_X", 91, MultiControl, _velocityFilterGain[0], VELOCITY_FILTER_GAIN_X_DEFAULT),

    // @Param: FLTR_GN_Y
    // @DisplayName: Y axis angular velocity filter gain
    // @Description: Gain for the filter of the angular velocity around Y axis 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FLTR_GN_Y", 92, MultiControl, _velocityFilterGain[1], VELOCITY_FILTER_GAIN_Y_DEFAULT),

    // @Param: FLTR_GN_Z
    // @DisplayName: Z axis angular velocity filter gain
    // @Description: Gain for the filter of the angular velocity around Z axis 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FLTR_GN_Z", 93, MultiControl, _velocityFilterGain[2], VELOCITY_FILTER_GAIN_Z_DEFAULT),
    //--------------------------

    // @Param: FTLQR_P11
    // @DisplayName: FT-LQR controller P matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P11", 94, MultiControl, _ftlqrConfig.P[0], FTLQR_CONFIG_P11_DEFAULT),

    // @Param: FTLQR_P22
    // @DisplayName: FT-LQR controller P matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P22", 95, MultiControl, _ftlqrConfig.P[1], FTLQR_CONFIG_P22_DEFAULT),

    // @Param: FTLQR_P33
    // @DisplayName: FT-LQR controller P matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P33", 96, MultiControl, _ftlqrConfig.P[2], FTLQR_CONFIG_P33_DEFAULT),

    // @Param: FTLQR_P44
    // @DisplayName: FT-LQR controller P matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P44", 97, MultiControl, _ftlqrConfig.P[3], FTLQR_CONFIG_P44_DEFAULT),

    // @Param: FTLQR_P55
    // @DisplayName: FT-LQR controller P matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P55", 98, MultiControl, _ftlqrConfig.P[4], FTLQR_CONFIG_P55_DEFAULT),

    // @Param: FTLQR_P66
    // @DisplayName: FT-LQR controller P matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P66", 99, MultiControl, _ftlqrConfig.P[5], FTLQR_CONFIG_P66_DEFAULT),

    // @Param: FTLQR_Q11
    // @DisplayName: FT-LQR controller Q matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q11", 100, MultiControl, _ftlqrConfig.Q[0], FTLQR_CONFIG_Q11_DEFAULT),

    // @Param: FTLQR_Q22
    // @DisplayName: FT-LQR controller Q matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q22", 101, MultiControl, _ftlqrConfig.Q[1], FTLQR_CONFIG_Q22_DEFAULT),

    // @Param: FTLQR_Q33
    // @DisplayName: FT-LQR controller Q matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q33", 102, MultiControl, _ftlqrConfig.Q[2], FTLQR_CONFIG_Q33_DEFAULT),

    // @Param: FTLQR_Q44
    // @DisplayName: FT-LQR controller Q matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q44", 103, MultiControl, _ftlqrConfig.Q[3], FTLQR_CONFIG_Q44_DEFAULT),

    // @Param: FTLQR_Q55
    // @DisplayName: FT-LQR controller Q matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q55", 104, MultiControl, _ftlqrConfig.Q[4], FTLQR_CONFIG_Q55_DEFAULT),

    // @Param: FTLQR_Q66
    // @DisplayName: FT-LQR controller Q matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q66", 105, MultiControl, _ftlqrConfig.Q[5], FTLQR_CONFIG_Q66_DEFAULT),

    // @Param: FTLQR_R11
    // @DisplayName: FT-LQR controller R matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R11", 106, MultiControl, _ftlqrConfig.R[0], FTLQR_CONFIG_R11_DEFAULT),

    // @Param: FTLQR_R22
    // @DisplayName: FT-LQR controller R matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R22", 107, MultiControl, _ftlqrConfig.R[1], FTLQR_CONFIG_R22_DEFAULT),

    // @Param: FTLQR_R33
    // @DisplayName: FT-LQR controller R matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R33", 108, MultiControl, _ftlqrConfig.R[2], FTLQR_CONFIG_R33_DEFAULT),

    // @Param: FTLQR_R44
    // @DisplayName: FT-LQR controller R matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R44", 109, MultiControl, _ftlqrConfig.R[3], FTLQR_CONFIG_R44_DEFAULT),

    // @Param: FTLQR_R55
    // @DisplayName: FT-LQR controller R matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R55", 110, MultiControl, _ftlqrConfig.R[4], FTLQR_CONFIG_R55_DEFAULT),

    // @Param: FTLQR_R66
    // @DisplayName: FT-LQR controller R matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R66", 111, MultiControl, _ftlqrConfig.R[5], FTLQR_CONFIG_R66_DEFAULT),

    // @Param: FTLQR_R77
    // @DisplayName: FT-LQR controller R matrix (7,7)
    // @Description: Element (7,7) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R77", 112, MultiControl, _ftlqrConfig.R[6], FTLQR_CONFIG_R77_DEFAULT),

    // @Param: FTLQR_R88
    // @DisplayName: FT-LQR controller R matrix (8,8)
    // @Description: Element (8,8) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R88", 113, MultiControl, _ftlqrConfig.R[7], FTLQR_CONFIG_R88_DEFAULT),

    // @Param: FTLQR_Ef1
    // @DisplayName: FT-LQR controller Ef matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef1", 114, MultiControl, _ftlqrConfig.Ef[0], FTLQR_CONFIG_EF1_DEFAULT),

    // @Param: FTLQR_Ef2
    // @DisplayName: FT-LQR controller Ef matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef2", 115, MultiControl, _ftlqrConfig.Ef[1], FTLQR_CONFIG_EF2_DEFAULT),

    // @Param: FTLQR_Ef3
    // @DisplayName: FT-LQR controller Ef matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef3", 116, MultiControl, _ftlqrConfig.Ef[2], FTLQR_CONFIG_EF3_DEFAULT),

    // @Param: FTLQR_Ef4
    // @DisplayName: FT-LQR controller Ef matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef4", 117, MultiControl, _ftlqrConfig.Ef[3], FTLQR_CONFIG_EF4_DEFAULT),

    // @Param: FTLQR_Ef5
    // @DisplayName: FT-LQR controller Ef matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef5", 118, MultiControl, _ftlqrConfig.Ef[4], FTLQR_CONFIG_EF5_DEFAULT),

    // @Param: FTLQR_Ef6
    // @DisplayName: FT-LQR controller Ef matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef6", 119, MultiControl, _ftlqrConfig.Ef[5], FTLQR_CONFIG_EF6_DEFAULT),

    // @Param: FTLQR_Eg1
    // @DisplayName: FT-LQR controller Eg matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg1", 120, MultiControl, _ftlqrConfig.Eg[0], FTLQR_CONFIG_EG1_DEFAULT),

    // @Param: FTLQR_Eg2
    // @DisplayName: FT-LQR controller Eg matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg2", 121, MultiControl, _ftlqrConfig.Eg[1], FTLQR_CONFIG_EG2_DEFAULT),

    // @Param: FTLQR_Eg3
    // @DisplayName: FT-LQR controller Eg matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg3", 122, MultiControl, _ftlqrConfig.Eg[2], FTLQR_CONFIG_EG3_DEFAULT),

    // @Param: FTLQR_Eg4
    // @DisplayName: FT-LQR controller Eg matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg4", 123, MultiControl, _ftlqrConfig.Eg[3], FTLQR_CONFIG_EG4_DEFAULT),

    // @Param: FTLQR_Eg5
    // @DisplayName: FT-LQR controller Eg matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg5", 124, MultiControl, _ftlqrConfig.Eg[4], FTLQR_CONFIG_EG5_DEFAULT),

    // @Param: FTLQR_Eg6
    // @DisplayName: FT-LQR controller Eg matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg6", 125, MultiControl, _ftlqrConfig.Eg[5], FTLQR_CONFIG_EG6_DEFAULT),

    // @Param: FTLQR_Eg7
    // @DisplayName: FT-LQR controller Eg matrix (7)
    // @Description: Element (7) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg7", 126, MultiControl, _ftlqrConfig.Eg[6], FTLQR_CONFIG_EG7_DEFAULT),

    // @Param: FTLQR_Eg8
    // @DisplayName: FT-LQR controller Eg matrix (8)
    // @Description: Element (8) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg8", 127, MultiControl, _ftlqrConfig.Eg[7], FTLQR_CONFIG_EG8_DEFAULT),

    // @Param: FTLQR_H1
    // @DisplayName: FT-LQR controller H matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H1", 128, MultiControl, _ftlqrConfig.H[0], FTLQR_CONFIG_H1_DEFAULT),

    // @Param: FTLQR_H2
    // @DisplayName: FT-LQR controller H matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H2", 129, MultiControl, _ftlqrConfig.H[1], FTLQR_CONFIG_H2_DEFAULT),

    // @Param: FTLQR_H3
    // @DisplayName: FT-LQR controller H matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H3", 130, MultiControl, _ftlqrConfig.H[2], FTLQR_CONFIG_H3_DEFAULT),

    // @Param: FTLQR_H4
    // @DisplayName: FT-LQR controller H matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H4", 131, MultiControl, _ftlqrConfig.H[3], FTLQR_CONFIG_H4_DEFAULT),

    // @Param: FTLQR_H5
    // @DisplayName: FT-LQR controller H matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H5", 132, MultiControl, _ftlqrConfig.H[4], FTLQR_CONFIG_H5_DEFAULT),

    // @Param: FTLQR_H6
    // @DisplayName: FT-LQR controller H matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H6", 133, MultiControl, _ftlqrConfig.H[5], FTLQR_CONFIG_H6_DEFAULT),

    // @Param: FTLQR_MU
    // @DisplayName: FT-LQR controller mu value
    // @Description: Mu value of FT-LQR controller 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_MU", 134, MultiControl, _ftlqrConfig.mu, FTLQR_CONFIG_MU_DEFAULT),

    // @Param: FTLQR_ALP
    // @DisplayName: FT-LQR controller alpha value
    // @Description: Alpha value of FT-LQR controller 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_ALP", 135, MultiControl, _ftlqrConfig.alpha, FTLQR_CONFIG_ALPHA_DEFAULT),
    
    // @Param: CA_WM
    // @DisplayName: Control allocation maneuverability weight
    // @Description: Weight to ponderate maneuverability over attitude following for control allocation algorithm
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("CA_WM", 136, MultiControl, _caConfig.Wm, CA_WM_DEFAULT),
    
    // @Param: CA_WA
    // @DisplayName: Control allocation attitude following weight
    // @Description: Weight to ponderate attitude following over maneuverability for control allocation algorithm
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("CA_WA", 137, MultiControl, _caConfig.Wa, CA_WA_DEFAULT),

    AP_GROUPEND
};

MultiControl::MultiControl(AP_AHRS_View &ahrs_other) : 
    // Links AHRS objects
    _ahrs(ahrs_other)
{
    AP_Param::setup_object_defaults(this, var_info);
}

MultiControl::~MultiControl()
{
}

    // Members
bool MultiControl::init()
{
    ///////////////////////
    /* Generic variables */

    // Load variables from EEPROM
    //AP_Param::load_all();

    // Control time step
    this->_controlTimeStep = 0.0025;

    // Weight vector
    this->_weightVector << 0.0, 0.0, this->_mass*9.81;

    // Calculates Mf and Mt
    this->_Mf.resize(3,this->_numberOfRotors);
    this->_Mt.resize(3,this->_numberOfRotors);
    this->_pinvMt.resize(this->_numberOfRotors,3);

    double liftCoeff = (double) this->_rotorLiftCoeff;
    double dragCoeff = (double) this->_rotorDragCoeff;
    Eigen::Vector3d rotorPosition;
    Eigen::Vector3d rotorOrientation;
    Eigen::Vector3d rotorDirection;

    for(int it=0;it<this->_numberOfRotors;it++){
        rotorPosition << (double) this->_rotorPosition[it][0], (double) this->_rotorPosition[it][1], (double) this->_rotorPosition[it][2];
        this->_Mf.col(it) = liftCoeff*rotorPosition;
        rotorOrientation << (double) this->_rotorOrientation[it][0], (double) this->_rotorOrientation[it][1], (double) this->_rotorOrientation[it][2];
        this->_Mt.col(it) = liftCoeff*rotorPosition.cross(rotorOrientation)-dragCoeff*((double) this->_rotorDirection[it])*rotorOrientation;
    }

    // Calculates pseudo-inverse of Mt
    this->_pinvMt = this->_Mt.completeOrthogonalDecomposition().pseudoInverse();

    // Calculates nullMt
    /* Do a singular value decomposition on the matrix */
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->_Mt, Eigen::ComputeFullV);
    /* Get the V matrix */
    this->_nullMt.resize((int)svd.matrixV().rows(), (int)svd.matrixV().cols());
    this->_nullMt = svd.matrixV();

    // Groups inertia values
    Vector3f mI(this->_momentsOfInertia[0],this->_momentsOfInertia[1],this->_momentsOfInertia[2]);
    Vector3f pI(this->_productsOfInertia[0],this->_productsOfInertia[1],this->_productsOfInertia[2]);
    this->_inertia << (double) mI.x, (double) pI.x, (double) pI.y,
                      (double) pI.x, (double) mI.y, (double) pI.z,
                      (double) pI.y, (double) pI.z, (double) mI.z;  

    // Maximum , minimum and operational squared speeds    
    this->_maxSpeedsSquared = (double) this->_rotorMaxSpeed*this->_rotorMaxSpeed;
    this->_minSpeedsSquared = (double) this->_rotorMinSpeed*this->_rotorMinSpeed;
    this->_opSquared = (double) 0.5*(this->_maxSpeedsSquared+this->_minSpeedsSquared);
    
    this->_dummyState.acceleration.setZero();
    this->_dummyState.position.setZero();
    this->_dummyState.velocity.setZero();
    this->_dummyState.yaw = 0.0;

    ////////////////////
    /* FT-LQR related */

    // Calculates C
    this->_ftLQRConst.C.resize(3, this->_numberOfRotors);
    this->_ftLQRConst.C = -this->_inertia.inverse()*this->_Mt;

    ///////////////////////////
    /* Position PIDD related */

    // Initiate PIDD gains
    this->_piddConst.Kp << (double) this->_piddKp[0], (double) this->_piddKp[1], (double) this->_piddKp[2];
    this->_piddConst.Ki << (double) this->_piddKi[0], (double) this->_piddKi[1], (double) this->_piddKi[2];
    this->_piddConst.Kd << (double) this->_piddKd[0], (double) this->_piddKd[1], (double) this->_piddKd[2];
    this->_piddConst.Kdd << (double) this->_piddKdd[0], (double) this->_piddKdd[1], (double) this->_piddKdd[2];

    // Initiate PIDD integral error
    this->_pidd.iError = Eigen::Vector3d::Zero();

    ///////////////////////////////
    /* Atitude reference related */

    // Calculates attRefAux
    Eigen::MatrixXd auxAtt(this->_nullMt.cols(),this->_nullMt.cols());
    this->_attRefAux.resize(this->_nullMt.cols(),this->_nullMt.rows());
    auxAtt = this->_nullMt.transpose()*(((double)this->_caConfig.Wm)*this->_nullMt+this->_Mf.transpose()*((double)this->_caConfig.Wa)*this->_Mf*this->_nullMt);
    this->_attRefAux = auxAtt.colPivHouseholderQr().solve(this->_nullMt.transpose());

    // Auxiliary variables
    this->_v.resize(this->_attRefAux.rows(),1);
    this->_omegaSquared.resize(this->_numberOfRotors,1);
    this->_qyd.x() = 0.0;
    this->_qyd.y() = 0.0;
    this->_attRefAux2.resize(this->_Mf.cols(),1);

    return true;
};

bool MultiControl::updateStates(PolyNavigation::state desiredState){
    // Update desired states from navigation
    this->_desiredPosition << (double)desiredState.position.x(), (double)desiredState.position.y(), (double)desiredState.position.z();
    this->_desiredVelocity << (double)desiredState.velocity.x(), (double)desiredState.velocity.y(), (double)desiredState.velocity.z();
    this->_desiredAcceleration << (double)desiredState.acceleration.x(), (double)desiredState.acceleration.y(), (double)desiredState.acceleration.z();
    this->_desiredYaw = (double) desiredState.yaw;

    // Update current states from AHRS
    bool returnState = true;
    if(this->_ahrs.get_relative_position_NED_home(this->_vectorAux)){
        this->_currentPosition << (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    } else { returnState = false;}
    if(this->_ahrs.get_velocity_NED(this->_vectorAux)){
        this->_currentVelocity << (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    } else { returnState = false;}
    this->_vectorAux = this->_ahrs.get_accel_ef_blended();
    this->_currentAcceleration <<  (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    // Update current attitude and angular velocity from AHRS converting from NED/NED to ENU/ENU
    this->_ahrs.get_quat_body_to_ned(this->_quat);
    this->_currentAttitude.w() = (double) -SQRT2_DIV2*(this->_quat.q1+this->_quat.q4);
    this->_currentAttitude.x() = (double) -SQRT2_DIV2*(this->_quat.q2+this->_quat.q3);
    this->_currentAttitude.y() = (double) SQRT2_DIV2*(this->_quat.q3-this->_quat.q2);
    this->_currentAttitude.z() = (double) SQRT2_DIV2*(this->_quat.q4-this->_quat.q1);
    this->_currentAttitude.normalize();
    this->_vectorAux = _ahrs.get_gyro();
    this->_currentAngularVelocity << (double) this->_vectorAux.x, (double) -this->_vectorAux.y, (double) -this->_vectorAux.z;

    // Update control time step
    double thisCall = (double) AP_HAL::millis()/1000.0;
    this->_controlTimeStep = 0.5*(this->_controlTimeStep+thisCall-this->_lastCall);
    this->_lastCall = thisCall;

    return returnState;
};

bool MultiControl::positionControl(){
    this->_pidd.error = (this->_desiredPosition - this->_currentPosition).array(); // Position error
    this->_pidd.iError = this->_pidd.iError + this->_pidd.error * this->_controlTimeStep; // "Integral" of the error
    this->_pidd.derror = (this->_desiredVelocity - this->_currentVelocity).array(); // Derivative of the error
    this->_pidd.dderror = (this->_desiredAcceleration - this->_currentAcceleration).array(); // Second derivative of the error
    this->_desiredForce = (this->_piddConst.Kp*this->_pidd.error+
                            this->_piddConst.Ki*this->_pidd.iError+
                            this->_piddConst.Kd*this->_pidd.derror+
                            this->_piddConst.Kdd*this->_pidd.dderror).matrix()+this->_weightVector;

    return true;
};

bool MultiControl::attitudeReference(){
    this->_qyd.w() = (double) cos(this->_desiredYaw/2.0);
    this->_qyd.z() = (double) sin(this->_desiredYaw/2.0);
    matrixBtoA(this->_qyd,this->_Qyd);
    this->_invQyd = this->_Qyd.transpose();
    this->_Tcd = this->_invQyd*this->_desiredForce;
    this->_attRefAux2 << (((double)this->_caConfig.Wm)*this->_opSquared+(this->_Mf.transpose()*((double)this->_caConfig.Wa)*this->_Tcd).array()).matrix();

    this->_v << this->_attRefAux*this->_attRefAux2;

    this->_omegaSquared = this->_nullMt*this->_v;

    for(int it=0;it<this->_omegaSquared.size();it++){
        if(this->_omegaSquared(it)<this->_minSpeedsSquared){
            this->_omegaSquared(it) = this->_minSpeedsSquared;
        }
        else if(this->_omegaSquared(it)>this->_maxSpeedsSquared){
            this->_omegaSquared(it) = this->_maxSpeedsSquared;
        }
    }

    this->_Tc << this->_Mf*this->_omegaSquared;
    if(this->_desiredForce(3)<0.0){
        this->_desiredForce(3) = -this->_desiredForce(2);
    }

    double thetaCB = 0.0;
    this->_Ta = this->_Qyd*this->_Tc;
    if(this->_Ta.norm()<=1e-9){
        thetaCB = acos(this->_desiredForce(2)/this->_desiredForce.norm());        
        this->_vCB << 0.0, 0.0, 1.0;
        this->_vCB = (this->_vCB.transpose()).cross(this->_desiredForce.normalized());
    }
    else{
        thetaCB = acos((double) (this->_Ta.transpose()*this->_desiredForce)/(this->_desiredForce.norm()*this->_Ta.norm()));
        this->_vCB << this->_Ta.normalized();
        this->_vCB = this->_vCB.cross(this->_desiredForce.normalized());
    }
    // thetaCB = real(thetaCB);
    // %thetaDegress = thetaCB*180/pi
    this->_qCB.w() = (double) cos(thetaCB/2);
    this->_qCB.x() = (double) this->_vCB(0)*sin(thetaCB/2);
    this->_qCB.y() = (double) this->_vCB(1)*sin(thetaCB/2);
    this->_qCB.z() = (double) this->_vCB(2)*sin(thetaCB/2);
    // compound rotation -> desired attitude
    this->_desiredAttitude.w() = this->_qCB.w()*this->_qyd.w()-this->_qCB.x()*this->_qyd.x()-this->_qCB.y()*this->_qyd.y()-this->_qCB.z()*this->_qyd.z();
    this->_desiredAttitude.x() = this->_qCB.w()*this->_qyd.x()+this->_qCB.x()*this->_qyd.w()+this->_qCB.y()*this->_qyd.z()-this->_qCB.z()*this->_qyd.y();
    this->_desiredAttitude.y() = this->_qCB.w()*this->_qyd.y()-this->_qCB.x()*this->_qyd.z()+this->_qCB.y()*this->_qyd.w()+this->_qCB.z()*this->_qyd.x();
    this->_desiredAttitude.z() = this->_qCB.w()*this->_qyd.z()+this->_qCB.x()*this->_qyd.y()-this->_qCB.y()*this->_qyd.x()+this->_qCB.z()*this->_qyd.w();
            
    this->_desiredAttitude.normalize();

    return true;
};

bool MultiControl::attitudeFTLQRControl(){
    return false;
};

bool MultiControl::controlAllocation(){
    return false;
};

/* Auxiliary private members */
void MultiControl::matrixBtoA(const Eigen::Quaterniond& quaternion, Eigen::Ref<Eigen::Matrix3d> transformationBA){
    double qww = quaternion.w()*quaternion.w();
    double qxx = quaternion.x()*quaternion.x();
    double qyy = quaternion.y()*quaternion.y();
    double qzz = quaternion.z()*quaternion.z();
    double q2xy = 2*quaternion.x()*quaternion.y();
    double q2wz = 2*quaternion.w()*quaternion.z();
    double q2wy = 2*quaternion.w()*quaternion.y();
    double q2xz = 2*quaternion.x()*quaternion.z();
    double q2yz = 2*quaternion.y()*quaternion.z();
    double q2wx = 2*quaternion.w()*quaternion.x();

    transformationBA(0,0) = qww+qxx-qyy-qzz;
    transformationBA(0,1) = q2xy-q2wz;
    transformationBA(0,2) = q2wy+q2xz;
    transformationBA(1,0) = q2xy+q2wz;
    transformationBA(1,1) = qww-qxx+qyy-qzz;
    transformationBA(1,2) = q2yz-q2wx;
    transformationBA(2,0) = q2xz-q2wy;
    transformationBA(2,1) = q2yz+q2wx;
    transformationBA(2,2) = qww-qxx-qyy+qzz;
};


void MultiControl::swapReferenceFrames(const Eigen::Quaterniond &quatIn, Quaternion &quatOut){
    quatOut.q1 = (double) -SQRT2_DIV2*(quatIn.w()+quatIn.z());
    quatOut.q2 = (double) -SQRT2_DIV2*(quatIn.x()+quatIn.y());
    quatOut.q3 = (double) SQRT2_DIV2*(quatIn.y()-quatIn.x());
    quatOut.q4 = (double) SQRT2_DIV2*(quatIn.z()-quatIn.w());
    quatOut.normalize();
};

/* Auxiliary public members */ 

    Vector3f MultiControl::toEuler(Eigen::Quaterniond quat){
        Vector3f eulerAngles;
        eulerAngles.x = atan2(2*quat.y()*quat.z()+2*quat.w()*quat.x(),quat.z()*quat.z()-quat.y()*quat.y()-quat.x()*quat.x()+quat.w()*quat.w());
        eulerAngles.y = -asin(2*quat.x()*quat.z()-2*quat.w()*quat.y());
        eulerAngles.z = atan2(2*quat.x()*quat.y()+2*quat.w()*quat.z(),quat.x()*quat.x()+quat.w()*quat.w()-quat.z()*quat.z()-quat.y()*quat.y());
        return eulerAngles;
    };

    Vector3f MultiControl::toEuler(Quaternion quat){
        Vector3f eulerAngles;
        eulerAngles.x = atan2(2*quat.q3*quat.q4+2*quat.q1*quat.q2,quat.q4*quat.q4-quat.q3*quat.q3-quat.q2*quat.q2+quat.q1*quat.q1);
        eulerAngles.y = -asin(2*quat.q2*quat.q4-2*quat.q1*quat.q3);
        eulerAngles.z = atan2(2*quat.q2*quat.q3+2*quat.q1*quat.q4,quat.q2*quat.q2+quat.q1*quat.q1-quat.q4*quat.q4-quat.q3*quat.q3);
        return eulerAngles;
    };

    // Variable access
    void MultiControl::desiredAttitude(Quaternion &quat){
        quat.q1 = this->_desiredAttitude.w();
        quat.q2 = this->_desiredAttitude.x();
        quat.q3 = this->_desiredAttitude.y();
        quat.q4 = this->_desiredAttitude.z();
    };

    Vector3f MultiControl::desiredAttitude(){
        return toEuler(this->_desiredAttitude);
    };

    void MultiControl::desiredAttitudeNED(Quaternion &quat){
        swapReferenceFrames(this->_desiredAttitude,quat);
    };

    Vector3f MultiControl::desiredAttitudeNED(){
        Quaternion quat;
        swapReferenceFrames(this->_desiredAttitude,quat);
        return toEuler(quat);
    };

    Vector3f MultiControl::currentPosition(){
        Vector3f pos;
        pos.x = this->_currentPosition(0);
        pos.y = this->_currentPosition(1);
        pos.z = this->_currentPosition(2);
        return pos;
    };

    Vector3f MultiControl::currentPositionNED(){
        Vector3f pos;
        pos.x = this->_currentPosition(1);
        pos.y = this->_currentPosition(0);
        pos.z = -this->_currentPosition(2);
        return pos;
    };

    Vector3f MultiControl::currentVelocity(){
        Vector3f vel;
        vel.x = this->_currentPosition(0);
        vel.y = this->_currentPosition(1);
        vel.z = this->_currentPosition(2);
        return vel;
    };

    Vector3f MultiControl::currentVelocityNED(){
        Vector3f vel;
        vel.x = this->_currentPosition(1);
        vel.y = this->_currentPosition(0);
        vel.z = -this->_currentPosition(2);
        return vel;
    };

    Vector3f MultiControl::currentAcceleration(){
        Vector3f acc;
        acc.x = this->_currentPosition(0);
        acc.y = this->_currentPosition(1);
        acc.z = this->_currentPosition(2);
        return acc;
    };

    Vector3f MultiControl::currentAccelerationNED(){
        Vector3f acc;
        acc.x = this->_currentPosition(1);
        acc.y = this->_currentPosition(0);
        acc.z = -this->_currentPosition(2);
        return acc;
    };

    void MultiControl::currentAttitude(Quaternion &quat){
        quat.q1 = this->_currentAttitude.w();
        quat.q2 = this->_currentAttitude.x();
        quat.q3 = this->_currentAttitude.y();
        quat.q4 = this->_currentAttitude.z();
    };

    Vector3f MultiControl::currentAttitude(){
        return toEuler(this->_currentAttitude);
    };

    void MultiControl::currentAttitudeNED(Quaternion &quat){
        swapReferenceFrames(this->_currentAttitude,quat);
    };

    Vector3f MultiControl::currentAttitudeNED(){
        Quaternion quat;
        swapReferenceFrames(this->_currentAttitude,quat);
        return toEuler(quat);
    };

    Vector3f MultiControl::currentAngularVelocity(){
        Vector3f vel;
        vel.x = this->_currentAngularVelocity(0);
        vel.y = this->_currentAngularVelocity(1);
        vel.z = this->_currentAngularVelocity(2);
        return vel;
    };

    Vector3f MultiControl::currentAngularVelocityNED(){
        Vector3f vel;
        vel.x = this->_currentAngularVelocity(0);
        vel.y = -this->_currentAngularVelocity(1);
        vel.z = -this->_currentAngularVelocity(2);
        return vel;
    };

    float* MultiControl::currentRotorSpeeds(){
        float *dummy = new float;
        return dummy;
    };