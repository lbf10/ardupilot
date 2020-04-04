/// @file	MultiControl.h
/// @brief	Library for multicopter control

#ifndef MULTICONTROL_H
#define MULTICONTROL_H

#pragma once

#include <AP_Math/AP_Math.h>
#include </usr/include/eigen3/Eigen/Eigen>
#include <cmath>
#include <string>
#include <iostream>

//////////////////////////////////////////////
/* Multicopter configuration default values */
#define MASS_DEFAULT    6.015 
#define MOMENTS_OF_INERTIA_DEFAULT (0.3143978800, 0.3122127800, 0.5557912400)
#define PRODUCTS_OF_INERTIA_DEFAULT (0.0000861200, -0.0014397600, 0.0002368800)
#define MAIN_TRANSLATIONAL_FRICTION_DEFAULT (0.25, 0.25, 0.25)
#define NUMBER_OF_ROTORS_DEFAULT 8
#define ROTOR_INERTIA_DEFAULT 0.00047935
#define ROTOR_LIFT_COEFF_DEFAULT 6.97e-5
#define ROTOR_DRAG_COEFF_DEFAULT 1.033e-6
#define ROTOR_MAX_SPEED_DEFAULT 729.0
#define ROTOR_OPERATING_POINT_DEFAULT 340.0
#define ROTOR_MIN_SPEED_DEFAULT 0.0
#define ROTOR_RM_DEFAULT 0.0975
#define ROTOR_L_DEFAULT 0.0033
#define ROTOR_KT_DEFAULT 0.02498
#define ROTOR_KV_DEFAULT 340.0
#define ROTOR_IO_DEFAULT 0.6/10.0
#define ROTOR_MAX_VOLTAGE_DEFAULT 22.0

#define ROTOR1_POSITION_DEFAULT (0.34374, 0.34245, 0.0143)
#define ROTOR2_POSITION_DEFAULT (-0.341, 0.34213, 0.0143)
#define ROTOR3_POSITION_DEFAULT (-0.34068, -0.34262, 0.0143)
#define ROTOR4_POSITION_DEFAULT (0.34407, -0.34229, 0.0143)
#define ROTOR5_POSITION_DEFAULT (0.33898, 0.33769, 0.0913)
#define ROTOR6_POSITION_DEFAULT (-0.33624, 0.33736, 0.0913)
#define ROTOR7_POSITION_DEFAULT (-0.33591, -0.33785, 0.0913)
#define ROTOR8_POSITION_DEFAULT (0.3393, -0.33753, 0.0913)

#define ROTOR1_ORIENTATION_DEFAULT (-0.061628417, -0.061628417, 0.996194698)
#define ROTOR2_ORIENTATION_DEFAULT (0.061628417, -0.061628417, 0.996194698)
#define ROTOR3_ORIENTATION_DEFAULT (0.061628417, 0.061628417, 0.996194698)
#define ROTOR4_ORIENTATION_DEFAULT (-0.061628417, 0.061628417, 0.996194698)
#define ROTOR5_ORIENTATION_DEFAULT (-0.061628417, -0.061628417, 0.996194698)
#define ROTOR6_ORIENTATION_DEFAULT (0.061628417, -0.061628417, 0.996194698)
#define ROTOR7_ORIENTATION_DEFAULT (0.061628417, 0.061628417, 0.996194698)
#define ROTOR8_ORIENTATION_DEFAULT (-0.061628417, 0.061628417, 0.996194698)

#define ROTOR1_DIRECTION_DEFAULT 1
#define ROTOR2_DIRECTION_DEFAULT -1
#define ROTOR3_DIRECTION_DEFAULT 1
#define ROTOR4_DIRECTION_DEFAULT -1
#define ROTOR5_DIRECTION_DEFAULT -1
#define ROTOR6_DIRECTION_DEFAULT 1
#define ROTOR7_DIRECTION_DEFAULT -1
#define ROTOR8_DIRECTION_DEFAULT 1

////////////////////////////////////////////////
/* Position PIDD configuration default values */
#define PIDD_KP_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KI_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KD_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KDD_DEFAULT (1.0, 1.0, 1.0)

class MultiControl
{
private:
    /* data */
    Eigen::Matrix3f 
    /* members */

public:
    // Constructor
    MultiControl(/* args */);
    
    // Destructor
    ~MultiControl();

    // Commands

    // Members
    
    // Variable access

    // AP variables
    static const struct AP_Param::GroupInfo var_info[];
   
protected:
    ////////////////////////////////
    /* Multicopter configuration */
    AP_Float _mass;
    AP_Vector3f _momentsOfInertia;
    AP_Vector3f _productsOfInertia;
    AP_Vector3f _mainTranslationalFriction;
    AP_Int8 _numberOfRotors;
    AP_Float _rotorInertia;
    AP_Float _rotorLiftCoeff;
    AP_Float _rotorDragCoeff;
    AP_Float _rotorMaxSpeed;
    AP_Float _rotorOperatingPoint;
    AP_Float _rotorMinSpeed;
    AP_Float _rotorRm;
    AP_Float _rotorL;
    AP_Float _rotorKt;
    AP_Float _rotorKv;
    AP_Float _rotorIo;
    AP_Float _rotorMaxVoltage;

    AP_Vector3f _rotor1Position;
    AP_Vector3f _rotor2Position;
    AP_Vector3f _rotor3Position;
    AP_Vector3f _rotor4Position;
    AP_Vector3f _rotor5Position;
    AP_Vector3f _rotor6Position;
    AP_Vector3f _rotor7Position;
    AP_Vector3f _rotor8Position;

    AP_Vector3f _rotor1Orientation;
    AP_Vector3f _rotor2Orientation;
    AP_Vector3f _rotor3Orientation;
    AP_Vector3f _rotor4Orientation;
    AP_Vector3f _rotor5Orientation;
    AP_Vector3f _rotor6Orientation;
    AP_Vector3f _rotor7Orientation;
    AP_Vector3f _rotor8Orientation;

    AP_Int8 _rotor1Direction;
    AP_Int8 _rotor2Direction;
    AP_Int8 _rotor3Direction;
    AP_Int8 _rotor4Direction;
    AP_Int8 _rotor5Direction;
    AP_Int8 _rotor6Direction;
    AP_Int8 _rotor7Direction;
    AP_Int8 _rotor8Direction;

    ////////////////////////////////
    /* Position PIDD configuration */
    AP_Vector3f _piddKp;
    AP_Vector3f _piddKi;
    AP_Vector3f _piddKd;
    AP_Vector3f _piddKdd;
};
 
#endif