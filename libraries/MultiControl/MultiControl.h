/// @file	MultiControl.h
/// @brief	Library for multicopter control

#ifndef MULTICONTROL_H
#define MULTICONTROL_H

#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <PolyNavigation/PolyNavigation.h>
#include </usr/include/eigen3/Eigen/Eigen>
#include <cmath>
#include <string>
#include <iostream>

#define SQRT2_DIV2 0.707106781

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


//////////////////////////////////////
/* General variables default values */
#define VELOCITY_FILTER_GAIN_DEFAULT (0.0, 0.0, 0.9)

////////////////////////////////////////////////
/* Position PIDD configuration default values */
#define PIDD_KP_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KI_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KD_DEFAULT (1.0, 1.0, 1.0)
#define PIDD_KDD_DEFAULT (1.0, 1.0, 1.0)

/////////////////////////////////////////
/* FT-LQR configuration default values */
#define FTLQR_CONFIG_P11_DEFAULT 0.0
#define FTLQR_CONFIG_P22_DEFAULT 0.0
#define FTLQR_CONFIG_P33_DEFAULT 0.0
#define FTLQR_CONFIG_P44_DEFAULT 0.0
#define FTLQR_CONFIG_P55_DEFAULT 0.0
#define FTLQR_CONFIG_P66_DEFAULT 0.0

#define FTLQR_CONFIG_Q11_DEFAULT 0.0
#define FTLQR_CONFIG_Q22_DEFAULT 0.0
#define FTLQR_CONFIG_Q33_DEFAULT 0.0
#define FTLQR_CONFIG_Q44_DEFAULT 0.0
#define FTLQR_CONFIG_Q55_DEFAULT 0.0
#define FTLQR_CONFIG_Q66_DEFAULT 0.0

#define FTLQR_CONFIG_R11_DEFAULT 0.0
#define FTLQR_CONFIG_R22_DEFAULT 0.0
#define FTLQR_CONFIG_R33_DEFAULT 0.0
#define FTLQR_CONFIG_R44_DEFAULT 0.0
#define FTLQR_CONFIG_R55_DEFAULT 0.0
#define FTLQR_CONFIG_R66_DEFAULT 0.0
#define FTLQR_CONFIG_R77_DEFAULT 0.0
#define FTLQR_CONFIG_R88_DEFAULT 0.0

#define FTLQR_CONFIG_EF1_DEFAULT 0.0
#define FTLQR_CONFIG_EF2_DEFAULT 0.0
#define FTLQR_CONFIG_EF3_DEFAULT 0.0
#define FTLQR_CONFIG_EF4_DEFAULT 0.0
#define FTLQR_CONFIG_EF5_DEFAULT 0.0
#define FTLQR_CONFIG_EF6_DEFAULT 0.0

#define FTLQR_CONFIG_EG1_DEFAULT 0.0
#define FTLQR_CONFIG_EG2_DEFAULT 0.0
#define FTLQR_CONFIG_EG3_DEFAULT 0.0
#define FTLQR_CONFIG_EG4_DEFAULT 0.0
#define FTLQR_CONFIG_EG5_DEFAULT 0.0
#define FTLQR_CONFIG_EG6_DEFAULT 0.0
#define FTLQR_CONFIG_EG7_DEFAULT 0.0
#define FTLQR_CONFIG_EG8_DEFAULT 0.0

#define FTLQR_CONFIG_H1_DEFAULT 0.0
#define FTLQR_CONFIG_H2_DEFAULT 0.0
#define FTLQR_CONFIG_H3_DEFAULT 0.0
#define FTLQR_CONFIG_H4_DEFAULT 0.0
#define FTLQR_CONFIG_H5_DEFAULT 0.0
#define FTLQR_CONFIG_H6_DEFAULT 0.0

#define FTLQR_CONFIG_MU_DEFAULT 0.0
#define FTLQR_CONFIG_ALPHA_DEFAULT 0.0

//////////////////////////////////////////////////////////////////
/* Passive NMAC Control allocation configuration default values */
#define CA_WM_DEFAULT 1.0
#define CA_WA_DEFAULT 0.0

class MultiControl
{
private:
    ///////////////////////////////////////////
    /* Variables to define on initialization */

    // Generic
    AP_AHRS_View &_ahrs;
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mf; // Matrix of forces calculated from rotor positions
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mt; // Matrix of torques calculated from rotor positions and torques
    Eigen::Matrix<double, Eigen::Dynamic, 3> _pinvMt; // pseudo-inverse of Mt
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _nullMt; // Null-space of Mt
    Eigen::Matrix3d _inertia; // body inertia matrix
    double _maxSpeedsSquared; // Maximum rotor speeds squared
    double _minSpeedsSquared; // Minimum rotor speeds squared
    double _opSquared; // Midpoint operational speeds squared
    Eigen::Vector3d _weightVector; // Weight vector in the opposite direction of gravitational aceleration

    // FT-LQR related
    struct ftlqrRelatedConst {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
    } _ftLQRConst;   

    // Position PIDD related
    struct piddRelatedConst {
        Eigen::Array3d Kp;
        Eigen::Array3d Ki;
        Eigen::Array3d Kd;
        Eigen::Array3d Kdd;
    } _piddConst;  

    /////////////////////////////////////////
    /* Variables updated at each iteration */

    // Desired state
    Eigen::Vector3d _desiredPosition;
    Eigen::Vector3d _desiredVelocity;
    Eigen::Vector3d _desiredAcceleration;
    double _desiredYaw;
    Eigen::Quaterniond _desiredAttitude;

    // Current state
    Eigen::Vector3d _currentPosition;
    Eigen::Vector3d _currentVelocity;
    Eigen::Vector3d _currentAcceleration;
    Eigen::Quaterniond _currentAttitude;
    Eigen::Vector3d _currentAngularVelocity;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _currentRotorSpeeds;

    // Other
    double _lastCall;
    double _controlTimeStep;
    Eigen::Vector3d _desiredForce;
    Eigen::Quaterniond _quaternionError;
    struct velFilter {
        Eigen::Vector3d Wbe;
        Eigen::Vector3d angularVelocity;
        Eigen::Vector3d desiredAngularVelocity;
    } _velocityFilter;

    // FT-LQR related
    struct ftlqrRelated {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
    } _ftLQR;

    // Position PIDD related
    struct piddRelated {
        Eigen::Array3d iError;
        Eigen::Array3d error;
        Eigen::Array3d derror;
        Eigen::Array3d dderror;
    } _pidd;

    /////////////////////////
    /* Auxiliary variables */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _attRefAux; //Auxiliary variable for attitude reference calculation. Calculated on init to reduce computation time
    Eigen::MatrixXd _attRefAux2;
    Vector3f _vectorAux;
    Quaternion _quat;
    Eigen::Quaterniond _qyd;
    Eigen::Matrix3d _Qyd; 
    Eigen::Matrix3d _invQyd;   
    Eigen::Matrix3d _Tcd;
    Eigen::MatrixXd _v;
    Eigen::MatrixXd _omegaSquared;
    Eigen::Vector3d _Tc;
    Eigen::Vector3d _vCB;    
    Eigen::Vector3d _Ta;
    Eigen::Quaterniond _qCB;

    /* members */
    void matrixBtoA(const Eigen::Quaterniond& quaternion, Eigen::Ref<Eigen::Matrix3d> transformationBA);
public:
    // Constructor
    MultiControl(AP_AHRS_View &ahrs_other);
    
    // Destructor
    ~MultiControl();

    // Commands

    // Members
    bool init();
    bool updateStates(PolyNavigation::state desiredState);
    bool positionControl();
    bool attitudeReference();
    bool attitudeFTLQRControl();
    bool controlAllocation();
    
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

    AP_Vector3f _rotorPosition[8];
    AP_Vector3f _rotorOrientation[8];
    AP_Int8 _rotorDirection[8];

    ///////////////////////
    /* General variables */
    AP_Vector3f _velocityFilterGain;

    ////////////////////////////////
    /* Position PIDD configuration */
    AP_Vector3f _piddKp;
    AP_Vector3f _piddKi;
    AP_Vector3f _piddKd;
    AP_Vector3f _piddKdd;

    ////////////////////////////////
    /* FT-LQR configuration */
    struct ftlqrConfig
    {
        AP_Float P[6];
        AP_Float Q[6];
        AP_Float R[8];
        AP_Float Ef[6];
        AP_Float Eg[8];
        AP_Float H[6];
        AP_Float mu;
        AP_Float alpha;  
    } _ftlqrConfig;
    
    ///////////////////////////////////////////////////
    /* Passive NMAC Control allocation configuration */
    struct caConfig {
        AP_Float Wm;
        AP_Float Wa;
    } _caConfig;
    
};
 
#endif