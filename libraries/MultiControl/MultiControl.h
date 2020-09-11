/// @file	MultiControl.h
/// @brief	Library for multicopter control

#ifndef MULTICONTROL_H
#define MULTICONTROL_H

#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Param/AP_Param.h>
#include <PolyNavigation/PolyNavigation.h>
#include </usr/include/Eigen/Eigen>
#include </usr/include/Eigen/unsupported/Eigen/MatrixFunctions>
#include <Eigen/Sparse>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#define SQRT2_DIV2 0.707106781

#define NUMBER_OF_ROTORS 8

////////////////////////////////////////////////
/* Position PIDD configuration default values */
#define PIDD_KP_X 20.0
#define PIDD_KP_Y 20.0
#define PIDD_KP_Z 20.0
#define PIDD_KI_X 4.0
#define PIDD_KI_Y 4.0
#define PIDD_KI_Z 4.0
#define PIDD_KD_X 15.0
#define PIDD_KD_Y 15.0
#define PIDD_KD_Z 15.0
#define PIDD_KDD_X 1.0
#define PIDD_KDD_Y 1.0
#define PIDD_KDD_Z 1.0

class MultiControl
{
    //private
public:
    ///////////////////////////////////////////
    /* Variables to define on initialization */
    std::ofstream dumpfile;
    Eigen::Quaterniond auxQuaternion;

    /* Multicopter configuration */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _rotorPosition;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _rotorOrientation;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _rotorDirection;

    // Generic
    AP_AHRS &_ahrs;
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mf; // Matrix of forces calculated from rotor positions
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mt; // Matrix of torques calculated from rotor positions and torques
    Eigen::Matrix<double, Eigen::Dynamic, 3> _pinvMt; // pseudo-inverse of Mt
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _nullMt; // Null-space of Mt
    Eigen::Matrix3d _inertia; // body inertia matrix
    double _maxSpeedsSquared; // Maximum rotor speeds squared
    double _minSpeedsSquared; // Minimum rotor speeds squared
    double _opSquared; // Midpoint operational speeds squared
    Eigen::Vector3d _weightVector; // Weight vector in the opposite direction of gravitational aceleration
    Eigen::Vector3d _velocityFilterGain; //Gain for the angular velocity filter

    // FT-LQR related
    struct ftlqrRelatedConst {
        Eigen::Matrix<double, 6, NUMBER_OF_ROTORS> ssB;
        Eigen::SparseMatrix<double> Q;
        Eigen::SparseMatrix<double> R;
        Eigen::Matrix<double, 6, 1> Ef;
    } _ftLQRConst;     

    // Position PIDD related
    struct piddRelatedConst {
        Eigen::Array3d Kp;
        Eigen::Array3d Ki;
        Eigen::Array3d Kd;
        Eigen::Array3d Kdd;
    } _piddConst;  

    PolyNavigation::state _dummyState;
    /////////////////////////////////////////
    /* Variables updated at each iteration */

    // Desired state
    Eigen::Vector3d _desiredPosition;
    Eigen::Vector3d _desiredVelocity;
    Eigen::Vector3d _desiredAcceleration;
    double _desiredYaw;
    Eigen::Quaterniond _desiredAttitude;
    Eigen::Vector3d _desiredForce;
    Eigen::Vector3d _desiredTorque;
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> _desiredRotorSpeeds;
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> _desiredRotorVoltages;

    // Current state
    Eigen::Vector3d _currentPosition;
    Eigen::Vector3d _currentVelocity;
    Eigen::Vector3d _currentAcceleration;
    Eigen::Quaterniond _currentAttitude;
    Eigen::Vector3d _currentAngularVelocity;
    Eigen::Matrix<double, NUMBER_OF_ROTORS, 1> _currentRotorSpeeds;

    // Other
    double _lastCall;
    double _controlTimeStep;
    double _measuredTimeStep;
    Eigen::Quaterniond _quaternionError;

    // Velocity filter related
    struct velocityFilterRelated {
        Eigen::Vector3d Wbe;
        Eigen::Vector3d previousAngularVelocity;
        Eigen::Vector3d desiredAngularVelocity;
        Eigen::Vector3d desiredAngularAcceleration;
    } _velFilter;

    // FT-LQR related
    struct ftlqrRelated {
        Eigen::Matrix<double,6,6> P;
        Eigen::SparseMatrix<double> left;
        Eigen::SparseMatrix<double> right;
        Eigen::Matrix<double, NUMBER_OF_ROTORS, 1> gainRotorSpeeds;
        Eigen::Vector3d desiredGainAngularAcceleration;
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
    Eigen::Vector3d _Tcd;
    Eigen::MatrixXd _v;
    Eigen::MatrixXd _omegaSquared;
    Eigen::Vector3d _Tc;
    Eigen::Vector3d _vCB;    
    Eigen::Vector3d _Ta;
    Eigen::Quaterniond _qCB;

    /* members */
    void matrixBtoA(const Eigen::Quaterniond& quaternion, Eigen::Ref<Eigen::Matrix3d> transformationBA);
    void swapReferenceFrames(const Eigen::Quaterniond &quatIn, Quaternion &quatOut);
    void c2d(const Eigen::Ref<const Eigen::MatrixXd>& Ac,const Eigen::Ref<const Eigen::MatrixXd>& Bc, double ts, Eigen::Ref<Eigen::MatrixXd> Ad, Eigen::Ref<Eigen::MatrixXd> Bd);
    void gainRLQR(Eigen::Ref<Eigen::MatrixXd> F, Eigen::Ref<Eigen::MatrixXd> G, Eigen::Ref<Eigen::MatrixXd> K);
public:

    static const struct AP_Param::GroupInfo var_info[];
    
    // Constructor
    MultiControl();
    
    /* Do not allow copies */
    MultiControl(const MultiControl &other) = delete;
    MultiControl &operator=(const MultiControl&) = delete;

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

    Vector3f toEuler(Eigen::Quaterniond quat);
    Vector3f toEuler(Quaternion quat);
    
    // Variable access
    PolyNavigation::state dummyState(){return _dummyState;};
    void desiredAttitude(Quaternion &quat);
    Vector3f desiredAttitude();
    void desiredAttitudeNED(Quaternion &quat);
    Vector3f desiredAttitudeNED();
    Vector3f currentPosition();
    Vector3f currentPositionNED();
    Vector3f currentVelocity();
    Vector3f currentVelocityNED();
    Vector3f currentAcceleration();
    Vector3f currentAccelerationNED();
    void currentAttitude(Quaternion &quat);
    Vector3f currentAttitude();
    void currentAttitudeNED(Quaternion &quat);
    Vector3f currentAttitudeNED();
    Vector3f currentAngularVelocity();
    Vector3f currentAngularVelocityNED();
    float* currentRotorSpeeds();
    float* desiredRotorSpeeds();
    float* desiredRotorVoltages();
    double controlTimeStep(){return _controlTimeStep;};
    double measuredTimeStep(){return _measuredTimeStep;};

protected:

    AP_Float _PIDD_KP_X;
    AP_Float _PIDD_KP_Y;
    AP_Float _PIDD_KP_Z;
    AP_Float _PIDD_KI_X;
    AP_Float _PIDD_KI_Y;
    AP_Float _PIDD_KI_Z;
    AP_Float _PIDD_KD_X;
    AP_Float _PIDD_KD_Y;
    AP_Float _PIDD_KD_Z;
    AP_Float _PIDD_KDD_X;
    AP_Float _PIDD_KDD_Y;
    AP_Float _PIDD_KDD_Z;
};
 
#endif