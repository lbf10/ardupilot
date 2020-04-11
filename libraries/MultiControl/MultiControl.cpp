# include "MultiControl.h"

MultiControl::MultiControl(AP_AHRS_View &ahrs_other) : 
    // Links AHRS objects
    _ahrs(ahrs_other)
{
}

MultiControl::~MultiControl()
{
}

    // Members
bool MultiControl::init()
{
    ///////////////////////
    /* Generic variables */

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
    Vector3f aux;

    for(int it=0;it<this->_numberOfRotors;it++){
        aux = this->_rotorPosition[it];
        rotorPosition << (double) aux.x, (double) aux.y, (double) aux.z;
        this->_Mf.col(it) = liftCoeff*rotorPosition;
        aux = this->_rotorOrientation[it];
        rotorOrientation << (double) aux.x, (double) aux.y, (double) aux.z;
        this->_Mt.col(it) = liftCoeff*rotorPosition.cross(rotorOrientation)-dragCoeff*((float) this->_rotorDirection[it])*rotorOrientation;
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
    Vector3f mI = this->_momentsOfInertia;
    Vector3f pI = this->_productsOfInertia;
    this->_inertia << (double) mI.x, (double) pI.x, (double) pI.y,
                      (double) pI.x, (double) mI.y, (double) pI.z,
                      (double) pI.y, (double) pI.z, (double) mI.z;  

    // Maximum , minimum and operational squared speeds    
    this->_maxSpeedsSquared = (double) this->_rotorMaxSpeed*this->_rotorMaxSpeed;
    this->_minSpeedsSquared = (double) this->_rotorMinSpeed*this->_rotorMinSpeed;
    this->_opSquared = (double) 0.5*(this->_maxSpeedsSquared+this->_minSpeedsSquared);
    
    ////////////////////
    /* FT-LQR related */

    // Calculates C
    this->_ftLQRConst.C.resize(3, this->_numberOfRotors);
    this->_ftLQRConst.C = -this->_inertia.inverse()*this->_Mt;

    ///////////////////////////
    /* Position PIDD related */

    // Initiate PIDD gains
    Vector3f piddAux = this->_piddKp;
    this->_piddConst.Kp << (double)piddAux.x, (double)piddAux.y, (double)piddAux.z;
    Vector3f piddAux = this->_piddKi;
    this->_piddConst.Ki << (double)piddAux.x, (double)piddAux.y, (double)piddAux.z;
    Vector3f piddAux = this->_piddKd;
    this->_piddConst.Kd << (double)piddAux.x, (double)piddAux.y, (double)piddAux.z;
    Vector3f piddAux = this->_piddKdd;
    this->_piddConst.Kdd << (double)piddAux.x, (double)piddAux.y, (double)piddAux.z;

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
    this->_qyd.x = 0.0;
    this->_qyd.y = 0.0;
    this->_attRefAux2.resize(this->_Mf.cols(),1);
};

bool MultiControl::updateStates(PolyNavigation::state desiredState){
    // Update desired states from navigation
    this->_desiredPosition << (double)desiredState.position.x, (double)desiredState.position.y, (double)desiredState.position.z;
    this->_desiredVelocity << (double)desiredState.velocity.x, (double)desiredState.velocity.y, (double)desiredState.velocity.z;
    this->_desiredAcceleration << (double)desiredState.acceleration.x, (double)desiredState.acceleration.y, (double)desiredState.acceleration.z;
    this->_desiredYaw = (double) desiredState.yaw;

    // Update current states from AHRS
    this->_ahrs.get_relative_position_NED_home(this->_vectorAux);
    this->_currentPosition << (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    this->_ahrs.get_velocity_NED(this->_vectorAux);
    this->_currentVelocity << (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    this->_vectorAux = this->_ahrs.get_accel_ef_blended();
    this->_currentAcceleration <<  (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;

    // Update current attitude and angular velocity from AHRS converting from NED/NED to ENU/ENU
    this->_ahrs.get_quat_body_to_ned(this->_quat);
    this->_currentAttitude.w = (double) -SQRT2_DIV2*(this->_quat.q1+this->_quat.q4);
    this->_currentAttitude.x = (double) -SQRT2_DIV2*(this->_quat.q2+this->_quat.q3);
    this->_currentAttitude.y = (double) SQRT2_DIV2*(this->_quat.q3-this->_quat.q2);
    this->_currentAttitude.z = (double) SQRT2_DIV2*(this->_quat.q4+this->_quat.q1);
    this->_currentAttitude.normalize();
    this->_vectorAux = _ahrs.get_gyro();
    this->_currentAngularVelocity << (double) this->_vectorAux.x, (double) -this->_vectorAux.y, (double) -this->_vectorAux.z;

    // Update control time step
    double thisCall = (double) AP_HAL::millis()/1000.0;
    this->_controlTimeStep = 0.5*(this->_controlTimeStep+thisCall-this->_lastCall);
    this->_lastCall = thisCall;

    return true;
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
};

bool MultiControl::attitudeReference(){
    this->_qyd.w = (double) cos(this->_desiredYaw/2.0);
    this->_qyd.z = (double) sin(this->_desiredYaw/2.0);
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
    this->_qCB.w = (double) cos(thetaCB/2);
    this->_qCB.x = (double) this->_vCB(0)*sin(thetaCB/2);
    this->_qCB.y = (double) this->_vCB(1)*sin(thetaCB/2);
    this->_qCB.z = (double) this->_vCB(2)*sin(thetaCB/2);
    // compound rotation -> desired attitude
    this->_desiredAttitude.w = this->_qCB.w*this->_qyd.w-this->_qCB.x*this->_qyd.x-this->_qCB.y*this->_qyd.y-this->_qCB.z*this->_qyd.z;
    this->_desiredAttitude.x = this->_qCB.w*this->_qyd.x+this->_qCB.x*this->_qyd.w+this->_qCB.y*this->_qyd.z-this->_qCB.z*this->_qyd.y;
    this->_desiredAttitude.y = this->_qCB.w*this->_qyd.y-this->_qCB.x*this->_qyd.z+this->_qCB.y*this->_qyd.w+this->_qCB.z*this->_qyd.x;
    this->_desiredAttitude.z = this->_qCB.w*this->_qyd.z+this->_qCB.x*this->_qyd.y-this->_qCB.y*this->_qyd.x+this->_qCB.z*this->_qyd.w;
            
    this->_desiredAttitude.normalize();
};

bool MultiControl::attitudeFTLQRControl(){

};

bool MultiControl::controlAllocation(){

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
    AP_GROUPINFO("ROT1_POS", 17, MultiControl, _rotorPosition[0], ROTOR1_POSITION_DEFAULT),

    // @Param: ROT2_POS
    // @DisplayName: Rotor 2 position
    // @Description: Position of rotor 2 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_POS", 18, MultiControl,  _rotorPosition[1], ROTOR2_POSITION_DEFAULT),

    // @Param: ROT3_POS
    // @DisplayName: Rotor 3 position
    // @Description: Position of rotor 3 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_POS", 19, MultiControl,  _rotorPosition[2], ROTOR3_POSITION_DEFAULT),

    // @Param: ROT4_POS
    // @DisplayName: Rotor 4 position
    // @Description: Position of rotor 4 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_POS", 20, MultiControl,  _rotorPosition[3], ROTOR4_POSITION_DEFAULT),

    // @Param: ROT5_POS
    // @DisplayName: Rotor 5 position
    // @Description: Position of rotor 5 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_POS", 21, MultiControl,  _rotorPosition[4], ROTOR5_POSITION_DEFAULT),

    // @Param: ROT6_POS
    // @DisplayName: Rotor 6 position
    // @Description: Position of rotor 6 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_POS", 22, MultiControl,  _rotorPosition[5], ROTOR6_POSITION_DEFAULT),

    // @Param: ROT7_POS
    // @DisplayName: Rotor 7 position
    // @Description: Position of rotor 7 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_POS", 23, MultiControl,  _rotorPosition[6], ROTOR7_POSITION_DEFAULT),

    // @Param: ROT8_POS
    // @DisplayName: Rotor 8 position
    // @Description: Position of rotor 8 in relation to CG. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_POS", 24, MultiControl,  _rotorPosition[7], ROTOR8_POSITION_DEFAULT),

    // @Param: ROT1_ORI
    // @DisplayName: Rotor 1 orientation
    // @Description: Orientation of rotor 1 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_ORI", 25, MultiControl, _rotorOrientation[0], ROTOR1_ORIENTATION_DEFAULT),

    // @Param: ROT2_ORI
    // @DisplayName: Rotor 2 orientation
    // @Description: Orientation of rotor 2 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_ORI", 26, MultiControl, _rotorOrientation[1], ROTOR2_ORIENTATION_DEFAULT),

    // @Param: ROT3_ORI
    // @DisplayName: Rotor 3 orientation
    // @Description: Orientation of rotor 3 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_ORI", 27, MultiControl, _rotorOrientation[2], ROTOR3_ORIENTATION_DEFAULT),

    // @Param: ROT4_ORI
    // @DisplayName: Rotor 4 orientation
    // @Description: Orientation of rotor 4 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_ORI", 28, MultiControl, _rotorOrientation[3], ROTOR4_ORIENTATION_DEFAULT),

    // @Param: ROT5_ORI
    // @DisplayName: Rotor 5 orientation
    // @Description: Orientation of rotor 5 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_ORI", 29, MultiControl, _rotorOrientation[4], ROTOR5_ORIENTATION_DEFAULT),

    // @Param: ROT6_ORI
    // @DisplayName: Rotor 6 orientation
    // @Description: Orientation of rotor 6 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_ORI", 30, MultiControl, _rotorOrientation[5], ROTOR6_ORIENTATION_DEFAULT),

    // @Param: ROT7_ORI
    // @DisplayName: Rotor 7 orientation
    // @Description: Orientation of rotor 7 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_ORI", 31, MultiControl, _rotorOrientation[6], ROTOR7_ORIENTATION_DEFAULT),

    // @Param: ROT8_ORI
    // @DisplayName: Rotor 8 orientation
    // @Description: Orientation of rotor 8 in relation to body system. (x,y,z) 
    // @Range: -3.4E+38 3.4E+38
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_ORI", 32, MultiControl, _rotorOrientation[7], ROTOR8_ORIENTATION_DEFAULT),

    // @Param: ROT1_DIR
    // @DisplayName: Rotor 1 rotation direction
    // @Description: Direction of rotation of rotor 1 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT1_DIR", 33, MultiControl, _rotorDirection[0], ROTOR1_DIRECTION_DEFAULT),

    // @Param: ROT2_DIR
    // @DisplayName: Rotor 2 rotation direction
    // @Description: Direction of rotation of rotor 2 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT2_DIR", 34, MultiControl, _rotorDirection[1], ROTOR2_DIRECTION_DEFAULT),

    // @Param: ROT3_DIR
    // @DisplayName: Rotor 3 rotation direction
    // @Description: Direction of rotation of rotor 3 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT3_DIR", 35, MultiControl, _rotorDirection[2], ROTOR3_DIRECTION_DEFAULT),

    // @Param: ROT4_DIR
    // @DisplayName: Rotor 4 rotation direction
    // @Description: Direction of rotation of rotor 4 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT4_DIR", 36, MultiControl, _rotorDirection[3], ROTOR4_DIRECTION_DEFAULT),

    // @Param: ROT5_DIR
    // @DisplayName: Rotor 5 rotation direction
    // @Description: Direction of rotation of rotor 5 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT5_DIR", 37, MultiControl, _rotorDirection[4], ROTOR5_DIRECTION_DEFAULT),

    // @Param: ROT6_DIR
    // @DisplayName: Rotor 6 rotation direction
    // @Description: Direction of rotation of rotor 6 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT6_DIR", 38, MultiControl, _rotorDirection[5], ROTOR6_DIRECTION_DEFAULT),

    // @Param: ROT7_DIR
    // @DisplayName: Rotor 7 rotation direction
    // @Description: Direction of rotation of rotor 7 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT7_DIR", 39, MultiControl, _rotorDirection[6], ROTOR7_DIRECTION_DEFAULT),

    // @Param: ROT8_DIR
    // @DisplayName: Rotor 8 rotation direction
    // @Description: Direction of rotation of rotor 8 in relation to its orientation axis 
    // @Values: -1:CW,1:CCW
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ROT8_DIR", 40, MultiControl, _rotorDirection[7], ROTOR8_DIRECTION_DEFAULT),

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

    // @Param: FTLQR_P11
    // @DisplayName: FT-LQR controller P matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P11", 46, MultiControl, _ftlqrConfig.P[0], FTLQR_CONFIG_P11_DEFAULT),

    // @Param: FTLQR_P22
    // @DisplayName: FT-LQR controller P matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P22", 47, MultiControl, _ftlqrConfig.P[1], FTLQR_CONFIG_P22_DEFAULT),

    // @Param: FTLQR_P33
    // @DisplayName: FT-LQR controller P matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P33", 48, MultiControl, _ftlqrConfig.P[2], FTLQR_CONFIG_P33_DEFAULT),

    // @Param: FTLQR_P44
    // @DisplayName: FT-LQR controller P matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P44", 49, MultiControl, _ftlqrConfig.P[3], FTLQR_CONFIG_P44_DEFAULT),

    // @Param: FTLQR_P55
    // @DisplayName: FT-LQR controller P matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P55", 50, MultiControl, _ftlqrConfig.P[4], FTLQR_CONFIG_P55_DEFAULT),

    // @Param: FTLQR_P66
    // @DisplayName: FT-LQR controller P matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller Ricatti matrix (P) 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_P66", 51, MultiControl, _ftlqrConfig.P[5], FTLQR_CONFIG_P66_DEFAULT),

    // @Param: FTLQR_Q11
    // @DisplayName: FT-LQR controller Q matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q11", 52, MultiControl, _ftlqrConfig.Q[0], FTLQR_CONFIG_Q11_DEFAULT),

    // @Param: FTLQR_Q22
    // @DisplayName: FT-LQR controller Q matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q22", 53, MultiControl, _ftlqrConfig.Q[1], FTLQR_CONFIG_Q22_DEFAULT),

    // @Param: FTLQR_Q33
    // @DisplayName: FT-LQR controller Q matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q33", 54, MultiControl, _ftlqrConfig.Q[2], FTLQR_CONFIG_Q33_DEFAULT),

    // @Param: FTLQR_Q44
    // @DisplayName: FT-LQR controller Q matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q44", 55, MultiControl, _ftlqrConfig.Q[3], FTLQR_CONFIG_Q44_DEFAULT),

    // @Param: FTLQR_Q55
    // @DisplayName: FT-LQR controller Q matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q55", 56, MultiControl, _ftlqrConfig.Q[4], FTLQR_CONFIG_Q55_DEFAULT),

    // @Param: FTLQR_Q66
    // @DisplayName: FT-LQR controller Q matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller state matrix Q 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Q66", 57, MultiControl, _ftlqrConfig.Q[5], FTLQR_CONFIG_Q66_DEFAULT),

    // @Param: FTLQR_R11
    // @DisplayName: FT-LQR controller R matrix (1,1)
    // @Description: Element (1,1) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R11", 58, MultiControl, _ftlqrConfig.R[0], FTLQR_CONFIG_R11_DEFAULT),

    // @Param: FTLQR_R22
    // @DisplayName: FT-LQR controller R matrix (2,2)
    // @Description: Element (2,2) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R22", 59, MultiControl, _ftlqrConfig.R[1], FTLQR_CONFIG_R22_DEFAULT),

    // @Param: FTLQR_R33
    // @DisplayName: FT-LQR controller R matrix (3,3)
    // @Description: Element (3,3) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R33", 60, MultiControl, _ftlqrConfig.R[2], FTLQR_CONFIG_R33_DEFAULT),

    // @Param: FTLQR_R44
    // @DisplayName: FT-LQR controller R matrix (4,4)
    // @Description: Element (4,4) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R44", 61, MultiControl, _ftlqrConfig.R[3], FTLQR_CONFIG_R44_DEFAULT),

    // @Param: FTLQR_R55
    // @DisplayName: FT-LQR controller R matrix (5,5)
    // @Description: Element (5,5) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R55", 62, MultiControl, _ftlqrConfig.R[4], FTLQR_CONFIG_R55_DEFAULT),

    // @Param: FTLQR_R66
    // @DisplayName: FT-LQR controller R matrix (6,6)
    // @Description: Element (6,6) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R66", 63, MultiControl, _ftlqrConfig.R[5], FTLQR_CONFIG_R66_DEFAULT),

    // @Param: FTLQR_R77
    // @DisplayName: FT-LQR controller R matrix (7,7)
    // @Description: Element (7,7) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R77", 64, MultiControl, _ftlqrConfig.R[6], FTLQR_CONFIG_R77_DEFAULT),

    // @Param: FTLQR_R88
    // @DisplayName: FT-LQR controller R matrix (8,8)
    // @Description: Element (8,8) of FT-LQR controller input matrix R 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_R88", 65, MultiControl, _ftlqrConfig.R[7], FTLQR_CONFIG_R88_DEFAULT),

    // @Param: FTLQR_Ef1
    // @DisplayName: FT-LQR controller Ef matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef1", 66, MultiControl, _ftlqrConfig.Ef[0], FTLQR_CONFIG_EF1_DEFAULT),

    // @Param: FTLQR_Ef2
    // @DisplayName: FT-LQR controller Ef matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef2", 67, MultiControl, _ftlqrConfig.Ef[1], FTLQR_CONFIG_EF2_DEFAULT),

    // @Param: FTLQR_Ef3
    // @DisplayName: FT-LQR controller Ef matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef3", 68, MultiControl, _ftlqrConfig.Ef[2], FTLQR_CONFIG_EF3_DEFAULT),

    // @Param: FTLQR_Ef4
    // @DisplayName: FT-LQR controller Ef matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef4", 69, MultiControl, _ftlqrConfig.Ef[3], FTLQR_CONFIG_EF4_DEFAULT),

    // @Param: FTLQR_Ef5
    // @DisplayName: FT-LQR controller Ef matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef5", 70, MultiControl, _ftlqrConfig.Ef[4], FTLQR_CONFIG_EF5_DEFAULT),

    // @Param: FTLQR_Ef6
    // @DisplayName: FT-LQR controller Ef matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix Ef 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Ef6", 71, MultiControl, _ftlqrConfig.Ef[5], FTLQR_CONFIG_EF6_DEFAULT),

    // @Param: FTLQR_Eg1
    // @DisplayName: FT-LQR controller Eg matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg1", 72, MultiControl, _ftlqrConfig.Eg[0], FTLQR_CONFIG_EG1_DEFAULT),

    // @Param: FTLQR_Eg2
    // @DisplayName: FT-LQR controller Eg matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg2", 73, MultiControl, _ftlqrConfig.Eg[1], FTLQR_CONFIG_EG2_DEFAULT),

    // @Param: FTLQR_Eg3
    // @DisplayName: FT-LQR controller Eg matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg3", 74, MultiControl, _ftlqrConfig.Eg[2], FTLQR_CONFIG_EG3_DEFAULT),

    // @Param: FTLQR_Eg4
    // @DisplayName: FT-LQR controller Eg matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg4", 75, MultiControl, _ftlqrConfig.Eg[3], FTLQR_CONFIG_EG4_DEFAULT),

    // @Param: FTLQR_Eg5
    // @DisplayName: FT-LQR controller Eg matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg5", 76, MultiControl, _ftlqrConfig.Eg[4], FTLQR_CONFIG_EG5_DEFAULT),

    // @Param: FTLQR_Eg6
    // @DisplayName: FT-LQR controller Eg matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg6", 77, MultiControl, _ftlqrConfig.Eg[5], FTLQR_CONFIG_EG6_DEFAULT),

    // @Param: FTLQR_Eg7
    // @DisplayName: FT-LQR controller Eg matrix (7)
    // @Description: Element (7) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg7", 78, MultiControl, _ftlqrConfig.Eg[6], FTLQR_CONFIG_EG7_DEFAULT),

    // @Param: FTLQR_Eg8
    // @DisplayName: FT-LQR controller Eg matrix (8)
    // @Description: Element (8) of FT-LQR controller error uncertainty matrix Eg 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_Eg8", 79, MultiControl, _ftlqrConfig.Eg[7], FTLQR_CONFIG_EG8_DEFAULT),

    // @Param: FTLQR_H1
    // @DisplayName: FT-LQR controller H matrix (1)
    // @Description: Element (1) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H1", 80, MultiControl, _ftlqrConfig.H[0], FTLQR_CONFIG_H1_DEFAULT),

    // @Param: FTLQR_H2
    // @DisplayName: FT-LQR controller H matrix (2)
    // @Description: Element (2) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H2", 81, MultiControl, _ftlqrConfig.H[1], FTLQR_CONFIG_H2_DEFAULT),

    // @Param: FTLQR_H3
    // @DisplayName: FT-LQR controller H matrix (3)
    // @Description: Element (3) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H3", 82, MultiControl, _ftlqrConfig.H[2], FTLQR_CONFIG_H3_DEFAULT),

    // @Param: FTLQR_H4
    // @DisplayName: FT-LQR controller H matrix (4)
    // @Description: Element (4) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H4", 83, MultiControl, _ftlqrConfig.H[3], FTLQR_CONFIG_H4_DEFAULT),

    // @Param: FTLQR_H5
    // @DisplayName: FT-LQR controller H matrix (5)
    // @Description: Element (5) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H5", 84, MultiControl, _ftlqrConfig.H[4], FTLQR_CONFIG_H5_DEFAULT),

    // @Param: FTLQR_H6
    // @DisplayName: FT-LQR controller H matrix (6)
    // @Description: Element (6) of FT-LQR controller error uncertainty matrix H 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_H6", 85, MultiControl, _ftlqrConfig.H[5], FTLQR_CONFIG_H6_DEFAULT),

    // @Param: FTLQR_MU
    // @DisplayName: FT-LQR controller mu value
    // @Description: Mu value of FT-LQR controller 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_MU", 86, MultiControl, _ftlqrConfig.mu, FTLQR_CONFIG_MU_DEFAULT),

    // @Param: FTLQR_ALP
    // @DisplayName: FT-LQR controller alpha value
    // @Description: Alpha value of FT-LQR controller 
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("FTLQR_ALP", 87, MultiControl, _ftlqrConfig.alpha, FTLQR_CONFIG_ALPHA_DEFAULT),
    
    // @Param: CA_WM
    // @DisplayName: Control allocation maneuverability weight
    // @Description: Weight to ponderate maneuverability over attitude following for control allocation algorithm
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("CA_WM", 88, MultiControl, _caConfig.Wm, CA_WM_DEFAULT),
    
    // @Param: CA_WA
    // @DisplayName: Control allocation attitude following weight
    // @Description: Weight to ponderate attitude following over maneuverability for control allocation algorithm
    // @Range: -3.4E+38 3.4E+38
    // @User: Advanced
    AP_GROUPINFO("CA_WA", 89, MultiControl, _caConfig.Wa, CA_WA_DEFAULT),

    AP_GROUPEND
};