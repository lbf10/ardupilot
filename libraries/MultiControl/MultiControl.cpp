# include "MultiControl.h"

MultiControl::MultiControl():
    // Links AHRS objects
    _ahrs(AP::ahrs())
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
    this->_weightVector << 0.0, 0.0, MASS*9.81;

    // Calculates Mf and Mt
    this->_Mf.resize(3,NUMBER_OF_ROTORS);
    this->_Mt.resize(3,NUMBER_OF_ROTORS);
    this->_pinvMt.resize(NUMBER_OF_ROTORS,3);

    this->_rotorPosition.resize(NUMBER_OF_ROTORS,3);
    this->_rotorPosition << ROTOR1_POSITION,
                            ROTOR2_POSITION,
                            ROTOR3_POSITION,
                            ROTOR4_POSITION,
                            ROTOR5_POSITION,
                            ROTOR6_POSITION,
                            ROTOR7_POSITION,
                            ROTOR8_POSITION;
    this->_rotorPosition.transposeInPlace();
    this->_rotorOrientation.resize(NUMBER_OF_ROTORS,3);
    this->_rotorOrientation <<  ROTOR1_ORIENTATION,
                                ROTOR2_ORIENTATION,
                                ROTOR3_ORIENTATION,
                                ROTOR4_ORIENTATION,
                                ROTOR5_ORIENTATION,
                                ROTOR6_ORIENTATION,
                                ROTOR7_ORIENTATION,
                                ROTOR8_ORIENTATION;
    this->_rotorOrientation.transposeInPlace();
    this->_rotorDirection.resize(NUMBER_OF_ROTORS);
    this->_rotorDirection << ROTOR_DIRECTION;
    Eigen::Vector3d aux;
    Eigen::Vector3d aux2;

    for(int it=0;it<NUMBER_OF_ROTORS;it++){
        this->_Mf.col(it) = ROTOR_LIFT_COEFF*this->_rotorPosition.col(it);
        aux = ROTOR_LIFT_COEFF*this->_rotorPosition.col(it);
        aux2 = this->_rotorOrientation.col(it);
        this->_Mt.col(it) = aux.cross(aux2)-ROTOR_DRAG_COEFF*((double) this->_rotorDirection(it))*this->_rotorOrientation.col(it);
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
    this->_inertia << (double) MOMENTS_OF_INERTIA_XX, (double) PRODUCTS_OF_INERTIA_XY, (double) PRODUCTS_OF_INERTIA_XZ,
                      (double) PRODUCTS_OF_INERTIA_XY, (double) MOMENTS_OF_INERTIA_YY, (double) PRODUCTS_OF_INERTIA_YZ,
                      (double) PRODUCTS_OF_INERTIA_XZ, (double) PRODUCTS_OF_INERTIA_YZ, (double) MOMENTS_OF_INERTIA_ZZ;  

    // Zero dummy state for testing   
    this->_dummyState.acceleration.setZero();
    this->_dummyState.position.setZero();
    this->_dummyState.velocity.setZero();
    this->_dummyState.yaw = 0.0;

    ///////////////////////////
    /* Position PIDD related */

    // Initiate PIDD gains
    this->_piddConst.Kp << (double) PIDD_KP_X, (double) PIDD_KP_Y, (double) PIDD_KP_Z;
    this->_piddConst.Ki << (double) PIDD_KI_X, (double) PIDD_KI_Y, (double) PIDD_KI_Z;
    this->_piddConst.Kd << (double) PIDD_KD_X, (double) PIDD_KD_Y, (double) PIDD_KD_Z;
    this->_piddConst.Kdd << (double) PIDD_KDD_X, (double) PIDD_KDD_Y, (double) PIDD_KDD_Z;

    // Initiate PIDD integral error
    this->_pidd.iError = Eigen::Vector3d::Zero();

    ///////////////////////////////
    /* Atitude reference related */

    // Calculates attRefAux
    Eigen::MatrixXd auxAtt(this->_nullMt.cols(),this->_nullMt.cols());
    this->_attRefAux.resize(this->_nullMt.cols(),this->_nullMt.rows());
    auxAtt = this->_nullMt.transpose()*(((double)CA_WM)*this->_nullMt+this->_Mf.transpose()*((double)CA_WA)*this->_Mf*this->_nullMt);
    this->_attRefAux = auxAtt.colPivHouseholderQr().solve(this->_nullMt.transpose());

    // Auxiliary variables
    this->_v.resize(this->_attRefAux.rows(),1);
    this->_omegaSquared.resize(NUMBER_OF_ROTORS,1);
    this->_qyd.x() = 0.0;
    this->_qyd.y() = 0.0;
    this->_attRefAux2.resize(this->_Mf.cols(),1);

    //////////////////////////////
    /* Attitude control related */
    
    // Zero velocity filter variables
    this->_velFilter.Wbe.setZero();
    this->_velFilter.angularVelocity.setZero();
    this->_velFilter.desiredAngularVelocity.setZero();

    // Zero rotor states
    this->_currentRotorSpeeds.setZero();

    ////////////////////
    /* FT-LQR related */

    // Calculates C
    this->_ftLQRConst.C.resize(3, NUMBER_OF_ROTORS);
    this->_ftLQRConst.C = -this->_inertia.inverse()*this->_Mt;

    return true;
};

//extern const AP_HAL::HAL& hal; 

bool MultiControl::updateStates(PolyNavigation::state desiredState){
    // Update desired states from navigation
    this->_desiredPosition << (double)desiredState.position.x(), (double)desiredState.position.y(), (double)desiredState.position.z();
    this->_desiredVelocity << (double)desiredState.velocity.x(), (double)desiredState.velocity.y(), (double)desiredState.velocity.z();
    this->_desiredAcceleration << (double)desiredState.acceleration.x(), (double)desiredState.acceleration.y(), (double)desiredState.acceleration.z();
    this->_desiredYaw = (double) desiredState.yaw;

    //hal.console->printf("passou desiredState");
    // Update current states from AHRS
    bool returnState = true;
    if(this->_ahrs.get_relative_position_NED_home(this->_vectorAux)){
        this->_currentPosition << (double) this->_vectorAux.y, (double) this->_vectorAux.x, (double) -this->_vectorAux.z;
    } else { returnState = false;}

    //hal.console->printf("passou get_relative_position_NED_home");
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
    this->_attRefAux2 << (((double)CA_WM)*ROTOR_OPERATING_POINT_SQUARED+(this->_Mf.transpose()*((double)CA_WA)*this->_Tcd).array()).matrix();

    this->_v << this->_attRefAux*this->_attRefAux2;

    this->_omegaSquared = this->_nullMt*this->_v;

    for(int it=0;it<this->_omegaSquared.size();it++){
        if(this->_omegaSquared(it)<ROTOR_MIN_SPEED_SQUARED){
            this->_omegaSquared(it) = ROTOR_MIN_SPEED_SQUARED;
        }
        else if(this->_omegaSquared(it)>ROTOR_MAX_SPEED_SQUARED){
            this->_omegaSquared(it) = ROTOR_MAX_SPEED_SQUARED;
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
    Eigen::Vector3d previousWbe;
    Eigen::Vector3d previousAngularVelocity;
    Eigen::Vector3d angularVelocity;
    Eigen::Quaterniond qe;
    Eigen::Vector3d angularAcceleration;
    Eigen::Vector3d dWbe;
    Eigen::Vector3d desiredAngularAcceleration;

    previousWbe = this->_velFilter.Wbe;
    previousAngularVelocity = this->_velFilter.angularVelocity;
    angularVelocity = this->_velFilter.angularVelocity;
    this->_velFilter.angularVelocity = this->_currentAngularVelocity;
       
    // Quaternion error2
    qe.w() = + this->_desiredAttitude.w()*this->_currentAttitude.w() + this->_desiredAttitude.x()*this->_currentAttitude.x() + this->_desiredAttitude.y()*this->_currentAttitude.y() + this->_desiredAttitude.z()*this->_currentAttitude.z();
    qe.x() = + this->_desiredAttitude.x()*this->_currentAttitude.w() - this->_desiredAttitude.w()*this->_currentAttitude.x() - this->_desiredAttitude.z()*this->_currentAttitude.y() + this->_desiredAttitude.y()*this->_currentAttitude.z();
    qe.y() = + this->_desiredAttitude.y()*this->_currentAttitude.w() + this->_desiredAttitude.z()*this->_currentAttitude.x() - this->_desiredAttitude.w()*this->_currentAttitude.y() - this->_desiredAttitude.x()*this->_currentAttitude.z();
    qe.z() = + this->_desiredAttitude.z()*this->_currentAttitude.w() - this->_desiredAttitude.y()*this->_currentAttitude.x() + this->_desiredAttitude.x()*this->_currentAttitude.y() - this->_desiredAttitude.w()*this->_currentAttitude.z();
    if(qe.w()<0){
        qe.x() = -qe.x();
        qe.y() = -qe.y();
        qe.z() = -qe.z();
    }
            
    this->_velFilter.desiredAngularVelocity = (this->_velocityFilterGain.array()*this->_velFilter.desiredAngularVelocity.array()+(Eigen::Array3d::Ones()-this->_velocityFilterGain.array())*(qe.vec().array()/this->_controlTimeStep)).matrix();
    angularAcceleration = (angularVelocity-previousAngularVelocity)/this->_controlTimeStep;
    this->_velFilter.Wbe = this->_velFilter.desiredAngularVelocity-angularVelocity;
    dWbe = (this->_velFilter.Wbe-previousWbe)/this->_controlTimeStep;
    desiredAngularAcceleration = (dWbe+angularAcceleration);
        /*    
     index = 3;   
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end                 
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(localRotorSpeeds(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    
                    C = -obj.inertia()\Mt;
                    ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                    ssB = [C;zeros(3,size(C,2))];
                    sys = ss(ssA,ssB,eye(6),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);  
                */    
    
    
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

/* Converts from continuous to discrete state-space model */
// References:
// C2D: http://staff.uz.zgora.pl/wpaszke/materialy/spc/Lec11.pdf
// Matrix exponential in Eigen: https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html#matrixbase_exp
// Matrix exponential in Matlab: https://www.mathworks.com/help/matlab/ref/expm.html
void MultiControl::c2d(const Eigen::Ref<const Eigen::MatrixXd>& Ac,const Eigen::Ref<const Eigen::MatrixXd>& Bc, 
                        double ts, Eigen::Ref<Eigen::MatrixXd> Ad, Eigen::Ref<Eigen::MatrixXd> Bd){
    Eigen::MatrixXd aux2(Ac.rows(),Ac.cols());
    aux2 = Ac*ts;
    Eigen::MatrixExponentialReturnValue<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> result(aux2);
    result.evalTo(Ad);
    Eigen::MatrixXd aux(Ac.rows(),Ac.cols());
    aux = Ad;
    aux.diagonal().array() -= 1;
    Bd = Ac.colPivHouseholderQr().solve(aux.matrix()*Bc);
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
        vel.x = this->_currentVelocity(0);
        vel.y = this->_currentVelocity(1);
        vel.z = this->_currentVelocity(2);
        return vel;
    };

    Vector3f MultiControl::currentVelocityNED(){
        Vector3f vel;
        vel.x = this->_currentVelocity(1);
        vel.y = this->_currentVelocity(0);
        vel.z = -this->_currentVelocity(2);
        return vel;
    };

    Vector3f MultiControl::currentAcceleration(){
        Vector3f acc;
        acc.x = this->_currentAcceleration(0);
        acc.y = this->_currentAcceleration(1);
        acc.z = this->_currentAcceleration(2);
        return acc;
    };

    Vector3f MultiControl::currentAccelerationNED(){
        Vector3f acc;
        acc.x = this->_currentAcceleration(1);
        acc.y = this->_currentAcceleration(0);
        acc.z = -this->_currentAcceleration(2);
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