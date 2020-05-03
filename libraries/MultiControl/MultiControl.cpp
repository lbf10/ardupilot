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
    this->_rotorPosition << ROTOR_POSITION;
    this->_rotorPosition.transposeInPlace();
    this->_rotorOrientation.resize(NUMBER_OF_ROTORS,3);
    this->_rotorOrientation <<  ROTOR_ORIENTATION;
    this->_rotorOrientation.transposeInPlace();
    this->_rotorDirection.resize(NUMBER_OF_ROTORS);
    this->_rotorDirection << ROTOR_DIRECTION;
    Eigen::Vector3d aux;
    Eigen::Vector3d aux2;

    for(int it=0;it<NUMBER_OF_ROTORS;it++){
        this->_Mf.col(it) = ROTOR_LIFT_COEFF*this->_rotorOrientation.col(it);
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
    this->_nullMt = svd.matrixV().rightCols(svd.matrixV().cols()-svd.nonzeroSingularValues());

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

    // Calculates ssB
    this->_ftLQRConst.ssB.resize(6, NUMBER_OF_ROTORS);
    this->_ftLQRConst.ssB.topRows<3>() = -this->_inertia.inverse()*this->_Mt;
    this->_ftLQRConst.ssB.bottomRows<3>().setZero();

    //Initialize Q
    Eigen::MatrixXd Qaux(6,6);
    Qaux.setZero();
    Qaux.diagonal() << FTLQR_CONFIG_Q_DIAG;
    this->_ftLQRConst.Q.resize(6,6);
    this->_ftLQRConst.Q.reserve(6);
    for(int it=0;it<6;it++){
        this->_ftLQRConst.Q.insert(it,it) = Qaux(it,it);
    };

    //Initialize R
    Eigen::MatrixXd Raux(NUMBER_OF_ROTORS,NUMBER_OF_ROTORS);
    Raux.setZero();
    Raux.diagonal() << FTLQR_CONFIG_R_DIAG;
    this->_ftLQRConst.R.resize(NUMBER_OF_ROTORS,NUMBER_OF_ROTORS);
    this->_ftLQRConst.R.reserve(NUMBER_OF_ROTORS);
    for(int it=0;it<NUMBER_OF_ROTORS;it++){
        this->_ftLQRConst.R.insert(it,it) = Raux(it,it);
    };

    // Initialize "P"
    this->_ftLQR.P.setZero();
    this->_ftLQR.P.diagonal() << FTLQR_CONFIG_P_DIAG;
    
    //Initialize Ef and Eg
    this->_ftLQRConst.Ef << FTLQR_CONFIG_EF_ROW;
        
    // Temporaries
    // H
    Eigen::MatrixXd H(6,1);
    H << FTLQR_CONFIG_H_COL;
    // lambda
    double lambda = FTLQR_CONFIG_ALPHA*FTLQR_CONFIG_MU*(H.transpose()*H).norm();
    // middle
    Eigen::MatrixXd middle(6,6);
    middle = Eigen::Matrix<double,6,6>::Ones()/FTLQR_CONFIG_MU-(H*H.transpose())/lambda;
    // inv R
    Eigen::VectorXd invR(Raux.cols());
    invR = Raux.inverse().diagonal();
    // inv Q
    Eigen::VectorXd invQ(Qaux.cols());
    invQ = Qaux.inverse().diagonal();
    // inv P
    Eigen::VectorXd invP(6);
    invP = this->_ftLQR.P.inverse().diagonal();
    // Eg
    Eigen::VectorXd Eg(NUMBER_OF_ROTORS);
    Eg << FTLQR_CONFIG_EG_ROW;


    // Initialize "right"
    this->_ftLQR.right.resize(25+2*NUMBER_OF_ROTORS,6);
    this->_ftLQR.right.reserve(50);
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(12);
    for(int it=0;it<6;it++)
    {
        tripletList.push_back(Eigen::Triplet<double>(6+NUMBER_OF_ROTORS+it,it,-1));
    }
    for(int it=0;it<6;it++)
    {
        tripletList.push_back(Eigen::Triplet<double>(18+NUMBER_OF_ROTORS,it,this->_ftLQRConst.Ef(it)));
    }
    this->_ftLQR.right.setFromTriplets(tripletList.begin(), tripletList.end());

    // Initialize "left"
    this->_ftLQR.left.resize(25+2*NUMBER_OF_ROTORS,25+2*NUMBER_OF_ROTORS);
    this->_ftLQR.left.reserve(103+NUMBER_OF_ROTORS*(5+12));
    tripletList.clear();
    tripletList.reserve(103+NUMBER_OF_ROTORS*(5+12));
    // push 6-sized blocks
    for(int it=0;it<6;it++)
    {   
        // push P
        tripletList.push_back(Eigen::Triplet<double>(it,it,invP(it)));
        // push Q
        tripletList.push_back(Eigen::Triplet<double>(6+NUMBER_OF_ROTORS+it,6+NUMBER_OF_ROTORS+it,invQ(it)));
        // push eyes
        tripletList.push_back(Eigen::Triplet<double>(it,19+NUMBER_OF_ROTORS+it,1));
        tripletList.push_back(Eigen::Triplet<double>(12+NUMBER_OF_ROTORS+it,19+NUMBER_OF_ROTORS+it,1));
        tripletList.push_back(Eigen::Triplet<double>(19+NUMBER_OF_ROTORS+it,it,1));
        tripletList.push_back(Eigen::Triplet<double>(19+NUMBER_OF_ROTORS+it,12+NUMBER_OF_ROTORS+it,1));
        
    }
    // push NUMBER_OF_ROTORS-sized blocks
    for(int it=0;it<NUMBER_OF_ROTORS;it++)
    {   
        // push R
        tripletList.push_back(Eigen::Triplet<double>(6+it,6+it,invR(it)));
        // push eyes
        tripletList.push_back(Eigen::Triplet<double>(6+it,25+NUMBER_OF_ROTORS+it,1));
        tripletList.push_back(Eigen::Triplet<double>(25+NUMBER_OF_ROTORS+it,6+it,1));
        // push Egs 
        tripletList.push_back(Eigen::Triplet<double>(18+NUMBER_OF_ROTORS,25+NUMBER_OF_ROTORS+it,-Eg(it)));
        tripletList.push_back(Eigen::Triplet<double>(25+NUMBER_OF_ROTORS+it,18+NUMBER_OF_ROTORS,-Eg(it)));
    }
    tripletList.push_back(Eigen::Triplet<double>(18+NUMBER_OF_ROTORS,18+NUMBER_OF_ROTORS,1/lambda));
    // push "middle"
    for(int it=0;it<6;it++){
        for(int jt=0;jt<6;jt++){
            tripletList.push_back(Eigen::Triplet<double>(12+NUMBER_OF_ROTORS+it,12+NUMBER_OF_ROTORS+jt,middle(it,jt)));
        }
    }
    this->_ftLQR.left.setFromTriplets(tripletList.begin(), tripletList.end());
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
       
    // Quaternion error
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
    Eigen::Vector3d torqueAux;
    torqueAux.setZero();
    for(int it=0;it<NUMBER_OF_ROTORS;it++){
        torqueAux = torqueAux + this->_rotorOrientation.col(it)*this->_currentRotorSpeeds(it)*ROTOR_INERTIA;
    }
    torqueAux = this->_inertia*this->_currentAngularVelocity-torqueAux; 
    Eigen::Matrix3d auxA;
    auxA <<             0, -torqueAux(2),  torqueAux(1),
             torqueAux(2),             0, -torqueAux(0),
            -torqueAux(1),  torqueAux(0),             0;
    auxA = this->_inertia.inverse()*auxA;
    
    Eigen::Matrix3d Sq;
    Sq << qe.w(), -qe.z(),  qe.y(),
          qe.z(),  qe.w(), -qe.x(),
         -qe.y(),  qe.x(),  qe.w();

    Eigen::Vector3d x_e;
    x_e << this->_velFilter.desiredAngularVelocity-this->_currentAngularVelocity, qe.vec();
    
    Eigen::Matrix<double,6,6> ssA;
    ssA.setZero();
    ssA.topLeftCorner<3,3>() = auxA;
    ssA.bottomLeftCorner<3,3>() = 0.5*Sq;

    Eigen::Matrix<double,6,6> F;
    Eigen::Matrix<double,6,NUMBER_OF_ROTORS> G;
    c2d(ssA, this->_ftLQRConst.ssB,this->_controlTimeStep,F,G);

    Eigen::Matrix<double,NUMBER_OF_ROTORS,6> K;
    gainRLQR(F,G,K);

    Eigen::Matrix<double,6,1> u;
    u << K*x_e;
    Eigen::Vector3d attitudeControlOutput;
    attitudeControlOutput = -this->_inertia*(this->_ftLQRConst.ssB.topRows<3>()*u-desiredAngularAcceleration+auxA*this->_velFilter.desiredAngularVelocity);  

    return true;
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

/* Calculates Robust LQR */
// References:
// Joao Paulo Cerri, Master`s
void MultiControl::gainRLQR(const Eigen::Ref<const Eigen::MatrixXd>& F, const Eigen::Ref<const Eigen::MatrixXd>& G, Eigen::Ref<Eigen::MatrixXd> K){
    // Update "left" matrix
    // Update P
    Eigen::MatrixXd invP(6,6);
    invP = this->_ftLQR.P.inverse();
    for(int it=0;it<6;it++){
        for(int jt=0;jt<6;jt++){
            this->_ftLQR.left.coeffRef(it,jt) = invP(it,jt);
        }
    }
    // Update G
    for(int it=0;it<6;it++){
        for(int jt=0;jt<NUMBER_OF_ROTORS;jt++){
            this->_ftLQR.left.coeffRef(12+NUMBER_OF_ROTORS+it,25+NUMBER_OF_ROTORS+jt) = -G(it,jt);
            this->_ftLQR.left.coeffRef(25+NUMBER_OF_ROTORS+jt,12+NUMBER_OF_ROTORS+it) = -G(it,jt);
        }
    }

    // Update "right" matrix
    // Update F
    for(int it=0;it<6;it++){
        for(int jt=0;jt<6;jt++){
            this->_ftLQR.right.coeffRef(12+NUMBER_OF_ROTORS+it,jt) = F(it,jt);
        }
    }

    // Calculate gains
    Eigen::MatrixXd gain;
    this->_ftLQR.left.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;
    solver.compute(this->_ftLQR.left);
    //Use the factors to solve the linear system 
    gain = solver.solve(this->_ftLQR.right); 

    //L = gain.middleRows<6>(19+NUMBER_OF_ROTORS);
    K = gain.middleRows<NUMBER_OF_ROTORS>(25+NUMBER_OF_ROTORS);

    Eigen::MatrixXd blockF(6,7);
    blockF.leftCols<6>() = F.transpose();
    blockF.rightCols<1>() = this->_ftLQRConst.Ef.transpose();
    this->_ftLQR.P = -gain.middleRows<6>(6+NUMBER_OF_ROTORS)+blockF*gain.middleRows<7>(12+NUMBER_OF_ROTORS);
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