# include "MultiControl.h"
#include <ctime>

//////////////////////////////////////////////
/* Multicopter configuration default values */
#define MASS    6.015 
#define MOMENTS_OF_INERTIA_XX 0.3143978800
#define MOMENTS_OF_INERTIA_YY 0.3122127800
#define MOMENTS_OF_INERTIA_ZZ 0.5557912400
#define PRODUCTS_OF_INERTIA_XY 0.0000861200
#define PRODUCTS_OF_INERTIA_XZ -0.0014397600
#define PRODUCTS_OF_INERTIA_YZ 0.0002368800
#define MAIN_TRANSLATIONAL_FRICTION_X 0.25
#define MAIN_TRANSLATIONAL_FRICTION_Y 0.25
#define MAIN_TRANSLATIONAL_FRICTION_Z 0.25
#define ROTOR_INERTIA 0.00047935
#define ROTOR_LIFT_COEFF 6.97e-5
#define ROTOR_DRAG_COEFF 1.033e-6
#define ROTOR_MAX_SPEED 729.0
#define ROTOR_MAX_SPEED_SQUARED (double) ROTOR_MAX_SPEED*ROTOR_MAX_SPEED
#define ROTOR_MIN_SPEED 0.0
#define ROTOR_MIN_SPEED_SQUARED (double) ROTOR_MIN_SPEED*ROTOR_MIN_SPEED
#define ROTOR_OPERATING_POINT 340.0
#define ROTOR_OPERATING_POINT_SQUARED (double) 0.5*(ROTOR_MAX_SPEED_SQUARED+ROTOR_MIN_SPEED_SQUARED)
#define ROTOR_RM 0.0975
#define ROTOR_L 0.0033
#define ROTOR_KT 0.02498
#define ROTOR_KV 340.0
#define ROTOR_IO 0.6/10.0
#define ROTOR_MAX_VOLTAGE 22.0

#define ROTOR_POSITION 0.33920, -0.33742, 0.09298, \
                       0.33887, 0.33758, 0.09298, \
                       -0.33613, 0.33726, 0.09298, \
                        -0.33581, -0.33775, 0.09298, \
                        0.34364, 0.34235, 0.01597, \
                       0.34396, -0.34219, 0.01597, \
                       -0.34057, -0.34251, 0.01597, \
                        -0.34090, 0.34202, 0.01597

#define ROTOR_ORIENTATION -6.179033021212912e-02, 6.146607671042632e-02, 9.961946980917455e-01, \
                           -6.174583072871852e-02, -6.151077857998878e-02, 9.961946980917455e-01, \
                           6.152491288718258e-02, -6.173174700363311e-02, 9.961946980917455e-01, \
                          6.145065852146369e-02, 6.180566366583593e-02, 9.961946980917455e-01, \
                          -6.174419940539115e-02, -6.151241609369010e-02, 9.961946980917455e-01, \
                           -6.178718847226844e-02, 6.146923486255825e-02, 9.961946980917455e-01, \
                           6.145313940784528e-02, 6.180319693038461e-02, 9.961946980917455e-01, \
                          6.152726235862697e-02, -6.172940531504134e-02, 9.961946980917455e-01

#define ROTOR_DIRECTION 1, -1, 1, -1, 1, -1, 1, -1

//////////////////////////////////////
/* General variables default values */
#define VELOCITY_FILTER_GAIN_X 0.999
#define VELOCITY_FILTER_GAIN_Y 0.999
#define VELOCITY_FILTER_GAIN_Z 0.999

#define CONTROL_TIME_STEP 1/400.0

/////////////////////////////////////////
/* FT-LQR configuration default values */
#define FTLQR_CONFIG_P_DIAG 5.0e+10, 5.0e+10, 5.0e+05, 5.0e+04, 5.0e+04, 5.0e+09
#define FTLQR_CONFIG_Q_DIAG 5.0e+10, 5.0e+10, 5.0e+05, 5.0e+04, 5.0e+04, 5.0e+09
#define FTLQR_CONFIG_R_DIAG 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05
#define FTLQR_CONFIG_EF_ROW 1, 1, 1, 1, 1, 1
#define FTLQR_CONFIG_EG_ROW 1, 1, 1, 1, 1, 1, 1, 1
#define FTLQR_CONFIG_H_COL 1, 1, 1, 1, 1, 1

#define FTLQR_CONFIG_MU 1.0e20
#define FTLQR_CONFIG_ALPHA 1.5

//////////////////////////////////////////////////////////////////
/* Passive NMAC Control allocation configuration default values */
#define CA_WM 1.0
#define CA_WA 0.0

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
    std::string filename_save;   
    time_t now = std::time(0);
    char* dt = std::ctime(&now);
    filename_save.append("logs/log ");
    filename_save.append(dt);
    filename_save.pop_back();
    filename_save.append(".txt");
    dumpfile.open(filename_save);
    // Control time step
    this->_controlTimeStep = CONTROL_TIME_STEP;
    this->_measuredTimeStep = CONTROL_TIME_STEP;
    this->_lastCall = 0.0;

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
    this->_piddConst.Kp << (double) _PIDD_KP_X, (double) _PIDD_KP_Y, (double) _PIDD_KP_Z;
    this->_piddConst.Ki << (double) _PIDD_KI_X, (double) _PIDD_KI_Y, (double) _PIDD_KI_Z;
    this->_piddConst.Kd << (double) _PIDD_KD_X, (double) _PIDD_KD_Y, (double) _PIDD_KD_Z;
    this->_piddConst.Kdd << (double) _PIDD_KDD_X, (double) _PIDD_KDD_Y, (double) _PIDD_KDD_Z;

    /* std::cout << "kp:" << this->_piddConst.Kp(0) << " ! " << this->_piddConst.Kp(1) << " ! " << this->_piddConst.Kp(2) << std::endl;
    std::cout << "ki:" << this->_piddConst.Ki(0) << " ! " << this->_piddConst.Ki(1) << " ! " << this->_piddConst.Ki(2) << std::endl;
    std::cout << "kd:" << this->_piddConst.Kd(0) << " ! " << this->_piddConst.Kd(1) << " ! " << this->_piddConst.Kd(2) << std::endl;
    std::cout << "kdd:" << this->_piddConst.Kdd(0) << " ! " << this->_piddConst.Kdd(1) << " ! " << this->_piddConst.Kdd(2) << std::endl;*/

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
    this->_velFilter.previousAngularVelocity.setZero();
    this->_velFilter.desiredAngularVelocity.setZero();
    this->_velFilter.desiredAngularAcceleration.setZero();

    // Zero rotor states
    this->_currentRotorSpeeds.setZero();

    ////////////////////
    /* FT-LQR related */
    this->_ftLQR.gainRotorSpeeds.setZero();
    this->_ftLQR.desiredGainAngularAcceleration.setZero();

    // Calculates ssB
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
    Eigen::Matrix<double,6,1> H;
    H << FTLQR_CONFIG_H_COL;
    // lambda
    double lambda = FTLQR_CONFIG_ALPHA*FTLQR_CONFIG_MU*abs(H.transpose()*H);
    // middle
    Eigen::Matrix<double,6,6> middle;
    middle = Eigen::Matrix<double,6,6>::Ones()/FTLQR_CONFIG_MU-(H*H.transpose())/lambda;
    // inv R
    Eigen::VectorXd invR(Raux.cols());
    invR = Raux.inverse().diagonal();
    // inv Q
    Eigen::VectorXd invQ(Qaux.cols());
    invQ = Qaux.inverse().diagonal();
    // inv P
    Eigen::Matrix<double,6,1> invP;
    invP = this->_ftLQR.P.inverse().diagonal();
    // Eg
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> Eg;
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

    this->_desiredRotorSpeeds.setZero();
    this->_velocityFilterGain << VELOCITY_FILTER_GAIN_X, VELOCITY_FILTER_GAIN_Y, VELOCITY_FILTER_GAIN_Z;
    return true;
};

//extern const AP_HAL::HAL& hal; 

bool MultiControl::updateStates(PolyNavigation::state desiredState){
    // Update desired states from navigation
    this->_desiredPosition << (double)desiredState.position.x(), (double)desiredState.position.y(), (double)desiredState.position.z();
    this->_desiredVelocity << (double)desiredState.velocity.x(), (double)desiredState.velocity.y(), (double)desiredState.velocity.z();
    this->_desiredAcceleration << (double)desiredState.acceleration.x(), (double)desiredState.acceleration.y(), (double)desiredState.acceleration.z();
    this->_desiredYaw = (double) desiredState.yaw;
    this->_currentRotorSpeeds = this->_desiredRotorSpeeds;

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
    this->_velFilter.previousAngularVelocity = this->_currentAngularVelocity;
    this->_currentAngularVelocity << (double) this->_vectorAux.x, (double) -this->_vectorAux.y, (double) -this->_vectorAux.z;
    updateVectorHistory(this->_currentAngularVelocity,this->_velFilter.angularVelocity);
    firstDerivative(this->_velFilter.angularVelocity,this->_currentAngularAcceleration);

    // Update control time step
    double thisCall = (double) AP_HAL::millis()/1000.0;
    if(this->_lastCall>=1e-60){
        this->_measuredTimeStep = 0.5*(this->_measuredTimeStep+thisCall-this->_lastCall);
    }
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
    if(this->_desiredForce(2)<0.0){
        this->_desiredForce(2) = -this->_desiredForce(2);
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

    angularVelocity = this->_currentAngularVelocity;
       
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

    this->auxQuaternion = qe;
            
    Eigen::MatrixXd qeVec(3,1);
    qeVec = qe.vec();
    this->_velFilter.desiredAngularVelocity = (this->_velocityFilterGain.array()*this->_velFilter.desiredAngularVelocity.array()+(Eigen::Array3d::Ones()-this->_velocityFilterGain.array())*(qeVec.array()/this->_controlTimeStep)).matrix();
    angularAcceleration = this->_currentAngularAcceleration;
    updateVectorHistory((this->_velFilter.desiredAngularVelocity-angularVelocity),this->_velFilter.Wbe);
    firstDerivative(this->_velFilter.Wbe,dWbe);
    desiredAngularAcceleration = (dWbe+angularAcceleration);
    this->_velFilter.desiredAngularAcceleration = desiredAngularAcceleration;
    Eigen::MatrixXd torqueAux(3,1);
    torqueAux.setZero();
    for(int it=0;it<NUMBER_OF_ROTORS;it++){
        torqueAux = torqueAux + this->_rotorOrientation.col(it)*this->_currentRotorSpeeds(it)*ROTOR_INERTIA;
    }
    torqueAux = this->_inertia*this->_currentAngularVelocity-torqueAux; 
    Eigen::MatrixXd auxA(3,3);
    auxA <<             0, -torqueAux(2),  torqueAux(1),
             torqueAux(2),             0, -torqueAux(0),
            -torqueAux(1),  torqueAux(0),             0;
    auxA = this->_inertia.inverse()*auxA;
    
    Eigen::MatrixXd Sq(3,3);
    Sq << qe.w(), -qe.z(),  qe.y(),
          qe.z(),  qe.w(), -qe.x(),
         -qe.y(),  qe.x(),  qe.w();

    Eigen::MatrixXd x_e(6,1);
    x_e << this->_velFilter.desiredAngularVelocity-this->_currentAngularVelocity, qe.vec();
    
    Eigen::MatrixXd ssA(6,6);
    ssA.setZero();
    ssA.topLeftCorner<3,3>() = auxA;
    ssA.bottomLeftCorner<3,3>() = 0.5*Sq;

    Eigen::MatrixXd F(6,6);
    Eigen::MatrixXd G(6,NUMBER_OF_ROTORS);
    c2d(ssA, this->_ftLQRConst.ssB,this->_controlTimeStep,F,G);

    // ssA.resize(0,0);
    // Sq.resize(0,0);
    // torqueAux.resize(0,0);
    // qeVec.resize(0,0);

    Eigen::MatrixXd K(NUMBER_OF_ROTORS,6);
    gainRLQR(F,G,K);

    Eigen::MatrixXd gainSpeeds(NUMBER_OF_ROTORS,1);
    //Eigen::MatrixXd gainSpeeds_sqrt(NUMBER_OF_ROTORS,1);
    gainSpeeds << K*x_e;
    //gainSpeeds_sqrt << (this->_rotorDirection.array()*gainSpeeds.array().sqrt()).matrix();
    //this->_ftLQR.gainRotorSpeeds = gainSpeeds_sqrt;
    //K.resize(0,0);
    this->_desiredTorque = -this->_inertia*(this->_ftLQRConst.ssB.topRows<3>()*gainSpeeds-desiredAngularAcceleration+auxA*this->_velFilter.desiredAngularVelocity);  
    return true;
};

bool MultiControl::controlAllocation(){
    this->_v << this->_attRefAux*(((double)CA_WM)*ROTOR_OPERATING_POINT_SQUARED+(this->_Mf.transpose()*((double)CA_WA)*this->_desiredForce).array()).matrix();
    Eigen::VectorXd b2(NUMBER_OF_ROTORS);
    b2 << this->_nullMt*this->_v;
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> utau;
    utau << this->_pinvMt*this->_desiredTorque;
    Eigen::Vector3d aux;
    aux << this->_Mf*b2;
    double c = (this->_desiredForce-this->_Mf*utau).transpose()*aux;
    c = c/(aux.transpose()*aux);

    this->_omegaSquared =  utau+b2*c;

    for(int it=0;it<this->_omegaSquared.size();it++){
        if(this->_omegaSquared(it)<ROTOR_MIN_SPEED_SQUARED){
            this->_omegaSquared(it) = ROTOR_MIN_SPEED_SQUARED;
        }
        else if(this->_omegaSquared(it)>ROTOR_MAX_SPEED_SQUARED){
            this->_omegaSquared(it) = ROTOR_MAX_SPEED_SQUARED;
        }
    }

    this->_desiredRotorSpeeds = (this->_rotorDirection.array()*this->_omegaSquared.array().sqrt()).matrix();
    this->_desiredRotorVoltages = (ROTOR_RM*ROTOR_DRAG_COEFF*this->_desiredRotorSpeeds.array()*this->_desiredRotorSpeeds.array().abs()/ROTOR_KT+60*this->_desiredRotorSpeeds.array()/(ROTOR_KV*M_PI*2)).matrix();
    return true;
};

/* Auxiliary private methods */
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
// C2D: Richard J. Gran; Numerical Computing with Simulink, Volume 1: Creating Simulations.
// https://books.google.com.br/books?id=IeGQP05ZnTUC&pg=PA37&lpg=PA37&dq=how+matlab+computes+B+matrix+in+c2d&source=bl&ots=rcg8Eep-on&sig=ACfU3U1GtU_FevCqiQjLKllc51ZOfXPtzQ&hl=pt-BR&sa=X&ved=2ahUKEwikyJTExpjpAhXEHrkGHfaZA28Q6AEwB3oECAgQAQ#v=onepage&q=how%20matlab%20computes%20B%20matrix%20in%20c2d&f=false
// Matrix exponential in Eigen: https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html#matrixbase_exp
// Matrix exponential in Matlab: https://www.mathworks.com/help/matlab/ref/expm.html
void MultiControl::c2d(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>& Ac,const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>& Bc, 
                        double ts, Eigen::Ref<Eigen::MatrixXd> Ad, Eigen::Ref<Eigen::MatrixXd> Bd){
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> augmented(Ac.rows()+Bc.cols(),Ac.cols()+Bc.cols());
    augmented.setZero();
    augmented.topLeftCorner(Ac.rows(),Ac.cols()) = Ac.cast<long double>();
    augmented.topRightCorner(Bc.rows(),Bc.cols()) = Bc.cast<long double>();
    augmented = augmented*(long double)ts;
    Eigen::MatrixExponentialReturnValue<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>> result(augmented);
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> output;
    output.resizeLike(augmented);
    result.evalTo(output);
    Ad = output.topLeftCorner(Ac.rows(),Ac.rows()).cast<double>();
    Bd = output.topRightCorner(Bc.rows(),Bc.cols()).cast<double>();
};

/* Calculates Robust LQR */
// References:
// Joao Paulo Cerri, Master`s
void MultiControl::gainRLQR(Eigen::Ref<Eigen::MatrixXd> F, Eigen::Ref<Eigen::MatrixXd> G, Eigen::Ref<Eigen::MatrixXd> K){
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

    Eigen::MatrixXd blockF(6,7);
    blockF.leftCols<6>() = F.transpose();
    blockF.rightCols<1>() = this->_ftLQRConst.Ef.transpose();
    
    this->_ftLQR.P = -gain.middleRows<6>(6+NUMBER_OF_ROTORS)+blockF*gain.middleRows<7>(12+NUMBER_OF_ROTORS);
    blockF.resize(0,0);
    //L = gain.middleRows<6>(19+NUMBER_OF_ROTORS);
    K = gain.middleRows<NUMBER_OF_ROTORS>(25+NUMBER_OF_ROTORS);
};


void MultiControl::updateVectorHistory(const Eigen::Ref<const Eigen::VectorXd>& newValue, Eigen::Ref<Eigen::MatrixXd> vectorHistory){
    for(int it=0;it<DERIVATIVE_WINDOW_SIZE-1;it++){
        vectorHistory.col(it) = vectorHistory.col(it+1);
    }
    vectorHistory.rightCols(1) << newValue;
};

void MultiControl::firstDerivative(const Eigen::Ref<const Eigen::MatrixXd>& vectorHistory, Eigen::Ref<Eigen::VectorXd> derivative){
    Eigen::Matrix<double, 4, DERIVATIVE_WINDOW_SIZE> MQmatrix;
    MQmatrix << 0.985714285714285, 0.057142857142858, -0.085714285714286, 0.057142857142856, -0.014285714285712,
               -1.488095238095233, 1.619047619047610,  0.571428571428571,-1.047619047619037,  0.345238095238088,
                0.642857142857139,-1.071428571428565, -0.142857142857142, 0.928571428571423, -0.357142857142854,
               -0.083333333333333, 0.166666666666666,  0.000000000000000,-0.166666666666665,  0.083333333333333;
    Eigen::Vector4d theta;
    int numberOfVariables = vectorHistory.rows(); 
    derivative.resize(numberOfVariables);
    for(int it=0;it<numberOfVariables;it++){
        theta << MQmatrix*((vectorHistory.row(it)).transpose());
        derivative(it) = theta(1)+8*theta(2)+48*theta(3);
    }    
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
        float *returnValue = new float [this->_currentRotorSpeeds.size()];
        for(int it=0;it<this->_currentRotorSpeeds.size();it++){
            returnValue[it] = (float) this->_currentRotorSpeeds(it);
        }
        return returnValue;
    };

    float* MultiControl::desiredRotorSpeeds(){
        float *returnValue = new float [this->_desiredRotorSpeeds.size()];
        for(int it=0;it<this->_desiredRotorSpeeds.size();it++){
            returnValue[it] = (float) this->_desiredRotorSpeeds(it);
        }
        return returnValue;
    };

    float* MultiControl::desiredRotorVoltages(){
        float *returnValue = new float [this->_desiredRotorVoltages.size()];
        for(int it=0;it<this->_desiredRotorVoltages.size();it++){
            returnValue[it] = (float) this->_desiredRotorVoltages(it);
        }
        return returnValue;
    };

const AP_Param::GroupInfo MultiControl::var_info[] = {
    // @Param: KP_X
    // @DisplayName: PIDD KP X
    // @Description: PIDD controller P gain in the X axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KP_X", 0, MultiControl, _PIDD_KP_X, PIDD_KP_X),
    // @Param: KP_Y
    // @DisplayName: PIDD KP Y
    // @Description: PIDD controller P gain in the Y axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KP_Y", 1, MultiControl, _PIDD_KP_Y, PIDD_KP_Y),
    // @Param: KP_Z
    // @DisplayName: PIDD KP Z
    // @Description: PIDD controller P gain in the Z axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KP_Z", 2, MultiControl, _PIDD_KP_Z, PIDD_KP_Z),
    // @Param: KI_X
    // @DisplayName: PIDD KI X
    // @Description: PIDD controller I gain in the X axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KI_X", 3, MultiControl, _PIDD_KI_X, PIDD_KI_X),
    // @Param: KI_Y
    // @DisplayName: PIDD KI Y
    // @Description: PIDD controller I gain in the Y axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KI_Y", 4, MultiControl, _PIDD_KI_Y, PIDD_KI_Y),
    // @Param: KI_Z
    // @DisplayName: PIDD KI Z
    // @Description: PIDD controller I gain in the Z axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KI_Z", 5, MultiControl, _PIDD_KI_Z, PIDD_KI_Z),
    // @Param: KD_X
    // @DisplayName: PIDD KD X
    // @Description: PIDD controller D gain in the X axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KD_X", 6, MultiControl, _PIDD_KD_X, PIDD_KD_X),
    // @Param: KD_Y
    // @DisplayName: PIDD KD Y
    // @Description: PIDD controller D gain in the Y axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KD_Y", 7, MultiControl, _PIDD_KD_Y, PIDD_KD_Y),
    // @Param: KD_Z
    // @DisplayName: PIDD KD Z
    // @Description: PIDD controller D gain in the Z axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KD_Z", 8, MultiControl, _PIDD_KD_Z, PIDD_KD_Z),
    // @Param: KDD_X
    // @DisplayName: PIDD KDD X
    // @Description: PIDD controller DD gain in the X axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KDD_X", 9, MultiControl, _PIDD_KDD_X, PIDD_KDD_X),
    // @Param: KDD_Y
    // @DisplayName: PIDD KDD Y
    // @Description: PIDD controller DD gain in the Y axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KDD_Y", 10, MultiControl, _PIDD_KDD_Y, PIDD_KDD_Y),
    // @Param: KDD_Z
    // @DisplayName: PIDD KDD Z
    // @Description: PIDD controller DD gain in the Z axis.
    // @Range: -1000 1000
    AP_GROUPINFO("KDD_Z", 11, MultiControl, _PIDD_KDD_Z, PIDD_KDD_Z),

    AP_GROUPEND
};