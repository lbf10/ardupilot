# include "PolyNavigation.h"

PolyNavigation::PolyNavigation(/* args */)
{
}

PolyNavigation::~PolyNavigation()
{
}

Eigen::Array<double, 1, 6> PolyNavigation::polyMatrix(double initialTime,double endTime,double qi,double dqi,double d2qi,double qf,double dqf,double d2qf){
    Eigen::Matrix<double,6,1> q_v;
    q_v << qi,
           dqi, 
           d2qi, 
           qf, 
           dqf, 
           d2qf;
    double delta_t = (endTime-initialTime);
            
    Eigen::Matrix<double,6,6> A;
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
         1.0, delta_t, powf(delta_t,2.0), powf(delta_t,3.0), powf(delta_t,4.0), powf(delta_t,5.0),
         0.0, 1.0, 2.0*delta_t, 3.0*powf(delta_t,2.0), 4.0*powf(delta_t,3.0), 5.0*powf(delta_t,4.0),
         0.0, 0.0, 2.0, 6.0*delta_t, 12.0*powf(delta_t,2.0), 20.0*powf(delta_t,3.0);
    
    Eigen::Matrix<double,1,6> solution = A.colPivHouseholderQr().solve(q_v);
    
    Eigen::Array<double, 1, 6> a;
    a << solution.array();
    return a;
}

void PolyNavigation::trajectoryMatrices(double initialTime, double endTime, const Eigen::Ref<const Eigen::Array4d>& si,const Eigen::Ref<const Eigen::Array4d>& dsi,
                                  const Eigen::Ref<const Eigen::Array4d>& d2si, const Eigen::Ref<const Eigen::Array4d>& sf,const Eigen::Ref<const Eigen::Array4d>& dsf,
                                  const Eigen::Ref<const Eigen::Array4d>& d2sf,Eigen::Ref<Eigen::ArrayXd> ax,Eigen::Ref<Eigen::ArrayXd> ay,Eigen::Ref<Eigen::ArrayXd> az,Eigen::Ref<Eigen::ArrayXd> ayaw){
    double xo= si(0);
    double yo= si(1);
    double zo= si(2);
    double yawo = si(3);
    double xf= sf(0);
    double yf= sf(1);
    double zf= sf(2);
    double yawf = sf(3);

    double dxo= dsi(0);
    double dyo= dsi(1);
    double dzo= dsi(2);
    double dyawo = dsi(3);
    double dxf= dsf(0);
    double dyf= dsf(1);
    double dzf= dsf(2);
    double dyawf = dsf(3);
 
    double d2xo= d2si(0);
    double d2yo= d2si(1);
    double d2zo= d2si(2);
    double d2yawo = d2si(3);
    double d2xf= d2sf(0);
    double d2yf= d2sf(1);
    double d2zf= d2sf(2);
    double d2yawf = d2sf(3);

    ax = this->polyMatrix(initialTime,endTime,xo,dxo,d2xo,xf,dxf,d2xf);
    ay = this->polyMatrix(initialTime,endTime,yo,dyo,d2yo,yf,dyf,d2yf);
    az = this->polyMatrix(initialTime,endTime,zo,dzo,d2zo,zf,dzf,d2zf);
    ayaw = this->polyMatrix(initialTime,endTime,yawo,dyawo,d2yawo,yawf,dyawf,d2yawf);
};

Eigen::Array<double, 1, 4> PolyNavigation::axisDesiredTrajectory(const Eigen::Ref<const Eigen::ArrayXd>& a, double time, double initialTime, double maxTime){
    double ao = a(0);
    double a1 = a(1);
    double a2 = a(2);
    double a3 = a(3);
    double a4 = a(4);
    double a5 = a(5);
    
    double delta_t;

    if(time<=initialTime){
        delta_t = 0;
    }        
    else{
        if(time>=maxTime){
            time = maxTime;
        }
        delta_t = time-initialTime;
    }

    Eigen::Array<double, 1, 4> q;
    q << ao + a1*delta_t + a2*powf(delta_t,2.0)  + a3*powf(delta_t,3.0) + a4*powf(delta_t,4.0) + a5*powf(delta_t,5.0),
         a1 +  2.0*a2*delta_t + 3.0*a3*powf(delta_t,2.0) + 4.0*a4*powf(delta_t,3.0) + 5.0*a5*powf(delta_t,4.0),
         2.0*a2 + 6.0*a3*delta_t + 12.0*a4*powf(delta_t,2.0) + 20.0*a5*powf(delta_t,3.0),
         6.0*a3 + 24.0*a4*delta_t + 60.0*a5*powf(delta_t,2.0);

    return q;
};

void PolyNavigation::geronoToWaypoints(double length, double width, double height, double endTime, 
                                 int steps, yawType yawtype, double yawSp, 
                                 Eigen::Ref<Eigen::MatrixXd> waypoints, Eigen::Ref<Eigen::ArrayXd> time){
    double a = length/2;
    double b = width;
    double c = height/2;

    double t = endTime/steps;
    
    for(int i=0; i<steps; i++){
        //cout << i << endl;
        time(i) = t;
        //cout << i << endl;
        waypoints(0,i) = -a*cosf(2.0*M_PI*t/endTime)+a; // x
        waypoints(1,i) = -(b/2)*sinf(4.0*M_PI*t/endTime); // y
        waypoints(2,i) = c*(1-cosf(2.0*M_PI*t/endTime)); // z
        waypoints(4,i) = (2.0*M_PI*a/endTime)*sinf(2.0*M_PI*t/endTime); // dx
        waypoints(5,i) = -(2.0*M_PI*b/endTime)*cosf(4.0*M_PI*t/endTime); // dy
        waypoints(6,i) = c*(2.0*M_PI/endTime)*sinf(2.0*M_PI*t/endTime); // dz
        waypoints(8,i) = a*powf(2.0*M_PI/endTime,2)*cosf(2.0*M_PI*t/endTime); // ddx
        waypoints(9,i) = (8.0*M_PI*M_PI*b/powf(endTime,2.0))*sinf(4.0*M_PI*t/endTime); // ddy
        waypoints(10,i) = c*powf(2.0*M_PI/endTime,2)*cosf(2.0*M_PI*t/endTime); // ddz
        //cout << "foi 1" << endl;
        switch(yawtype){
            case rot360:
                waypoints(3,i) = M_PI*(1-cosf(M_PI*t/endTime)); // yaw
                waypoints(7,i) = M_PI*(M_PI/endTime)*sinf(M_PI*t/endTime); // dyaw
                waypoints(11,i) = M_PI*powf(M_PI/endTime,2)*cosf(M_PI*t/endTime); // ddyaw
                break;
            case goTo:
                waypoints(3,i) = yawSp*(1-cosf(M_PI*t/endTime))/2; // yaw
                waypoints(7,i) = yawSp*(M_PI/endTime)*sinf(M_PI*t/endTime)/2; // dyaw
                waypoints(11,i) = yawSp*powf(M_PI/endTime,2)*cosf(M_PI*t/endTime)/2; // ddyaw    
                break;
            default:
                waypoints(3,i) = yawSp; // yaw
                waypoints(7,i) = 0; // dyaw
                waypoints(11,i) = 0; // ddyaw
        }
        t = t + endTime/steps;
    }
    //cout << "foi 2" << endl;
};

void PolyNavigation::start(const Eigen::Ref<const Eigen::Array3d>& initialPosition, 
                             const Eigen::Ref<const Eigen::Array3d>& initialVelocity, double initialYaw, double initialYawSpeed){
    Eigen::Array4d si;
    Eigen::Array4d dsi;
    Eigen::Array4d d2si; 
    Eigen::Array4d sf;
    Eigen::Array4d dsf;
    Eigen::Array4d d2sf;
    double ti, tf;

    Eigen::ArrayXd ax = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd ay = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd az = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd ayaw = Eigen::ArrayXd::Zero(6);

    this->_endTime.resize(this->_timeTo.size());
    this->_map_ax.resize(this->_timeTo.size(),6);
    this->_map_ay.resize(this->_timeTo.size(),6);
    this->_map_az.resize(this->_timeTo.size(),6);
    this->_map_ayaw.resize(this->_timeTo.size(),6);
    
    si << initialPosition, initialYaw;
    dsi << initialVelocity, initialYawSpeed;
    d2si << 0.0, 0.0, 0.0, 0.0;
    ti = 0;
    for(int it=0;it<this->_waypoints.cols();it++){
        sf << this->_waypoints(0,it), this->_waypoints(1,it), this->_waypoints(2,it), this->_waypoints(3,it);
        dsf << this->_waypoints(4,it), this->_waypoints(5,it), this->_waypoints(6,it), this->_waypoints(7,it);
        d2sf << this->_waypoints(8,it), this->_waypoints(9,it), this->_waypoints(10,it), this->_waypoints(11,it);
        tf += this->_timeTo(it);
        this->_endTime(it) = tf;
        this->trajectoryMatrices(ti,tf,si,dsi,d2si,sf,dsf,d2sf,ax,ay,az,ayaw);
        this->_map_ax.row(it)=ax;
        this->_map_ay.row(it)=ay;
        this->_map_az.row(it)=az;
        this->_map_ayaw.row(it)=ayaw;
        si = sf;
        dsi = dsf;
        d2si = d2sf;
        ti = tf;
    }

    this->_startTime = AP_HAL::millis()/1000.0;
    this->_isRunning = true;
};

bool PolyNavigation::updateTrajectory(){
    if(this->_isRunning && this->_startTime!=NULL){
        double time;
        time = AP_HAL::millis()/1000.0 - this->_startTime;
        
        int index=0;
        for(index=0; index<this->_endTime.size();index++){
            if(time <= this->_endTime(index)){
                break;
            }
        }

        if(index>=this->_endTime.size())
        {
            index=this->_endTime.size()-1;
        }

        double ti;
        if(index==0){
            ti = 0;
        }
        else{
            ti = this->_endTime(index-1);
        }
        
        Eigen::Array<double, 1, 4> desTrajx;
        Eigen::Array<double, 1, 4> desTrajy;
        Eigen::Array<double, 1, 4> desTrajz;
        Eigen::Array<double, 1, 4> desTrajyaw;
        desTrajx = axisDesiredTrajectory(this->_map_ax.row(index),time,ti,this->_endTime(index));
        desTrajy = axisDesiredTrajectory(this->_map_ay.row(index),time,ti,this->_endTime(index));
        desTrajz = axisDesiredTrajectory(this->_map_az.row(index),time,ti,this->_endTime(index));
        desTrajyaw = axisDesiredTrajectory(this->_map_ayaw.row(index),time,ti,this->_endTime(index));

        this->_desiredPosition << desTrajx(0),desTrajy(0),desTrajz(0);
        this->_desiredVelocity << desTrajx(1),desTrajy(1),desTrajz(1);
        this->_desiredAcceleration << desTrajx(2),desTrajy(2),desTrajz(2);
        this->_desiredYaw = desTrajyaw(0);

        return true;
    }
    else{
        return false;
    }
};

void PolyNavigation::stop(){
    this->_isRunning = false;
};

void PolyNavigation::addWaypoint(const Eigen::Ref<const Eigen::Array<double, 12, 1>>& waypoint, double timeTo){
    this->_waypoints.conservativeResize(Eigen::NoChange,this->_waypoints.cols()+1);
    this->_waypoints.rightCols(1) = waypoint;
    this->_timeTo.conservativeResize(this->_timeTo.size()+1);
    this->_timeTo.tail(1) = timeTo;
};

void PolyNavigation::clear(){
    this->_waypoints.resize(12,0);
    this->_timeTo.resize(0);
    this->_endTime.resize(0);
    this->_startTime = NULL;
    this->_map_ax.resize(0,6);
    this->_map_ay.resize(0,6);
    this->_map_az.resize(0,6);
    this->_map_ayaw.resize(0,6);
};



            