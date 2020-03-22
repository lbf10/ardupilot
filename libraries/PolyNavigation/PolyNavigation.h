
/// @file	PolyNavigation.h
/// @brief	Library to configure and execute 5th polynomial trajectory with positions, velocities and accelerations.

#ifndef POLYNAVIGATION_H
#define POLYNAVIGATION_H

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <iostream>

using namespace Eigen;
using namespace std;

enum yawType {rot360,goTo};

class PolyNavigation
{
private:
    /* data */
    // Trajectory matrices
    Array<double, Dynamic, 6> _map_ax;
    Array<double, Dynamic, 6> _map_ay;
    Array<double, Dynamic, 6> _map_az;
    Array<double, Dynamic, 6> _map_ayaw;
    Array<double, Dynamic, 1> _endTime;

    // Last desired values
    Vector3d _desiredPosition;
    Vector3d _desiredVelocity;
    Vector3d _desiredAcceleration;
    double _desiredYaw;

    //Other
    Array<double, 12, Dynamic> _waypoints;  // Waypoints
    Array<double, Dynamic, 1> _timeTo;      // [s] Duration of the transition from the last WP to the next
    double _startTime;                      // [s] Time that the trajectory started 
    bool _isRunning;                        // Flag to indicate that the trajectory is running

    /* members */
    void trajectoryMatrices(double initialTime, 
                            double endTime, 
                            const Ref<const Array4d>& si,
                            const Ref<const Array4d>& dsi,
                            const Ref<const Array4d>& d2si, 
                            const Ref<const Array4d>& sf,
                            const Ref<const Array4d>& dsf,
                            const Ref<const Array4d>& d2sf,
                            Ref<ArrayXd> ax,
                            Ref<ArrayXd> ay,
                            Ref<ArrayXd> az,
                            Ref<ArrayXd> ayaw);
    Array<double, 1, 6> polyMatrix(double initialTime,
                                    double endTime,
                                    double qi,
                                    double dqi,
                                    double d2qi,
                                    double qf,
                                    double dqf,
                                    double d2qf);
    Array<double, 1, 4> axisDesiredTrajectory(const Ref<const ArrayXd>& a, double time, double initialTime, double maxTime);
public:
    PolyNavigation(/* args */);
    ~PolyNavigation();

    void geronoToWaypoints(double length, double width, double height, double endTime, int steps, 
                           yawType yawtype, double yawSp, Ref<MatrixXd> waypoints, Ref<ArrayXd> time);
    
    // Commands
    bool updateTrajectory(); 
    void start(const Ref<const Array3d>& initialPosition, const Ref<const Array3d>& initialVelocity, double initialYaw, double initialYawSpeed);
    void stop();
    void addWaypoint(const Ref<const Array<double, 12, 1>>& waypoint, double timeTo);
    void clear();

    //Access
    const Vector3d getDesiredPosition() {return _desiredPosition;};
    const Vector3d getDesiredVelocity() {return _desiredVelocity;};
    const Vector3d getDesiredAcceleration() {return _desiredAcceleration;};
    const double getDesiredYaw() {return _desiredYaw;};
    const bool isRunning() {return _isRunning;};
    const Array<double, 12, Dynamic> getWaypoints() {return _waypoints;};
    const Array<double, Dynamic,1> getTimeTo() {return _timeTo;};
    const std::string getWaypointsString() {std::stringstream ss; ss << _waypoints; return ss.str();};
    const std::string getTimeToString() {std::stringstream ss; ss << _timeTo; return ss.str();};
};
 
#endif