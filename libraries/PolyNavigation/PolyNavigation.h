
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

enum yawType {rot360,goTo};

class PolyNavigation
{
private:
    /* data */
    // Trajectory matrices
    Eigen::Array<double, Eigen::Dynamic, 6> _map_ax;
    Eigen::Array<double, Eigen::Dynamic, 6> _map_ay;
    Eigen::Array<double, Eigen::Dynamic, 6> _map_az;
    Eigen::Array<double, Eigen::Dynamic, 6> _map_ayaw;
    Eigen::Array<double, Eigen::Dynamic, 1> _endTime;

    // Last desired values
    Eigen::Vector3d _desiredPosition;
    Eigen::Vector3d _desiredVelocity;
    Eigen::Vector3d _desiredAcceleration;
    double _desiredYaw;

    //Other
    Eigen::Array<double, 12, Eigen::Dynamic> _waypoints;  // Waypoints
    Eigen::Array<double, Eigen::Dynamic, 1> _timeTo;      // [s] Duration of the transition from the last WP to the next
    double _startTime;                      // [s] Time that the trajectory started 
    bool _isRunning;                        // Flag to indicate that the trajectory is running

    /* members */
    void trajectoryMatrices(double initialTime, 
                            double endTime, 
                            const Eigen::Ref<const Eigen::Array4d>& si,
                            const Eigen::Ref<const Eigen::Array4d>& dsi,
                            const Eigen::Ref<const Eigen::Array4d>& d2si, 
                            const Eigen::Ref<const Eigen::Array4d>& sf,
                            const Eigen::Ref<const Eigen::Array4d>& dsf,
                            const Eigen::Ref<const Eigen::Array4d>& d2sf,
                            Eigen::Ref<Eigen::ArrayXd> ax,
                            Eigen::Ref<Eigen::ArrayXd> ay,
                            Eigen::Ref<Eigen::ArrayXd> az,
                            Eigen::Ref<Eigen::ArrayXd> ayaw);
    Eigen::Array<double, 1, 6> polyMatrix(double initialTime,
                                    double endTime,
                                    double qi,
                                    double dqi,
                                    double d2qi,
                                    double qf,
                                    double dqf,
                                    double d2qf);
    Eigen::Array<double, 1, 4> axisDesiredTrajectory(const Eigen::Ref<const Eigen::ArrayXd>& a, double time, double initialTime, double maxTime);
public:
    PolyNavigation(/* args */);
    ~PolyNavigation();

    void geronoToWaypoints(double length, double width, double height, double endTime, int steps, 
                           yawType yawtype, double yawSp, Eigen::Ref<Eigen::MatrixXd> waypoints, Eigen::Ref<Eigen::ArrayXd> time);
    
    // Commands
    bool updateTrajectory(); 
    void start(const Eigen::Ref<const Eigen::Array3d>& initialPosition, const Eigen::Ref<const Eigen::Array3d>& initialVelocity, double initialYaw, double initialYawSpeed);
    void stop();
    void addWaypoint(const Eigen::Ref<const Eigen::Array<double, 12, 1>>& waypoint, double timeTo);
    void clear();

    //Access
    const Eigen::Vector3d getDesiredPosition() {return _desiredPosition;};
    const Eigen::Vector3d getDesiredVelocity() {return _desiredVelocity;};
    const Eigen::Vector3d getDesiredAcceleration() {return _desiredAcceleration;};
    const double getDesiredYaw() {return _desiredYaw;};
    const bool isRunning() {return _isRunning;};
    const Eigen::Array<double, 12, Eigen::Dynamic> getWaypoints() {return _waypoints;};
    const Eigen::Array<double, Eigen::Dynamic,1> getTimeTo() {return _timeTo;};
    const std::string getWaypointsString() {std::stringstream ss; ss << _waypoints; return ss.str();};
    const std::string getTimeToString() {std::stringstream ss; ss << _timeTo; return ss.str();};
};
 
#endif