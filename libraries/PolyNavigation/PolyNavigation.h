
/// @file	PolyNavigation.h
/// @brief	Library to configure and execute 5th polynomial trajectory with positions, velocities and accelerations.

#ifndef POLYNAVIGATION_H
#define POLYNAVIGATION_H

#pragma once

#include <AP_HAL/AP_HAL.h>
#include </usr/include/eigen3/Eigen/Eigen>
#include <cmath>
#include <string>
#include <iostream>

enum yawType {rot360,goTo};

class PolyNavigation
{
public:

    struct state
    {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Vector3f acceleration;
        float yaw;
    };

private:
    /* data */
    // Trajectory matrices
    Eigen::Array<float, Eigen::Dynamic, 6> _map_ax;
    Eigen::Array<float, Eigen::Dynamic, 6> _map_ay;
    Eigen::Array<float, Eigen::Dynamic, 6> _map_az;
    Eigen::Array<float, Eigen::Dynamic, 6> _map_ayaw;
    Eigen::Array<float, Eigen::Dynamic, 1> _endTime;

    // Last desired values
    state _desiredState;

    //Other
    Eigen::Array<float, 12, Eigen::Dynamic> _waypoints;  // Waypoints
    Eigen::Array<float, Eigen::Dynamic, 1> _timeTo;      // [s] Duration of the transition from the last WP to the next
    float _startTime;                      // [s] Time that the trajectory started 
    bool _isRunning = false;                        // Flag to indicate that the trajectory is running

    /* members */
    void trajectoryMatrices(float initialTime, 
                            float endTime, 
                            const Eigen::Ref<const Eigen::Array4f>& si,
                            const Eigen::Ref<const Eigen::Array4f>& dsi,
                            const Eigen::Ref<const Eigen::Array4f>& d2si, 
                            const Eigen::Ref<const Eigen::Array4f>& sf,
                            const Eigen::Ref<const Eigen::Array4f>& dsf,
                            const Eigen::Ref<const Eigen::Array4f>& d2sf,
                            Eigen::Ref<Eigen::ArrayXf> ax,
                            Eigen::Ref<Eigen::ArrayXf> ay,
                            Eigen::Ref<Eigen::ArrayXf> az,
                            Eigen::Ref<Eigen::ArrayXf> ayaw);
    Eigen::Array<float, 1, 6> polyMatrix(float initialTime,
                                    float endTime,
                                    float qi,
                                    float dqi,
                                    float d2qi,
                                    float qf,
                                    float dqf,
                                    float d2qf);
    Eigen::Array<float, 1, 4> axisDesiredTrajectory(const Eigen::Ref<const Eigen::ArrayXf>& a, float time, float initialTime, float maxTime);
public:
    PolyNavigation(/* args */);
    ~PolyNavigation();

    void geronoToWaypoints(float length, float width, float height, float endTime, int steps, 
                           yawType yawtype, float yawSp, Eigen::Ref<Eigen::MatrixXf> waypoints, Eigen::Ref<Eigen::ArrayXf> time);
    
    // Commands
    bool updateTrajectory(); 
    void start(const Eigen::Ref<const Eigen::Array3f>& initialPosition, const Eigen::Ref<const Eigen::Array3f>& initialVelocity, float initialYaw, float initialYawSpeed);
    void stop();
    void addWaypoint(const Eigen::Ref<const Eigen::Array<float, 12, 1>>& waypoint, float timeTo);
    void clear();

    //Access
    const Eigen::Vector3f getDesiredPosition() {return _desiredState.position;};
    const Eigen::Vector3f getDesiredVelocity() {return _desiredState.velocity;};
    const Eigen::Vector3f getDesiredAcceleration() {return _desiredState.acceleration;};
    float getDesiredYaw() {return _desiredState.yaw;};
    state getDesiredState() {return _desiredState;};
    bool isRunning() {return _isRunning;};
    const Eigen::Array<float, 12, Eigen::Dynamic> getWaypoints() {return _waypoints;};
    const Eigen::Array<float, Eigen::Dynamic,1> getTimeTo() {return _timeTo;};
    const std::string getWaypointsString() {std::stringstream ss; ss << _waypoints; return ss.str();};
    const std::string getTimeToString() {std::stringstream ss; ss << _timeTo; return ss.str();};
};
 
#endif