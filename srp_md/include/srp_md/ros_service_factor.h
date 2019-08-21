#ifndef SRP_MD_ROS_SERVICE_FACTOR_H_
#define SRP_MD_ROS_SERVICE_FACTOR_H_

// C++
#include <string>

// ROS
#include <ros/ros.h>

// DAI
#include <dai/util.h>
#include <dai/factor.h>

// This project
#include "srp_md/EvalFactor.h"

class RosServiceFactor : public dai::TFactor<dai::Real>
{
  private:
    // Forbid the use of the standard constructors
    RosServiceFactor() {};
    RosServiceFactor ( dai::Real p = 1 ) {};
    RosServiceFactor( const dai::Var &v ) {};
    RosServiceFactor( const dai::VarSet& vars ) {};
    RosServiceFactor( const dai::VarSet& vars, dai::Real p ) {};
    template<typename S>
    RosServiceFactor( const dai::VarSet& vars, const std::vector<S> &x ) {};
    RosServiceFactor( const dai::VarSet& vars, const dai::Real* p ) {};
    RosServiceFactor( const dai::VarSet& vars, const dai::TProb<dai::Real> &p ) {};
    RosServiceFactor( const std::vector<dai::Var> &vars, const std::vector<dai::Real> &p ) {};

  public:
    // Constructors and destructors
    RosServiceFactor(const std::string& service_name, const dai::VarSet& vars);

    // Setup ros
    // Returns false if something goes wrong otherwise true
    bool Setup();

  private:
    // ROS vars
    std::string service_name_;
    ros::ServiceClient client_;
};

#endif
