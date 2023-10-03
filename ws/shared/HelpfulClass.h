#include "AMPCore.h"

class MyClass {
    public:
        void hereIsAMethod();
};

class config : public amp::ConfigurationSpace2D{
    public:
        config(double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max){}

        bool inCollision(double x0, double x1) const{ 
            return false;
        }
};

class Link2d : public amp::LinkManipulator2D{
    public:
        // declare class
        Link2d() : amp::LinkManipulator2D(){}
        Link2d(const std::vector<double>& link_lengths) : amp::LinkManipulator2D(link_lengths){}
        Link2d(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths) : amp::LinkManipulator2D(base_location, link_lengths){}
        
    // implement funciton here to solve forward kinematics
        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{

            return Eigen::Vector2d(0,0);
        }

    // implement function here to solve inverse kinematics
        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
            
            return amp::ManipulatorState();
        }
};