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
            // get link lengths
            std::vector<double> link_lens = getLinkLengths();
            
            // get base location
            Eigen::Vector2d base_loc = getBaseLocation();

            // perform forward kinematics
            Eigen::MatrixXd curr(3,3); // start with identity
            curr << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;

            int i = 0;
            while (i < joint_index+1){
                if (i == 0){
                    curr = curr*Tmatrix(state[i], 0); // first condition
                }
                else if(i == joint_index){
                    curr = curr*Tmatrix(0, link_lens[i-1]); // last condition
                }
                else{
                    curr = curr*Tmatrix(state[i], link_lens[i-1]); // algorithm
                }
                i++;
            }
            //std::cout << curr << std::endl;
            // now compute final position
            Eigen::MatrixXd final = curr*Eigen::Vector3d(base_loc.x(), base_loc.y(), 1);
            // output final[0] and final[1]
            std::cout << "joint index: " << joint_index << ", x: " << final(0) << " y: " << final(1) << std::endl;

            return Eigen::Vector2d(final(0),final(1));
        }

    // implement function here to solve inverse kinematics
        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
            // get link lengths
            std::vector<double> link_lens = getLinkLengths();
            // get base location
            Eigen::Vector2d base_loc = getBaseLocation();
            Eigen::Vector2d endLoc(end_effector_location);
            std::vector<double> link_angles;
            int count = 0;
            // if we have 2 links, we can solve for the angles
            // thus, set first angle to 0, if 3 links, to constain to 2 dof.
            if (link_lens.size() == 3){
                link_angles.push_back(0); // make it 0 
                // now we solve for new end effector position we are aiming for, with 2 dof
                endLoc = endLoc - Eigen::Vector2d(link_lens[0], 0);
                count++; // increment so that we skip the first link later
            }
            //std::cout << "New end location: x: " << endLoc.x() << " y: " << endLoc.y() << std::endl;

            double a1 = link_lens[0+count];
            double a2 = link_lens[1+count];
            //std::cout << "a1: " << a1 << " a2: " << a2 << std::endl;
            
            // now, perform inverse kinematics on 2 dof to new endLoc (maybe)
            // double temp = acos((endLoc.x()*endLoc.x() + endLoc.y()*endLoc.y() - a1*a1 - a2*a2)/(2*a1*a2));
            // double t2 = asin(sqrt(1-cos(temp)*cos(temp))); // sign check
            // std::cout << t2 << std::endl;
            
            // double temp2_cos = (endLoc.x()*(a1 + a2*cos(t2)) + endLoc.y()*a2*sin(t2))/(endLoc.x()*endLoc.x() + endLoc.y()*endLoc.y());
            // double t1 = asin(sqrt(1-temp2_cos*temp2_cos));
            // //std::cout << t1 << std::endl;

            double c2 = (endLoc.x()*endLoc.x() + endLoc.y()*endLoc.y() - a1*a1 - a2*a2)/(2*a1*a2);
            double s2 = sqrt(1-c2*c2);
            double t1 = atan2(endLoc.y(), endLoc.x()) - atan2(a2*s2, a1+a2*c2);
            double t2 = atan2(s2, c2);

            link_angles.push_back(t1);
            link_angles.push_back(t2);

            
            return link_angles;
        }

    private:
        // function to create transformation matrix
        Eigen::MatrixXd Tmatrix(double theta, double a) const{
            Eigen::MatrixXd T(3,3);
            T << cos(theta), -sin(theta), a,
                 sin(theta), cos(theta), 0,
                 0, 0, 1;
            return T;
        }
};

