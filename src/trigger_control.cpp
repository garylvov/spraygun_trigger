#include "ros/ros.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "trigger_control/squeeze.h"
#include "trigger_control/unsqueeze.h"
#include "trigger_control/timed_squeeze.h"
#include "trigger_control/partial_squeeze.h"

class TriggerControl
{
    public:
        TriggerControl(ros::NodeHandle *nh)
            {   
                single_motor_control_pub= nh->advertise<interbotix_xs_msgs::JointSingleCommand>("/vx300/commands/joint_single", 10);
                squeeze_service = nh->advertiseService("gripper/squeeze", &Gripper::squeeze, this);
                unsqueeze_service = nh->advertiseService("gripper/unsqueeze", &Gripper::unsqueeze, this);
                timed_squeeze_service = nh->advertiseService("gripper/timed_squeeze", &Gripper::timed_squeeze, this);
                partial_squeeze_service = nh->advertiseService("/gripper/partial_squeeze", &Gripper::partial_squeeze, this);
            }
    
    private:
        bool state; // this->state is true if squeezed, false if unsqueezed

        // motor position values for approximately unsqueeze/squeezed orientation
        // determined experimentally from viewing motor position at squeeze/unsqueeze in Dynamixel Wizard
        uint32_t squeeze_pos = 340; // TODO: ADJUST ON PHYSICAL ROBOT
        uint32_t unsqueeze_pos = 687;  // TODO: ADJUST ON PHYSICAL ROBOT

        ros::Publisher single_motor_control_pub;

        ros::ServiceServer squeeze_service;
        ros::ServiceServer unsqueeze_service; 
        ros::ServiceServer timed_squeeze_service; 
        ros::ServiceServer partial_squeeze_service; 
        
        // service callback functions 
        bool squeeze(single_motor::squeeze::Request &req,
                        single_motor::squeeze::Response &res){
            this->state = true;
            return (this->send_motor_request(squeeze_pos));
        }

        bool unsqueeze(single_motor::unsqueeze::Request &req,
                        single_motor::unsqueeze::Response &res){
            this->state = false;
            return (this->send_motor_request(unsqueeze_pos));
        }

        bool timed_squeeze(single_motor::timed_squeeze::Request &req,
                            single_motor::timed_squeeze::Response &res){
            if (this->state){ // this->state is true, so gripper is squeezed, to timed_squeeze unsqueeze gripper
                this->state = false;
                return (this->send_motor_request(unsqueeze_pos));
            }
            else{ // this->state is false, so gripper is squeezed, to timed_squeeze squeeze gripper
                this->state = true;
                return (this->send_motor_request(squeeze_pos));
            }
        }

        bool partial_squeeze(single_motor::partial_squeeze::Request &req,
                             single_motor::partial_squeeze::Response &res){
            /* This is meant for partial_squeeze closing upon fragile objects that would be damaged 
               by a full squeeze. The this->state is changed to be squeezed so that the timed_squeeze function still works
               within this context - toggling from a partial_squeeze squeeze unsqueezes the gripper. */
            double percentage_squeeze = (double) req.value / (double) 255;
            int squeeze_value = (int) (percentage_squeeze * ((int)squeeze_pos - (int)unsqueeze_pos)) + (int)unsqueeze_pos;
            this->state = true; 
            return (this->send_motor_request(squeeze_value));
        }

        // client function : communicates with the dynamixel workbench to control the motor
        bool send_motor_request(int value){
            std::cout<<"test"<<endl;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger_control");
    ros::NodeHandle nh;
    TriggerControl tc = TriggerControl(&nh);
    ros::spin();
    return 0;
}
