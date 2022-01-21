#include "ros/ros.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "spraygun_trigger/squeeze.h"
#include "spraygun_trigger/unsqueeze.h"
#include "spraygun_trigger/partial_squeeze.h"

class TriggerControl
{
    public:
        TriggerControl(ros::NodeHandle *nh)
            {   
                joint_single_cmd_pub= nh->advertise<interbotix_xs_msgs::JointSingleCommand>("/vx300/commands/joint_single", 10);
                squeeze_service = nh->advertiseService("gripper/squeeze", &TriggerControl::squeeze, this);
                unsqueeze_service = nh->advertiseService("gripper/unsqueeze", &TriggerControl::unsqueeze, this);
                partial_squeeze_service = nh->advertiseService("/gripper/partial_squeeze", &TriggerControl::partial_squeeze, this);
            }
    
    private:
        // motor position values for approximately unsqueeze/squeezed orientation
        // determined experimentally from viewing motor position at squeeze/unsqueeze in Dynamixel Wizard
        uint32_t squeeze_pos = 3.14; // TODO: adjust based on physical robot constrains
        uint32_t unsqueeze_pos = 1.14;  // TODO: adjust based on physical robot constrains
        interbotix_xs_msgs::JointSingleCommand jsc;

        ros::Publisher joint_single_cmd_pub;

        ros::ServiceServer squeeze_service;
        ros::ServiceServer unsqueeze_service; 
        ros::ServiceServer partial_squeeze_service; 
        
        // service callback functions 
        bool squeeze(spraygun_trigger::squeeze::Request &req,
                        spraygun_trigger::squeeze::Response &res){
            return (this->publish_trigger_cmd(squeeze_pos));
        }

        bool unsqueeze(spraygun_trigger::unsqueeze::Request &req,
                        spraygun_trigger::unsqueeze::Response &res){
            return (this->publish_trigger_cmd(unsqueeze_pos));
        }

        bool partial_squeeze(spraygun_trigger::partial_squeeze::Request &req,
                             spraygun_trigger::partial_squeeze::Response &res){
            double percentage_squeeze = ((double) req.value / (double) 255)*6.28;
            int squeeze_value = (int) (percentage_squeeze * ((int)squeeze_pos - (int)unsqueeze_pos)) + (int)unsqueeze_pos;
            return (this->publish_trigger_cmd(squeeze_value));
        }

        // publish desired end-effector motor position
        bool publish_trigger_cmd(int value){
            jsc.name = "gripper";
            jsc.cmd = value;
            
            joint_single_cmd_pub.publish(jsc);
            //TODO: Implement check to see if motor command executed succesfully
            return true;
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
