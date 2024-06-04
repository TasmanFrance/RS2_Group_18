#include "rclcpp/rclcpp.hpp"
#include "arm_controller.h"


///////////////////////////////////////////////////////////////////////////////


int main(int argc, char * argv[])
{
/*---------------------------------------------------------------------------*/
    //Initialize ROS and create the Node
    rclcpp::init(argc, argv);

/*---------------------------------------------------------------------------*/
    //Run arm_controller until stopped
    rclcpp::spin(std::make_shared<ArmController>());

/*---------------------------------------------------------------------------*/
    //Shutdown ROS
    rclcpp::shutdown();

/*---------------------------------------------------------------------------*/
    return 0;
}


///////////////////////////////////////////////////////////////////////////////