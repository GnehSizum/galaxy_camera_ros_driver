#include "GxCamera.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "galaxy_camera_node");

    auto GalaxyCamera = std::make_shared<GxCamera>();

    ros::spin();
    
    return 0;
}
