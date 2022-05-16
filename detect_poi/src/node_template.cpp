#include <node_template.h>

template <typename T>
int NodeMain(int argc, char **argv, std::string const &nodeName)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    T a(nh, pnh);

    ros::spin();
    return EXIT_SUCCESS;
}
