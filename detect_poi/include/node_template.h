#ifndef NODE_TEMPLATE_H
#define NODE_TEMPLATE_H

#include <ros/ros.h>

template <typename T>
int NodeMain(int argc, char **argv, std::string const &nodeName);

#endif // NODE_TEMPLATE_H
