#include <tf/tf.h>
#include <iostream>
#include <distance_map_msgs/DistanceMap.h>
#include "distance_map_core/distance_map.h"

namespace utils{
    inline distmap::DistanceMap distmap_from_msg(const distance_map_msgs::DistanceMap &msg)
    {
        distmap::DistanceMap grid(distmap::DistanceMap::Dimension(msg.info.width, msg.info.height),
                                msg.info.resolution,
                                distmap::DistanceMap::Origin(msg.info.origin.position.x,
                                                            msg.info.origin.position.y,
                                                            tf::getYaw(msg.info.origin.orientation)));

        std::copy(msg.data.data(),
                msg.data.data() + (msg.info.width * msg.info.height),
                grid.data());

        return grid;
    }
}
