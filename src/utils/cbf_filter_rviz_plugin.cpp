#include "utils/cbf_filter_rviz_plugin.h"

namespace cbf_filter_rviz_plugin
{
    CBFRVizPlugin::CBFRVizPlugin()
    {
    }

    void CBFRVizPlugin::onInitialize()
    {

        twist_topic1_ = new rviz::StringProperty("Twist Topic 1", "twist1", "The topic on which to subscribe for twist 1", this, SLOT(updateTopic1()));
        twist_topic2_ = new rviz::StringProperty("Twist Topic 2", "twist2", "The topic on which to subscribe for twist 2", this, SLOT(updateTopic2()));

        twist_sub_1_ = nh_.subscribe(twist_topic1_->getStdString(), 1, &CBFRVizPlugin::twistCallback1, this);
        twist_sub_2_ = nh_.subscribe(twist_topic2_->getStdString(), 1, &CBFRVizPlugin::twistCallback2, this);

        vis_manager_ = context_->getVisualizationManager();
    }

    void CBFRVizPlugin::updateTopic1()
    {
        twist_sub_1_.shutdown();
        twist_sub_1_ = nh_.subscribe(twist_topic1_->getStdString(), 1, &CBFRVizPlugin::twistCallback1, this);
    }

    void CBFRVizPlugin::updateTopic2()
    {
        twist_sub_2_.shutdown();
        twist_sub_2_ = nh_.subscribe(twist_topic2_->getStdString(), 1, &CBFRVizPlugin::twistCallback2, this);
    }

    void CBFRVizPlugin::twistCallback1(const geometry_msgs::Twist::ConstPtr &msg)
    {
        ROS_INFO("Twist 1: Linear x: %f, Angular z: %f", msg->linear.x, msg->angular.z);
    }

    void CBFRVizPlugin::twistCallback2(const geometry_msgs::Twist::ConstPtr &msg)
    {
        ROS_INFO("Twist 2: Linear x: %f, Angular z: %f", msg->linear.x, msg->angular.z);
    }

}
