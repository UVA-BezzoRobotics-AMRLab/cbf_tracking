#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/view_controller.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/ogre_helpers/viewport.h>
#include <rviz/ogre_helpers/ogre_logging.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreTexture.h>

#include <QVBoxLayout>
#include <QPushButton>

namespace cbf_filter_rviz_plugin {

class CBFRVizPlugin : public rviz::Display {
public:
  // Constructor
  YourRVizPlugin() {}

  // Overridden methods from Display
  virtual void onInitialize() override;

  virtual void update(float wall_dt, float ros_dt) override;

  virtual void fixedFrameChanged() override;

  virtual void reset() override;

protected:

  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_1_;
  ros::Subscriber twist_sub_2_;

  rviz::StringProperty *twist_topic1_;
  rviz::StringProperty *twist_topic2_;

  rviz::VisualizationManager* vis_manager_;

  void updateTopic1();
  void updateTopic2();

  void twistCallback1(const geometry_msgs::Twist::ConstPtr& msg);
  void twistCallback2(const geometry_msgs::Twist::ConstPtr& msg);

  
};

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CBFRVizPlugin::CBFRVizPlugin, rviz::Display)
