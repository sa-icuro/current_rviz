#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/forwards.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

#include "build_route_tool.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <QVariant>

Ogre::Vector3 route_data;
std_msgs::String route_msg;

namespace ubiquity_tools
{
  BuildRouteTool::BuildRouteTool()
  {
    updateTopic();
  }

  BuildRouteTool::~BuildRouteTool()
  {
  }

  void BuildRouteTool::onInitialize()
    {
      setName( "Build Route" );
    }

  void BuildRouteTool::updateTopic()
  {
    pub_route = mnh_route.advertise<std_msgs::String>( "route_data", 1 );
  }

  int BuildRouteTool::processMouseEvent( rviz::ViewportMouseEvent& event )
  {
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode
    if( event.alt() )
    {
      selecting_ = false;
    }
    else
    {
      if( event.leftDown() )
      {
        selecting_ = true;
      }
    }

    if( selecting_ )
    {
      if( event.leftUp() )
      {
        //Determine Position_X and Position_Y of selected marker
        rviz::SelectionManager* sel_manager = context_->getSelectionManager();
        rviz::M_Picked selection = sel_manager->getSelection();
        rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
        int num_points = model->rowCount();
        if( selection.empty() || num_points <= 0 )
        {
          return flags;
        }

        for( int i = 0; i < num_points; i++ )
        {
          QModelIndex child_index = model->index( i, 0 );
          rviz::Property* child = model->getProp( child_index );
          rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
          Ogre::Vector3 vec = subchild->getVector();
          route_data = vec;
          std::stringstream stream;
          //Create message using X and Y position data, Quat-Z will be found when a Node_ID is matched by USER
          stream << vec.x << " " << vec.y << " " << vec.z;
          route_msg.data = stream.str();
        }
      }
    }
    return flags;
  }

  int BuildRouteTool::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
  {
    int render = rviz::SelectionTool::processKeyEvent(event, panel);
    if( event->key() == Qt::Key_A )
    {
      pub_route.publish(route_msg);
    }
  }

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ubiquity_tools::BuildRouteTool, rviz::Tool )
