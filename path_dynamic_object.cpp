/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>

#include "geometry_msgs/PoseArray.h"

#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcPredictionFullState.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/transformer.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace path_object
{

/* class PathDynamicObject //{ */

class PathDynamicObject : public nodelet::Nodelet {

public:
  virtual void onInit();
   bool      objects_visited_ = true;
   volatile bool teste_path = true;

private:
  bool is_initialized_ = false;

  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  
  double randd(const double from, const double to);
  int    randi(const int from, const int to);
  bool   setPathSrv(const mrs_msgs::Path path_in);

  bool checkReference(const std::string frame, const double x, const double y, const double z, const double hdg);

  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>           sh_position_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>    sh_mpc_predition_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>            sh_next_point_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>            sh_next_object_;

  std::optional<mrs_msgs::PositionCommand> transformPositionCmd(const mrs_msgs::PositionCommand& position_cmd, const std::string& target_frame);

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  ros::Publisher publisher_goto_;
  ros::Publisher real_pos_pub;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_path_;
  ros::ServiceClient service_client_check_reference_;

  ros::Timer timer_main_;

  std::string _frame_id_;
  std::string _uav_name_;

  double _main_timer_rate_;

  bool _relax_heading_;
  bool _use_heading_;

  int _n_points_min_;
  int _n_points_max_;

  double _point_distance_min_;
  double _point_distance_max_;

  double _z_value_;
  double _z_deviation_;

  double _future_stamp_prob_;
  double _future_stamp_min_;
  double _future_stamp_max_;

  double _replanning_time_min_;
  double _replanning_time_max_;

  double _heading_change_;
  double _bearing_change_;
  double _initial_bearing_change_;

  bool   _override_constraints_;
  double _override_speed_;
  double _override_acceleration_;

  bool active_ = true;
  int count_global;
  int tam;
  int num_obj;
  int count_obj;

  int path_id_ = 0, age =0;

  bool      next_wait_for_finish_ = false;
  bool      coverage_finish_ = false;

  
  ros::Time next_replan_time_;

  ros::Time last_successfull_command_;

  double bearing_ = 0;
};

//}

/* onInit() //{ */

void PathDynamicObject::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "PathDynamicObject");

  // load parameters from config file
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("active", active_);

  param_loader.loadParam("frame_id", _frame_id_);
  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("relax_heading", _relax_heading_);
  param_loader.loadParam("use_heading", _use_heading_);

  param_loader.loadParam("heading_change", _heading_change_);
  param_loader.loadParam("bearing_change", _bearing_change_);
  param_loader.loadParam("initial_bearing_change", _initial_bearing_change_);
  param_loader.loadParam("n_points/min", _n_points_min_);
  param_loader.loadParam("n_points/max", _n_points_max_);
  param_loader.loadParam("point_distance/min", _point_distance_min_);
  param_loader.loadParam("point_distance/max", _point_distance_max_);
  param_loader.loadParam("z/value", _z_value_);
  param_loader.loadParam("z/deviation", _z_deviation_);

  param_loader.loadParam("future_stamp/prob", _future_stamp_prob_);
  param_loader.loadParam("future_stamp/min", _future_stamp_min_);
  param_loader.loadParam("future_stamp/max", _future_stamp_max_);

  param_loader.loadParam("replanning_time/min", _replanning_time_min_);
  param_loader.loadParam("replanning_time/max", _replanning_time_max_);

  param_loader.loadParam("override_constraints/enabled", _override_constraints_);
  param_loader.loadParam("override_constraints/speed", _override_speed_);
  param_loader.loadParam("override_constraints/acceleration", _override_acceleration_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PathDynamicObject]: Could not load all parameters!");
    ros::shutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "PathDynamicObject";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;

  sh_position_cmd_         = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_command_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");
  sh_mpc_predition_        = mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>(shopts, "mpc_prediction_in");
  sh_next_point_           = mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>(shopts, "/coverage_node_1");
  sh_next_object_	    = mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>(shopts, "//objects_point_1");

  service_server_activate_        = nh_.advertiseService("activate_in", &PathDynamicObject::callbackActivate, this);
  service_client_path_            = nh_.serviceClient<mrs_msgs::PathSrv>("path_out");
  service_client_check_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("check_reference_out");

  real_pos_pub = nh_.advertise<geometry_msgs::Pose>("/real_pos", 1000);

  // initialize the random number generator
  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  /* srand(time(NULL)); */

  last_successfull_command_ = ros::Time(0);

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &PathDynamicObject::timerMain, this);
  

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "PathDynamicObject");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[PathDynamicObject]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActivate() //{ */

bool PathDynamicObject::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  active_ = true;

  res.success = true;
  res.message = "activated";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void PathDynamicObject::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_ONCE("[PathDynamicObject]: waiting for initialization");
    return;
  }

  if (!sh_position_cmd_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for PositionCommand");
    return;
  }
  // !Verify the utility
  
  if (!sh_mpc_predition_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for MPC prediction");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for ControlManager diagnostics");
    return;
  }

  bool has_goal = sh_control_manager_diag_.getMsg()->tracker_status.have_goal;

  auto position_cmd_transformed = transformPositionCmd(*sh_position_cmd_.getMsg(), _uav_name_ + "/" + _frame_id_);

  if (!position_cmd_transformed) {
    std::stringstream ss;
    ss << "could not transform position_cmd to the path frame";
    ROS_ERROR_STREAM("[PathDynamicObject]: " << ss.str());
    return;
  }
  
  auto [cmd_x, cmd_y, cmd_z]                   = mrs_lib::getPosition(position_cmd_transformed.value());
 

  auto [cmd_speed_x, cmd_speed_y, cmd_speed_z] = mrs_lib::getVelocity(position_cmd_transformed.value());

  geometry_msgs::Pose msg;
  msg.position.x = cmd_x;
  msg.position.y = cmd_y;
  msg.position.z = cmd_z;

  real_pos_pub.publish(msg);

  ROS_INFO("[PathDynamicObject]: cmd_x: %.2f, cmd_y: %.2f, cmd_z: %.2f", cmd_x, cmd_y, cmd_z);

  
  // if the uav reached the previousy set destination
  // if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 &&
  //     (!has_goal || (!next_wait_for_finish_ && (next_replan_time_ - ros::Time::now()).toSec() < 0))) {
  if(teste_path == true){
    teste_path = false;
    // create new point to fly to
    mrs_msgs::Path path;
    path.fly_now       = true;
    path.use_heading   = _use_heading_;
    path.relax_heading = _relax_heading_;

    double pos_x, pos_y, pos_z;

    // if (!next_wait_for_finish_) {

    //   double time_offset = randd(_future_stamp_min_, _future_stamp_max_);

    //   int prediction_idx = int(round((time_offset - 0.01) / 0.2));

    //   mrs_msgs::ReferenceStamped new_point;
      
    //   new_point.header               = sh_mpc_predition_.getMsg()->header;
    //   new_point.reference.position.x = sh_mpc_predition_.getMsg()->position[prediction_idx].x;
    //   new_point.reference.position.y = sh_mpc_predition_.getMsg()->position[prediction_idx].y;
    //   new_point.reference.position.z = sh_mpc_predition_.getMsg()->position[prediction_idx].z;
      
      
    //   new_point.reference.heading    = sh_mpc_predition_.getMsg()->heading[prediction_idx];

    //   auto res = transformer_->transformSingle(new_point, _frame_id_);

    //   if (res) {
    //     new_point = res.value();
    //   } else {
    //     std::stringstream ss;
    //     ss << "could not transform initial condition to the desired frame";
    //     ROS_ERROR_STREAM("[PathDynamicObject]: " << ss.str());
    //     return;
    //   }

    //   if (has_goal) {
    //     path.header.stamp = ros::Time::now() + ros::Duration(time_offset);
    //   } else {
    //     path.header.stamp = ros::Time(0);
    //   }

    //   pos_x    = new_point.reference.position.x;
    //   pos_y    = new_point.reference.position.y;
    //   pos_z    = new_point.reference.position.z;
    //   bearing_ = new_point.reference.heading;
    
    //   path.points.push_back(new_point.reference);

    // } else {

      pos_x = cmd_x;
      pos_y = cmd_y;
      pos_z = cmd_z;

      path.header.stamp = ros::Time(0);
    //}

    path.header.frame_id = _uav_name_ + "/" + _frame_id_;

    double dist;

    bearing_ += randd(-_initial_bearing_change_, _initial_bearing_change_);

    double heading = randd(-M_PI, M_PI);

    ROS_INFO("[PathDynamicObject]: pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);
    // int n_points = randi(_n_points_min_, _n_points_max_);

   //if(!coverage_finish_){
   
   	tam = sh_next_point_.getMsg()->poses[0].orientation.x;
   	//int count_global = sh_next_point_.getMsg()->poses[0].orientation.y;
    int count_global = 0;
    	double axisX[] = {};
    	double axisY[] = {};
    	double axisZ[] = {};

   	 for (int it = count_global; it < tam; it++) {
    	  axisX[it] = sh_next_point_.getMsg()->poses[it].position.x;
    	  axisY[it] = sh_next_point_.getMsg()->poses[it].position.y;
   	  axisZ[it] = sh_next_point_.getMsg()->poses[it].position.z;
         // double heading_change = randd(-_heading_change_, _heading_change_);

      	 ROS_INFO("[PathDynamicObject]: check pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);
      
   
        // if (!checkReference(_uav_name_ + "/" + _frame_id_, pos_x, pos_y, pos_z, bearing_)) {
        //   break;
        // }
      
        bearing_ += randd(-_bearing_change_, _bearing_change_);

        // heading += randd(-_heading_change_, _heading_change_);
      
        // double distance = randd(_point_distance_min_, _point_distance_max_);
        
    
        // pos_x += cos(bearing_) * distance;
        // pos_y += sin(bearing_) * distance;
        // pos_z = _z_value_ + randd(-_z_deviation_, _z_deviation_);

        pos_x = axisX[it];  
        pos_y = axisY[it];  
        pos_z = axisZ[it];
  
        //receive points of the trajectory
        mrs_msgs::Reference new_point;
        new_point.position.x = pos_x;
     	 new_point.position.y = pos_y;
     	 new_point.position.z = pos_z;
     	 new_point.heading    = bearing_;

     	 ROS_INFO("[PathDynamicObject]: pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

        //collect image
	
    	  path.points.push_back(new_point);
    	}
    //}
    // else {
    //  if(!objects_visited_){
    
          
    //       ROS_INFO("[PathDynamicObject]: Visit objects pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);
	  // tam = sh_next_point_.getMsg()->poses[0].orientation.x;
   	//   int count_global = sh_next_point_.getMsg()->poses[0].orientation.y;
    // 	  double axisX[] = {};
    // 	  double axisY[] = {};
    // 	  double axisZ[] = {};

   	//   for (int it = count_global; it < tam; it++) {
    // 	    axisX[it] = sh_next_point_.getMsg()->poses[it].position.x;
    // 	    axisY[it] = sh_next_point_.getMsg()->poses[it].position.y;
   	//     axisZ[it] = sh_next_point_.getMsg()->poses[it].position.z;
    //        // double heading_change = randd(-_heading_change_, _heading_change_);


         
       
    //   	   bearing_ += randd(-_bearing_change_, _bearing_change_);

    //        pos_x = axisX[it];  
    //        pos_y = axisY[it];  
    //        pos_z = axisZ[it];
   
    //       //receive points of the trajectory
    //       mrs_msgs::Reference new_point;
    //       new_point.position.x = pos_x;
    //  	  new_point.position.y = pos_y;
    //  	  new_point.position.z = pos_z;
    //  	  new_point.heading    = bearing_;

    //   	  ROS_INFO("[PathDynamicObject]: Object 1 pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

    // 	  path.points.push_back(new_point);

    //      }	
    //   }
    // }
    
    // next_wait_for_finish_ = randd(0, 10) <= 100* _future_stamp_prob_ ? false : true;

    // if (!next_wait_for_finish_) {
    //   double replan_time = randd(_replanning_time_min_, _replanning_time_max_);
    //   next_replan_time_  = ros::Time::now() + ros::Duration(replan_time);
    //   ROS_INFO("[PathDynamicObject]: replanning in %.2f s", replan_time);
    // } 

    // if (_override_constraints_) {

    //   path.override_constraints = true;

    //   path.override_max_velocity_horizontal = _override_speed_;
    //   path.override_max_velocity_vertical   = _override_speed_;

    //   path.override_max_acceleration_horizontal = _override_acceleration_;
    //   path.override_max_acceleration_vertical   = _override_acceleration_;

    //   ROS_INFO_THROTTLE(1.0, "[PathDynamicObject]: overriding constraints to speed: %.2f m/s, acc: %.2f m/s2", path.override_max_velocity_horizontal,
    //                     path.override_max_acceleration_horizontal);
    // }

    if (setPathSrv(path)) {

      ROS_INFO("[PathDynamicObject]: path set");

      last_successfull_command_ = ros::Time::now();
    }
  }
  ///////

  //  if(count_global == tam-1){
  //  		 coverage_finish_ = true;
  //  		 ROS_INFO("[PathDynamicObject]: Finished coverage!");
  //  		 objects_visited_ = true; 
  //  		 //ROS_INFO("[PathDynamicObject]: To visit objects detected !");

  //  }
}  // timerMain

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double PathDynamicObject::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int PathDynamicObject::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* setPathSrv() //{ */

bool PathDynamicObject::setPathSrv(const mrs_msgs::Path path_in) {

  mrs_msgs::PathSrv srv;
  srv.request.path = path_in;

  srv.request.path.input_id = path_id_++;

  bool success = service_client_path_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[PathDynamicObject]: service call for setting path failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[PathDynamicObject]: service call for setting path failed");
    return false;
  }
}

//}

/* checkReference() //{ */

bool PathDynamicObject::checkReference(const std::string frame, const double x, const double y, const double z, const double hdg) {

  mrs_msgs::ValidateReference srv;
  srv.request.reference.header.frame_id      = frame;
  srv.request.reference.reference.position.x = x;
  srv.request.reference.reference.position.y = y;
  srv.request.reference.reference.position.z = z;
  srv.request.reference.reference.heading    = hdg;

  bool success = service_client_check_reference_.call(srv);

  if (success) {

    return srv.response.success;

  } else {
    ROS_ERROR_THROTTLE(1.0, "[PathDynamicObject]: service call for setting path failed");
    return false;
  }
}

//}

/* transformPositionCmd() //{ */

std::optional<mrs_msgs::PositionCommand> PathDynamicObject::transformPositionCmd(const mrs_msgs::PositionCommand& position_cmd, const std::string& target_frame) {


  // if we transform to the current control frame, which is in fact the same frame as the position_cmd is in
  if (target_frame == "") {
    return position_cmd; 
  }

  // find the transformation
  auto tf = transformer_->getTransform(position_cmd.header.frame_id, target_frame, position_cmd.header.stamp);

  if (!tf) {
    ROS_ERROR("[PathDynamicObject]: could not find transform from '%s' to '%s' in time %f", position_cmd.header.frame_id.c_str(), target_frame.c_str(),
              position_cmd.header.stamp.toSec());
    return {};
  }

  mrs_msgs::PositionCommand cmd_out;

  cmd_out.header.stamp    = tf.value().header.stamp;
  cmd_out.header.frame_id = transformer_->frame_to(tf.value());

  /* position + heading //{ */

  {
    geometry_msgs::PoseStamped pos;
    pos.header = position_cmd.header;

    pos.pose.position    = position_cmd.position;
    pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, position_cmd.heading);

    if (auto ret = transformer_->transform(pos, tf.value())) {
      cmd_out.position = ret.value().pose.position;
      try {
        cmd_out.heading = mrs_lib::AttitudeConverter(ret.value().pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[PathDynamicObject]: failed to transform heading in position_cmd");
        cmd_out.heading = 0;
      }
    } else {
      return {};
    }
  }

  //}

  /* velocity //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.velocity;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.velocity = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* acceleration //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.acceleration;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.acceleration = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* jerk //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.jerk;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.jerk = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* heading derivatives //{ */

  // this does not need to be transformed
  cmd_out.heading_rate         = position_cmd.heading_rate;
  cmd_out.heading_acceleration = position_cmd.heading_acceleration;
  cmd_out.heading_jerk         = position_cmd.heading_jerk;

  //}

  return cmd_out;
}

//}

}  // namespace path_object
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_object::PathDynamicObject, nodelet::Nodelet)


