// Generated by gencpp from file hexapodservice/hexapodserviceAction.msg
// DO NOT EDIT!


#ifndef HEXAPODSERVICE_MESSAGE_HEXAPODSERVICEACTION_H
#define HEXAPODSERVICE_MESSAGE_HEXAPODSERVICEACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hexapodservice/hexapodserviceActionGoal.h>
#include <hexapodservice/hexapodserviceActionResult.h>
#include <hexapodservice/hexapodserviceActionFeedback.h>

namespace hexapodservice
{
template <class ContainerAllocator>
struct hexapodserviceAction_
{
  typedef hexapodserviceAction_<ContainerAllocator> Type;

  hexapodserviceAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  hexapodserviceAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::hexapodservice::hexapodserviceActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::hexapodservice::hexapodserviceActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::hexapodservice::hexapodserviceActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;




  typedef boost::shared_ptr< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> const> ConstPtr;

}; // struct hexapodserviceAction_

typedef ::hexapodservice::hexapodserviceAction_<std::allocator<void> > hexapodserviceAction;

typedef boost::shared_ptr< ::hexapodservice::hexapodserviceAction > hexapodserviceActionPtr;
typedef boost::shared_ptr< ::hexapodservice::hexapodserviceAction const> hexapodserviceActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hexapodservice::hexapodserviceAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hexapodservice

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'hexapodservice': ['/home/quan/hexapod_service_ws/src/hexapodservice/msg', '/home/quan/hexapod_service_ws/devel/share/hexapodservice/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "253731c42181dfd32504f6a64aacfded";
  }

  static const char* value(const ::hexapodservice::hexapodserviceAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x253731c42181dfd3ULL;
  static const uint64_t static_value2 = 0x2504f6a64aacfdedULL;
};

template<class ContainerAllocator>
struct DataType< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hexapodservice/hexapodserviceAction";
  }

  static const char* value(const ::hexapodservice::hexapodserviceAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
hexapodserviceActionGoal action_goal\n\
hexapodserviceActionResult action_result\n\
hexapodserviceActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
hexapodserviceGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
int32 MODE \n\
int32 SIMPLEMOTION_MODE\n\
int32 MAXPOINTS\n\
int32 LEG_INDEX\n\
hexapodservice/legjoints ONELEG\n\
hexapodservice/legs ALLLEGS\n\
\n\
================================================================================\n\
MSG: hexapodservice/legjoints\n\
int64[] coxa\n\
int64[] femur\n\
int64[] tibia\n\
int64[] tarsus\n\
\n\
================================================================================\n\
MSG: hexapodservice/legs\n\
hexapodservice/legjoints[6] leg\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
hexapodserviceResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
int32 status\n\
int32 freespace\n\
bool motionActive\n\
string result\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
hexapodserviceFeedback feedback\n\
\n\
================================================================================\n\
MSG: hexapodservice/hexapodserviceFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
hexapodservice/leg ALLLEGS_fdbk\n\
\n\
\n\
================================================================================\n\
MSG: hexapodservice/leg\n\
hexapodservice/legjoint[6] leg\n\
\n\
================================================================================\n\
MSG: hexapodservice/legjoint\n\
float64 coxa\n\
float64 femur\n\
float64 tibia\n\
float64 tarsus\n\
";
  }

  static const char* value(const ::hexapodservice::hexapodserviceAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct hexapodserviceAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hexapodservice::hexapodserviceAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hexapodservice::hexapodserviceAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::hexapodservice::hexapodserviceActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::hexapodservice::hexapodserviceActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::hexapodservice::hexapodserviceActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEXAPODSERVICE_MESSAGE_HEXAPODSERVICEACTION_H
