// Generated by gencpp from file snake_control/GetJointTrajectoryResponse.msg
// DO NOT EDIT!


#ifndef SNAKE_CONTROL_MESSAGE_GETJOINTTRAJECTORYRESPONSE_H
#define SNAKE_CONTROL_MESSAGE_GETJOINTTRAJECTORYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace snake_control
{
template <class ContainerAllocator>
struct GetJointTrajectoryResponse_
{
  typedef GetJointTrajectoryResponse_<ContainerAllocator> Type;

  GetJointTrajectoryResponse_()
    : header()
    , joint_names()
    , points()  {
    }
  GetJointTrajectoryResponse_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , joint_names(_alloc)
    , points(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  _joint_names_type joint_names;

   typedef std::vector< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::other >  _points_type;
  _points_type points;





  typedef boost::shared_ptr< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetJointTrajectoryResponse_

typedef ::snake_control::GetJointTrajectoryResponse_<std::allocator<void> > GetJointTrajectoryResponse;

typedef boost::shared_ptr< ::snake_control::GetJointTrajectoryResponse > GetJointTrajectoryResponsePtr;
typedef boost::shared_ptr< ::snake_control::GetJointTrajectoryResponse const> GetJointTrajectoryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace snake_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'snake_control': ['/home/paul/snake_ws/src/snake_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "65b4f94a94d1ed67169da35a02f33d3f";
  }

  static const char* value(const ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x65b4f94a94d1ed67ULL;
  static const uint64_t static_value2 = 0x169da35a02f33d3fULL;
};

template<class ContainerAllocator>
struct DataType< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "snake_control/GetJointTrajectoryResponse";
  }

  static const char* value(const ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
string[] joint_names\n\
trajectory_msgs/JointTrajectoryPoint[] points\n\
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
MSG: trajectory_msgs/JointTrajectoryPoint\n\
# Each trajectory point specifies either positions[, velocities[, accelerations]]\n\
# or positions[, effort] for the trajectory to be executed.\n\
# All specified values are in the same order as the joint names in JointTrajectory.msg\n\
\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
float64[] effort\n\
duration time_from_start\n\
";
  }

  static const char* value(const ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.joint_names);
      stream.next(m.points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetJointTrajectoryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::snake_control::GetJointTrajectoryResponse_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SNAKE_CONTROL_MESSAGE_GETJOINTTRAJECTORYRESPONSE_H
