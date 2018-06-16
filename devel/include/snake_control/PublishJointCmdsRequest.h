// Generated by gencpp from file snake_control/PublishJointCmdsRequest.msg
// DO NOT EDIT!


#ifndef SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDSREQUEST_H
#define SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace snake_control
{
template <class ContainerAllocator>
struct PublishJointCmdsRequest_
{
  typedef PublishJointCmdsRequest_<ContainerAllocator> Type;

  PublishJointCmdsRequest_()
    : rate(0)
    , T()
    , params()
    , reset(false)  {
    }
  PublishJointCmdsRequest_(const ContainerAllocator& _alloc)
    : rate(0)
    , T()
    , params(_alloc)
    , reset(false)  {
  (void)_alloc;
    }



   typedef uint32_t _rate_type;
  _rate_type rate;

   typedef ros::Duration _T_type;
  _T_type T;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _params_type;
  _params_type params;

   typedef uint8_t _reset_type;
  _reset_type reset;





  typedef boost::shared_ptr< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PublishJointCmdsRequest_

typedef ::snake_control::PublishJointCmdsRequest_<std::allocator<void> > PublishJointCmdsRequest;

typedef boost::shared_ptr< ::snake_control::PublishJointCmdsRequest > PublishJointCmdsRequestPtr;
typedef boost::shared_ptr< ::snake_control::PublishJointCmdsRequest const> PublishJointCmdsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace snake_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'snake_control': ['/home/paul/snake_ws/src/snake_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8270d4695d0e9b9e853f2034dab5ddb";
  }

  static const char* value(const ::snake_control::PublishJointCmdsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8270d4695d0e9b9ULL;
  static const uint64_t static_value2 = 0xe853f2034dab5ddbULL;
};

template<class ContainerAllocator>
struct DataType< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "snake_control/PublishJointCmdsRequest";
  }

  static const char* value(const ::snake_control::PublishJointCmdsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 rate\n\
duration T\n\
float32[] params\n\
bool reset\n\
";
  }

  static const char* value(const ::snake_control::PublishJointCmdsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rate);
      stream.next(m.T);
      stream.next(m.params);
      stream.next(m.reset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PublishJointCmdsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::snake_control::PublishJointCmdsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::snake_control::PublishJointCmdsRequest_<ContainerAllocator>& v)
  {
    s << indent << "rate: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.rate);
    s << indent << "T: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.T);
    s << indent << "params[]" << std::endl;
    for (size_t i = 0; i < v.params.size(); ++i)
    {
      s << indent << "  params[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.params[i]);
    }
    s << indent << "reset: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDSREQUEST_H
