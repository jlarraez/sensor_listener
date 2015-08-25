// Generated by gencpp from file sensor_listener/PositionAPICoordSpaceQuatRequest.msg
// DO NOT EDIT!


#ifndef SENSOR_LISTENER_MESSAGE_POSITIONAPICOORDSPACEQUATREQUEST_H
#define SENSOR_LISTENER_MESSAGE_POSITIONAPICOORDSPACEQUATREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace sensor_listener
{
template <class ContainerAllocator>
struct PositionAPICoordSpaceQuatRequest_
{
  typedef PositionAPICoordSpaceQuatRequest_<ContainerAllocator> Type;

  PositionAPICoordSpaceQuatRequest_()
    : target()  {
    }
  PositionAPICoordSpaceQuatRequest_(const ContainerAllocator& _alloc)
    : target(_alloc)  {
    }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _target_type;
  _target_type target;




  typedef boost::shared_ptr< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PositionAPICoordSpaceQuatRequest_

typedef ::sensor_listener::PositionAPICoordSpaceQuatRequest_<std::allocator<void> > PositionAPICoordSpaceQuatRequest;

typedef boost::shared_ptr< ::sensor_listener::PositionAPICoordSpaceQuatRequest > PositionAPICoordSpaceQuatRequestPtr;
typedef boost::shared_ptr< ::sensor_listener::PositionAPICoordSpaceQuatRequest const> PositionAPICoordSpaceQuatRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensor_listener

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sensor_listener': ['/home/jorgearraez/catkin_ws/src/sensor_listener/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4b4e41b93b97f9307dacbb0795153e4";
  }

  static const char* value(const ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf4b4e41b93b97f93ULL;
  static const uint64_t static_value2 = 0x07dacbb0795153e4ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_listener/PositionAPICoordSpaceQuatRequest";
  }

  static const char* value(const ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose target\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct PositionAPICoordSpaceQuatRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_listener::PositionAPICoordSpaceQuatRequest_<ContainerAllocator>& v)
  {
    s << indent << "target: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.target);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_LISTENER_MESSAGE_POSITIONAPICOORDSPACEQUATREQUEST_H
