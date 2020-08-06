// Generated by gencpp from file ackermann_msgs/AckermannDriveStamped.msg
// DO NOT EDIT!


#ifndef ACKERMANN_MSGS_MESSAGE_ACKERMANNDRIVESTAMPED_H
#define ACKERMANN_MSGS_MESSAGE_ACKERMANNDRIVESTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace ackermann_msgs
{
template <class ContainerAllocator>
struct AckermannDriveStamped_
{
  typedef AckermannDriveStamped_<ContainerAllocator> Type;

  AckermannDriveStamped_()
    : header()
    , drive()  {
    }
  AckermannDriveStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drive(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::ackermann_msgs::AckermannDrive_<ContainerAllocator>  _drive_type;
  _drive_type drive;





  typedef boost::shared_ptr< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> const> ConstPtr;

}; // struct AckermannDriveStamped_

typedef ::ackermann_msgs::AckermannDriveStamped_<std::allocator<void> > AckermannDriveStamped;

typedef boost::shared_ptr< ::ackermann_msgs::AckermannDriveStamped > AckermannDriveStampedPtr;
typedef boost::shared_ptr< ::ackermann_msgs::AckermannDriveStamped const> AckermannDriveStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ackermann_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ackermann_msgs': ['/home/jun/smartcar_ws/src/system/ackermann_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1fd5d7f58889cefd44d29f6653240d0c";
  }

  static const char* value(const ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1fd5d7f58889cefdULL;
  static const uint64_t static_value2 = 0x44d29f6653240d0cULL;
};

template<class ContainerAllocator>
struct DataType< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ackermann_msgs/AckermannDriveStamped";
  }

  static const char* value(const ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## Time stamped drive command for robots with Ackermann steering.\n\
#  $Id$\n\
\n\
Header          header\n\
AckermannDrive  drive\n\
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
MSG: ackermann_msgs/AckermannDrive\n\
## Driving command for a car-like vehicle using Ackermann steering.\n\
#  $Id$\n\
\n\
# Assumes Ackermann front-wheel steering. The left and right front\n\
# wheels are generally at different angles. To simplify, the commanded\n\
# angle corresponds to the yaw of a virtual wheel located at the\n\
# center of the front axle, like on a tricycle.  Positive yaw is to\n\
# the left. (This is *not* the angle of the steering wheel inside the\n\
# passenger compartment.)\n\
#\n\
# Zero steering angle velocity means change the steering angle as\n\
# quickly as possible. Positive velocity indicates a desired absolute\n\
# rate of change either left or right. The controller tries not to\n\
# exceed this limit in either direction, but sometimes it might.\n\
#\n\
float32 steering_angle          # desired virtual angle (radians)\n\
float32 steering_angle_velocity # desired rate of change (radians/s)\n\
\n\
# Drive at requested speed, acceleration and jerk (the 1st, 2nd and\n\
# 3rd derivatives of position). All are measured at the vehicle's\n\
# center of rotation, typically the center of the rear axle. The\n\
# controller tries not to exceed these limits in either direction, but\n\
# sometimes it might.\n\
#\n\
# Speed is the desired scalar magnitude of the velocity vector.\n\
# Direction is forward unless the sign is negative, indicating reverse.\n\
#\n\
# Zero acceleration means change speed as quickly as\n\
# possible. Positive acceleration indicates a desired absolute\n\
# magnitude; that includes deceleration.\n\
#\n\
# Zero jerk means change acceleration as quickly as possible. Positive\n\
# jerk indicates a desired absolute rate of acceleration change in\n\
# either direction (increasing or decreasing).\n\
#\n\
float32 speed                   # desired forward speed (m/s)\n\
float32 acceleration            # desired acceleration (m/s^2)\n\
float32 jerk                    # desired jerk (m/s^3)\n\
";
  }

  static const char* value(const ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drive);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AckermannDriveStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ackermann_msgs::AckermannDriveStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drive: ";
    s << std::endl;
    Printer< ::ackermann_msgs::AckermannDrive_<ContainerAllocator> >::stream(s, indent + "  ", v.drive);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACKERMANN_MSGS_MESSAGE_ACKERMANNDRIVESTAMPED_H
