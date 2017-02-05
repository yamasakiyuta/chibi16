/* Auto-generated by genmsg_cpp for file /home/amsl/AMSL_ros_pkg/roomba_robot_meiji/roomba_500driver_meiji/msg/MotorOvercurrent.msg */
#ifndef ROOMBA_500DRIVER_MEIJI_MESSAGE_MOTOROVERCURRENT_H
#define ROOMBA_500DRIVER_MEIJI_MESSAGE_MOTOROVERCURRENT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace roomba_500driver_meiji
{
template <class ContainerAllocator>
struct MotorOvercurrent_ {
  typedef MotorOvercurrent_<ContainerAllocator> Type;

  MotorOvercurrent_()
  : side_brush(false)
  , vacuum(false)
  , main_brush(false)
  , drive_right(false)
  , drive_left(false)
  {
  }

  MotorOvercurrent_(const ContainerAllocator& _alloc)
  : side_brush(false)
  , vacuum(false)
  , main_brush(false)
  , drive_right(false)
  , drive_left(false)
  {
  }

  typedef uint8_t _side_brush_type;
  uint8_t side_brush;

  typedef uint8_t _vacuum_type;
  uint8_t vacuum;

  typedef uint8_t _main_brush_type;
  uint8_t main_brush;

  typedef uint8_t _drive_right_type;
  uint8_t drive_right;

  typedef uint8_t _drive_left_type;
  uint8_t drive_left;


  typedef boost::shared_ptr< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator>  const> ConstPtr;
}; // struct MotorOvercurrent
typedef  ::roomba_500driver_meiji::MotorOvercurrent_<std::allocator<void> > MotorOvercurrent;

typedef boost::shared_ptr< ::roomba_500driver_meiji::MotorOvercurrent> MotorOvercurrentPtr;
typedef boost::shared_ptr< ::roomba_500driver_meiji::MotorOvercurrent const> MotorOvercurrentConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace roomba_500driver_meiji

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6f9cad922d9c9777c65cca16d91d80bf";
  }

  static const char* value(const  ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6f9cad922d9c9777ULL;
  static const uint64_t static_value2 = 0xc65cca16d91d80bfULL;
};

template<class ContainerAllocator>
struct DataType< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roomba_500driver_meiji/MotorOvercurrent";
  }

  static const char* value(const  ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool side_brush\n\
bool vacuum\n\
bool main_brush\n\
bool drive_right\n\
bool drive_left\n\
\n\
";
  }

  static const char* value(const  ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.side_brush);
    stream.next(m.vacuum);
    stream.next(m.main_brush);
    stream.next(m.drive_right);
    stream.next(m.drive_left);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MotorOvercurrent_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::roomba_500driver_meiji::MotorOvercurrent_<ContainerAllocator> & v) 
  {
    s << indent << "side_brush: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.side_brush);
    s << indent << "vacuum: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vacuum);
    s << indent << "main_brush: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.main_brush);
    s << indent << "drive_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drive_right);
    s << indent << "drive_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drive_left);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROOMBA_500DRIVER_MEIJI_MESSAGE_MOTOROVERCURRENT_H

