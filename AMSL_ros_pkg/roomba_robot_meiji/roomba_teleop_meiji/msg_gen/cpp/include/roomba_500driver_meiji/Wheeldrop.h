/* Auto-generated by genmsg_cpp for file /home/amsl/AMSL_ros_pkg/roomba_robot_meiji/roomba_500driver_meiji/msg/Wheeldrop.msg */
#ifndef ROOMBA_500DRIVER_MEIJI_MESSAGE_WHEELDROP_H
#define ROOMBA_500DRIVER_MEIJI_MESSAGE_WHEELDROP_H
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
struct Wheeldrop_ {
  typedef Wheeldrop_<ContainerAllocator> Type;

  Wheeldrop_()
  : left(false)
  , right(false)
  , caster(false)
  {
  }

  Wheeldrop_(const ContainerAllocator& _alloc)
  : left(false)
  , right(false)
  , caster(false)
  {
  }

  typedef uint8_t _left_type;
  uint8_t left;

  typedef uint8_t _right_type;
  uint8_t right;

  typedef uint8_t _caster_type;
  uint8_t caster;


  typedef boost::shared_ptr< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Wheeldrop
typedef  ::roomba_500driver_meiji::Wheeldrop_<std::allocator<void> > Wheeldrop;

typedef boost::shared_ptr< ::roomba_500driver_meiji::Wheeldrop> WheeldropPtr;
typedef boost::shared_ptr< ::roomba_500driver_meiji::Wheeldrop const> WheeldropConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace roomba_500driver_meiji

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6cc054efdd2e28563be3cbd30cbc4f26";
  }

  static const char* value(const  ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6cc054efdd2e2856ULL;
  static const uint64_t static_value2 = 0x3be3cbd30cbc4f26ULL;
};

template<class ContainerAllocator>
struct DataType< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roomba_500driver_meiji/Wheeldrop";
  }

  static const char* value(const  ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool left\n\
bool right\n\
bool caster\n\
\n\
\n\
";
  }

  static const char* value(const  ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.left);
    stream.next(m.right);
    stream.next(m.caster);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Wheeldrop_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::roomba_500driver_meiji::Wheeldrop_<ContainerAllocator> & v) 
  {
    s << indent << "left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right);
    s << indent << "caster: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.caster);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROOMBA_500DRIVER_MEIJI_MESSAGE_WHEELDROP_H

