// Generated by gencpp from file hins_le_driver/hins_srvRequest.msg
// DO NOT EDIT!


#ifndef HINS_LE_DRIVER_MESSAGE_HINS_SRVREQUEST_H
#define HINS_LE_DRIVER_MESSAGE_HINS_SRVREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hins_le_driver
{
template <class ContainerAllocator>
struct hins_srvRequest_
{
  typedef hins_srvRequest_<ContainerAllocator> Type;

  hins_srvRequest_()
    : channel(0)  {
    }
  hins_srvRequest_(const ContainerAllocator& _alloc)
    : channel(0)  {
  (void)_alloc;
    }



   typedef int64_t _channel_type;
  _channel_type channel;





  typedef boost::shared_ptr< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct hins_srvRequest_

typedef ::hins_le_driver::hins_srvRequest_<std::allocator<void> > hins_srvRequest;

typedef boost::shared_ptr< ::hins_le_driver::hins_srvRequest > hins_srvRequestPtr;
typedef boost::shared_ptr< ::hins_le_driver::hins_srvRequest const> hins_srvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hins_le_driver::hins_srvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hins_le_driver::hins_srvRequest_<ContainerAllocator1> & lhs, const ::hins_le_driver::hins_srvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.channel == rhs.channel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hins_le_driver::hins_srvRequest_<ContainerAllocator1> & lhs, const ::hins_le_driver::hins_srvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hins_le_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8850208bd0bd330676886afebd02f977";
  }

  static const char* value(const ::hins_le_driver::hins_srvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8850208bd0bd3306ULL;
  static const uint64_t static_value2 = 0x76886afebd02f977ULL;
};

template<class ContainerAllocator>
struct DataType< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hins_le_driver/hins_srvRequest";
  }

  static const char* value(const ::hins_le_driver::hins_srvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 channel\n"
;
  }

  static const char* value(const ::hins_le_driver::hins_srvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.channel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct hins_srvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hins_le_driver::hins_srvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hins_le_driver::hins_srvRequest_<ContainerAllocator>& v)
  {
    s << indent << "channel: ";
    Printer<int64_t>::stream(s, indent + "  ", v.channel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HINS_LE_DRIVER_MESSAGE_HINS_SRVREQUEST_H
