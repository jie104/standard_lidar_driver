// Generated by gencpp from file ltme_node/QueryHardwareVersionRequest.msg
// DO NOT EDIT!


#ifndef LTME_NODE_MESSAGE_QUERYHARDWAREVERSIONREQUEST_H
#define LTME_NODE_MESSAGE_QUERYHARDWAREVERSIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ltme_node
{
template <class ContainerAllocator>
struct QueryHardwareVersionRequest_
{
  typedef QueryHardwareVersionRequest_<ContainerAllocator> Type;

  QueryHardwareVersionRequest_()
    {
    }
  QueryHardwareVersionRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct QueryHardwareVersionRequest_

typedef ::ltme_node::QueryHardwareVersionRequest_<std::allocator<void> > QueryHardwareVersionRequest;

typedef boost::shared_ptr< ::ltme_node::QueryHardwareVersionRequest > QueryHardwareVersionRequestPtr;
typedef boost::shared_ptr< ::ltme_node::QueryHardwareVersionRequest const> QueryHardwareVersionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace ltme_node

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ltme_node/QueryHardwareVersionRequest";
  }

  static const char* value(const ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QueryHardwareVersionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::ltme_node::QueryHardwareVersionRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LTME_NODE_MESSAGE_QUERYHARDWAREVERSIONREQUEST_H
