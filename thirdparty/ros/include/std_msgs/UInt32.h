// Generated by gencpp from file std_msgs/UInt32.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_UINT32_H
#define STD_MSGS_MESSAGE_UINT32_H


#include <string>
#include <vector>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs
{
template <class ContainerAllocator>
struct UInt32_
{
  typedef UInt32_<ContainerAllocator> Type;

  UInt32_()
    : data(0)  {
    }
  UInt32_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef uint32_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::std_msgs::UInt32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::std_msgs::UInt32_<ContainerAllocator> const> ConstPtr;

}; // struct UInt32_

typedef ::std_msgs::UInt32_<std::allocator<void> > UInt32;

typedef boost::shared_ptr< ::std_msgs::UInt32 > UInt32Ptr;
typedef boost::shared_ptr< ::std_msgs::UInt32 const> UInt32ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::UInt32_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::UInt32_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_msgs::UInt32_<ContainerAllocator1> & lhs, const ::std_msgs::UInt32_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_msgs::UInt32_<ContainerAllocator1> & lhs, const ::std_msgs::UInt32_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt32_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt32_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt32_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt32_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt32_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt32_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::UInt32_<ContainerAllocator> >
{
  static const char* value()
  {
    return "304a39449588c7f8ce2df6e8001c5fce";
  }

  static const char* value(const ::std_msgs::UInt32_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x304a39449588c7f8ULL;
  static const uint64_t static_value2 = 0xce2df6e8001c5fceULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::UInt32_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/UInt32";
  }

  static const char* value(const ::std_msgs::UInt32_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::UInt32_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 data\n"
;
  }

  static const char* value(const ::std_msgs::UInt32_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::UInt32_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UInt32_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::UInt32_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::UInt32_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_UINT32_H
