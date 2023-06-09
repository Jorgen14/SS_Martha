// Generated by gencpp from file zed_interfaces/Skeleton3D.msg
// DO NOT EDIT!


#ifndef ZED_INTERFACES_MESSAGE_SKELETON3D_H
#define ZED_INTERFACES_MESSAGE_SKELETON3D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <zed_interfaces/Keypoint3D.h>

namespace zed_interfaces
{
template <class ContainerAllocator>
struct Skeleton3D_
{
  typedef Skeleton3D_<ContainerAllocator> Type;

  Skeleton3D_()
    : keypoints()  {
    }
  Skeleton3D_(const ContainerAllocator& _alloc)
    : keypoints()  {
  (void)_alloc;
      keypoints.assign( ::zed_interfaces::Keypoint3D_<ContainerAllocator> (_alloc));
  }



   typedef boost::array< ::zed_interfaces::Keypoint3D_<ContainerAllocator> , 18>  _keypoints_type;
  _keypoints_type keypoints;





  typedef boost::shared_ptr< ::zed_interfaces::Skeleton3D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zed_interfaces::Skeleton3D_<ContainerAllocator> const> ConstPtr;

}; // struct Skeleton3D_

typedef ::zed_interfaces::Skeleton3D_<std::allocator<void> > Skeleton3D;

typedef boost::shared_ptr< ::zed_interfaces::Skeleton3D > Skeleton3DPtr;
typedef boost::shared_ptr< ::zed_interfaces::Skeleton3D const> Skeleton3DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::zed_interfaces::Skeleton3D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::zed_interfaces::Skeleton3D_<ContainerAllocator1> & lhs, const ::zed_interfaces::Skeleton3D_<ContainerAllocator2> & rhs)
{
  return lhs.keypoints == rhs.keypoints;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::zed_interfaces::Skeleton3D_<ContainerAllocator1> & lhs, const ::zed_interfaces::Skeleton3D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace zed_interfaces

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::Skeleton3D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::Skeleton3D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::Skeleton3D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b704d948cb88b776b9f51ee392e13c62";
  }

  static const char* value(const ::zed_interfaces::Skeleton3D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb704d948cb88b776ULL;
  static const uint64_t static_value2 = 0xb9f51ee392e13c62ULL;
};

template<class ContainerAllocator>
struct DataType< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "zed_interfaces/Skeleton3D";
  }

  static const char* value(const ::zed_interfaces::Skeleton3D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Skeleton joints indices\n"
"#        16-14   15-17\n"
"#             \\ /\n"
"#              0\n"
"#              |\n"
"#       2------1------5\n"
"#       |    |   |    |\n"
"#	    |    |   |    |\n"
"#       3    |   |    6\n"
"#       |    |   |    |\n"
"#       |    |   |    |\n"
"#       4    8   11   7\n"
"#            |   |\n"
"#            |   |\n"
"#            |   |\n"
"#            9   12\n"
"#            |   |\n"
"#            |   |\n"
"#            |   |\n"
"#           10   13\n"
"zed_interfaces/Keypoint3D[18] keypoints\n"
"\n"
"================================================================================\n"
"MSG: zed_interfaces/Keypoint3D\n"
"float32[3] kp\n"
;
  }

  static const char* value(const ::zed_interfaces::Skeleton3D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.keypoints);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Skeleton3D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zed_interfaces::Skeleton3D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::zed_interfaces::Skeleton3D_<ContainerAllocator>& v)
  {
    s << indent << "keypoints[]" << std::endl;
    for (size_t i = 0; i < v.keypoints.size(); ++i)
    {
      s << indent << "  keypoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::zed_interfaces::Keypoint3D_<ContainerAllocator> >::stream(s, indent + "    ", v.keypoints[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ZED_INTERFACES_MESSAGE_SKELETON3D_H
