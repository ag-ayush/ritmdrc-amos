FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/base_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/base_msgs/motorOdom.h"
  "../msg_gen/cpp/include/base_msgs/powerInfo.h"
  "../msg_gen/cpp/include/base_msgs/setPid.h"
  "../msg_gen/cpp/include/base_msgs/setSpeed.h"
  "../msg_gen/cpp/include/base_msgs/motorRst.h"
  "../msg_gen/cpp/include/base_msgs/setAccel.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
