FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/base_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/base_msgs/msg/__init__.py"
  "../src/base_msgs/msg/_motorOdom.py"
  "../src/base_msgs/msg/_powerInfo.py"
  "../src/base_msgs/msg/_setPid.py"
  "../src/base_msgs/msg/_setSpeed.py"
  "../src/base_msgs/msg/_motorRst.py"
  "../src/base_msgs/msg/_setAccel.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
