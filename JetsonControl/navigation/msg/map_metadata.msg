# map_metadata.msg
from geometry_msgs.msg import PoseStamped

MapMetadata
  uint32 width
  uint32 height
  float64 resolution
  geometry_msgs/PoseStamped origin
  time map_load_time
  float64 map_width
  float64 map_height