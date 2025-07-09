import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/install/plotly_ros'
