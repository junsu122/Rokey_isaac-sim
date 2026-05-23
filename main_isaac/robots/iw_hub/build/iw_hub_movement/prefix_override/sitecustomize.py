import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim/main_isaac/robots/iw_hub/install/iw_hub_movement'
