import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cameron/Project_Echo/Educational/amr_ws/install/echo_sim_v1'
