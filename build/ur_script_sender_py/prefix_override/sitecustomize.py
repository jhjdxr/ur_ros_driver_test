import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dxr/ws_ur_scripts/install/ur_script_sender_py'
