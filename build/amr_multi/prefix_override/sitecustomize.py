import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/djqsp2/rokey3_E4_ws/install/amr_multi'
