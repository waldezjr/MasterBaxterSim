## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['master_baxter_sim'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'numpy', 'baxter_pykdl', 'baxter_interface',
        'baxter_external_devices', 'time', 'sys', 'std_msgs', 'baxter_core_msgs',
        'sensor_msgs', 'math', 'tf']
)

setup(**setup_args)
