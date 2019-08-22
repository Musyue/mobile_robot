## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
 
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
 
# fetch values from package.xml
setup_args = generate_distutils_setup(
     packages=['can_analysis','config','logger_config','mobile_control','mobile_homing','pid_control'],
     package_dir={'': 'src'},
     )
   
setup(**setup_args)