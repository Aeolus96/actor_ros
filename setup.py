## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

try:
    from setuptools import setup  # Python 3.10 onwards distutils is now inside setuptools
except ImportError:
    from distutils.core import setup  # Python < 3.10 uses this module
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["actor_ros"],
    package_dir={"": "src"},
    
    # Add install dependencies below:
    # for example: <sudo pip install required_python_package1 required_python_package2> becomes:
    # install_requires=['required_python_package1', 'required_python_package2'],
    # Also make sure to add these to package.xml using rosdistro list keys
    # https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
    # example: <run_depend>python-ws4py-pip</run_depend>
    # if not found in rosdistro list, install manually...
)

setup(**setup_args)
