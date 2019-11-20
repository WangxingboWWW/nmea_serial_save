from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lib_nmea_serial', 'lib_nmea_serial.nodes'],
    package_dir={'': 'src'}
)

setup(**d)
