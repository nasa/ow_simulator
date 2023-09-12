
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ow_sim_tests'],
    package_dir={'': 'src'}
)

setup(**d)
