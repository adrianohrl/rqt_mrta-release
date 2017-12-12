# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rqt_mrta'],
    package_dir={'': 'src'},
    requires=['roscpp'],
	scripts=['scripts/rqt_mrta'],
)

setup(**setup_args)
