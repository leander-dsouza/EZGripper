#!/usr/bin/env python

"""
Setup Program for EZGripper
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ezgripper_libs'],
    package_dir={'': 'src'},
    )

setup(**d)
