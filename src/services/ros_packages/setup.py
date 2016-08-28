#!/usr/bin/env python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
# http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

# DO NOT USE
# python setup.py install

#####################################################################

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['skysense_sample'],
    package_dir={'': 'src'},
)

setup(**setup_args)

#####################################################################
# EOF
