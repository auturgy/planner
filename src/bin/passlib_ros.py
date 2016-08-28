#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

import sys
import os
import subprocess
import string
import random
import errno

from passlib.apps import custom_app_context as pwd_context

#####################################################################

#####################################################################


def main(argv):

    pwd = raw_input()
    print(pwd_context.encrypt(pwd))

    return 0

#####################################################################

if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print "Ctrl-c pressed ..."
        sys.exit(1)

#####################################################################
