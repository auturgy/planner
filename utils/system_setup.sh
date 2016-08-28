#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

# copy installation files
CMD_CHECK "${UTILS_DIR}/deploy.sh $SRV_TAGNAME"

# setup locale
CMD_SSH_CHECK "sudo locale-gen 'en_US.UTF-8'"
CMD_SSH_CHECK "echo 'export LC_ALL=en_US.UTF-8' >>${SRV_HOMEDIR}/.bashrc"
CMD_SSH_CHECK "echo 'export LANG=en_US.UTF-8' >>${SRV_HOMEDIR}/.bashrc"

# make ssh fast
CMD_SSH_CHECK "sudo bash -c 'echo \"UseDNS no\" >>/etc/ssh/sshd_config'"
CMD_SSH_CHECK "sudo bash -c 'echo \"GatewayPorts clientspecified\" >>/etc/ssh/sshd_config'"

# remove login messages
CMD_SSH_CHECK "sudo rm -f /etc/update-motd.d/10-help-text"
CMD_SSH_CHECK "sudo bash -c 'echo >/etc/issue'"
CMD_SSH_CHECK "sudo bash -c 'echo >/etc/issue.net'"

# install packages

CMD_SSH_CHECK "sudo apt-get update"
CMD_SSH_CHECK "sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu trusty main\" > /etc/apt/sources.list.d/ros-latest.list'"
CMD_SSH_CHECK "wget -O ${SRV_HOMEDIR}/ros.key --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
CMD_SSH_CHECK "sudo apt-key add ${SRV_HOMEDIR}/ros.key && rm ${SRV_HOMEDIR}/ros.key"
CMD_SSH_CHECK "sudo apt-get update"
CMD_SSH_CHECK "sudo apt-get -fy install telnet g++ vim git unzip python-pip supervisor nginx daemontools \
python-passlib bash-completion screen device-tree-compiler python-serial  ros-indigo-mavros \
ros-indigo-ros-base ros-indigo-rosbridge-suite ros-indigo-roslint htop libusb-1.0-0-dev psmisc \
python-daemon uwsgi-plugin-python uwsgi uwsgi-plugin-python python-flask lvm2 cryptsetup e2fsprogs \
htop bmon python-virtualenv python3-pip curl hping3 apache2-utils realpath udev stunnel socat \
tcpdump usbmount sharutils ntp dnsutils gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav v4l-utils python-wstool librtlsdr-dev rtl-sdr \
libpng-dev libjpeg8-dev libfreetype6-dev expect"

# rosclient websocket
CMD_SSH_CHECK "sudo pip install trollius"
CMD_SSH_CHECK "sudo pip install twisted"
CMD_SSH_CHECK "sudo pip install LatLon"
CMD_SSH_CHECK "sudo pip install matplotlib"
CMD_SSH_CHECK "sudo pip install numpy"

CMD_SSH_CHECK "sudo apt-get clean"

#install python packages
CMD_SSH_CHECK "sudo pip install 'autobahn[asyncio]'"
CMD_SSH_CHECK "sudo pip3 install blinker"
CMD_SSH_CHECK "sudo pip3 install autobahn"

# disable apache2, enable nginx
#CMD_SSH_CHECK "sudo update-rc.d apache2 disable"
#CMD_SSH_CHECK "sudo /etc/init.d/apache2 stop"
CMD_SSH_CHECK "sudo usermod -a -G www-data ubuntu"

# setup ros
# if this is not the first init, remove this file otherwise rosdep init will fail.
CMD_SSH_CHECK sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
CMD_SSH_CHECK "sudo rosdep init"
CMD_SSH_CHECK "rosdep update"

# link nginx conf
CMD_SSH_CHECK "sudo rm -f /etc/nginx/sites-enabled/default"
CMD_SSH_CHECK "sudo rm -f /etc/nginx/sites-available/rosgcs /etc/nginx/sites-enabled/rosgcs"
CMD_SSH_CHECK "sudo ln -s /home/ubuntu/dist/conf/nginx.conf /etc/nginx/sites-available/rosgcs"
CMD_SSH_CHECK "sudo ln -s  /etc/nginx/sites-available/rosgcs /etc/nginx/sites-enabled/rosgcs"
CMD_SSH_CHECK "sudo rm -f /etc/supervisor/supervisord.conf"
CMD_SSH_CHECK "sudo ln -s /home/ubuntu/dist/conf/supervisord.conf /etc/supervisor/supervisord.conf"

# restart supervisor , this starts ROS etc.
CMD_SSH_CHECK "sudo /etc/init.d/supervisor restart"
CMD_SSH_CHECK "sudo /etc/init.d/nginx restart"




#####################################################################
#EOF
