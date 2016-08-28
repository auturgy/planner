#!/usr/bin/expect -f

# launch mavproxy
spawn /home/ubuntu/dist/bin/arducopter_launch_no_init.sh

# wait patterns forever
set timeout -1

# wait for mavproxy to connect and exchange commands...
expect "GPS lock at 0 meters"

# we're ready to execute our additional initialization commands:
send "script /home/ubuntu/dist/conf/mavproxy.conf\r"

# make shell interactive again. never returns.
interact
