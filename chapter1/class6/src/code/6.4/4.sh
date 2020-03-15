#!/bin/bash

case "$1" in

'up')

	sudo ifconfig eth0 up;;

'down')

	sudo ifconfig eth0 down;;

*)

	echo "usage $0 up|down";;

esac



if [ $1 = "up" -o $1 = "down" ]

then

ifconfig eth0 | grep UP 1>/dev/null 2>/dev/null

if [ $? -eq 0 ]

then 

	echo "eth0 state is Up"

else

	echo "eth0 state is Down"

fi

fi
