#!/bin/bash

ping -c1 -w1 $1 | grep "ttl=[0-9]\+" &>/dev/null

if [ $? -eq 0 ]

then

	echo "success"

else

	echo "failed"

fi
