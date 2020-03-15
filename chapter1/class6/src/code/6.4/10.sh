#!/bin/bash
for i in `seq 1 254`
do
    {
        ping -c2 192.168.1.$i &>/dev/null && echo "192.168.1.$i is alive"
    }&
done
wait
