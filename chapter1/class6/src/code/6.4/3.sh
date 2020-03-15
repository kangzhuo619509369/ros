#!/bin/bash

ifconfig wlan0 | grep "inet " | awk 'BEGIN {OFS="\n"} {print $2,$3,$4}' | awk -F: '{print $2}'
