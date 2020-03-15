#!/bin/bash

cat /etc/passwd | awk '{if(NR%2==1) {print NR,$0}else{print $0}}'
