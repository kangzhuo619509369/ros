#!/bin/bash

str=`find ./ -name "*.txt"`

for i in $str

do	
	echo $i

	mv "$i" "${i%txt}h"

done
