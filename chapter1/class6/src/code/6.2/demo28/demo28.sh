#! /bin/bash

echo "enter filename to print from awk"
read filename

if [[ -f $filename ]]
then
	echo "enter the text to search"
	read grepVar
	echo $grepVar
	awk '/'$grepVar'/ {print}' $filename
	#awk '/linux/ {print}' $filename

else
	echo "$filename does not exist"
fi

