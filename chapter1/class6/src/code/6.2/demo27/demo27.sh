#! /bin/bash

echo "enter filename to search text from"
read filename

if [[ -f $filename ]]
then
	echo "enter the text to search"
	read grepVar
	grep $grepVar $filename
else
	echo "$filename does not exist"
fi
