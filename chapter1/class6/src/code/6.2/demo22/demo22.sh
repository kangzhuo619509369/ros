#! /bin/bash

echo "enter filename from which you want to read"
read filename

if [[ -f "$filename" ]]
then
	while IFS= read -r line
	do
		echo "$line"
	done < $filename
else
	echo "$filename does not exits"
fi
