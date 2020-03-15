#! /bin/bash
: '
#echo "enter directory name to check"
echo "enter directory name to append"
read direct

if [ -d "$direct" ]
then
	echo "$direct exists"
else
	echo "$direct does not exist"
fi'

#echo "Enter the file name to create"
echo "Enter the filename"
read  filename
if [ -f "$filename" ]
then
	echo "enter the text that you want to append"
	read fileText
	echo "$fileText" >> $filename
else
	echo "$filename does not exist"
fi
touch $filename


