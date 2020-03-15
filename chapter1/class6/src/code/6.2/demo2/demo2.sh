#! /bin/bash

number=1
#while [ $number -lt 10 ]
while(($number<10)) 
do
	echo "$number"
	: '
	number=$(( $number+1 ))
	number=$(( $number+1 ))'
	#let number=number+1
	number=$((number+1))
done
