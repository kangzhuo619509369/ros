#! /bin/bash

select car in BMW TOYOTA HONDA
do
	#echo "You have select $car"
	case $car in
		BMW)
			echo "$car Selected";;
		TOYOTA)
			echo "$car Selected";;
		HONDA)
			echo "$car Selected";;
		*)
			echo "Please Input 1-3";;
	esac
done
