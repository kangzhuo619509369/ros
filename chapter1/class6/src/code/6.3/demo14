#!/bin/bash

if [ $# != 2 ]
    then echo "option is not enough"
    exit 1
fi

if [ ! -d $1 -o ! -d $2 ]
    then echo "$1 and $2 must be directory"
    exit 1
fi

for file in $(du -a $1 | awk '$1>10 {print $2}' | awk 'FS="/" {print $2}' | uniq)
do
    if [ -e $2/$file -a -d $2/$file ]
	then echo "$file is a directory and exist!"
    else
	mv -i $1/$file $2
    fi
done
