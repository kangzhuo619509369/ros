#/bin/bash
find ./ -name "* *" | while read line
do
mv "$line" $(echo $line | sed 's/ //g')
done

