#/bin/bash

cat /etc/passwd | grep /bin/bash | awk -F: '{print $7}'|wc -l
