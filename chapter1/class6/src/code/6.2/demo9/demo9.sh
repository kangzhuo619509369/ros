#! /bin/bash

#ls -al 1>file1.txt 2>file2.txt
#ls +al 1>file1.txt 2>file2.txt
#ls +al>file1.txt 2>&1
ls +al >& file1.txt
