#! /bin/bash

: '
function funcName()
{
	echo "this is new function"
}
funcName'
function funcPrint()
{
	echo $1 $2 $3 $4
}

funcPrint Hi How are you
