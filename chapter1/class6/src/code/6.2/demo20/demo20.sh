#! /bin/bash

function funcCheck()
{
	returningValue="using function right now"
	#echo "$returningValue"
}

returningValue="i love mac"
echo $returningValue

funcCheck
echo $returningValue

