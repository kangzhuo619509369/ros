#! /bin/bash

car=("BMW" "TOYOTA" "HONDA")
unset car[2]
car[2]="MESD"
echo "${!car[@]}"
echo "${#car[@]}"
