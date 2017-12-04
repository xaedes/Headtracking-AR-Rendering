#!/bin/bash

for f in test_*
do 
    echo ./$f "$@"
    ./$f "$@"
done
