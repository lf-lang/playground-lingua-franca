#!bin/bash

script_dir=$PWD/scripts
cli=$PWD/cli.py

tests=("example.sh")

if [[ ! -d $script_dir ]] && [[ ! -f $cli ]]
then
    exit 1
else
    for i in ${tests[@]}
    do
        test_file=$script_dir/${i}
        if [ -f $test_file ]
        then
            sh $test_file $cli
        fi
    done
fi    