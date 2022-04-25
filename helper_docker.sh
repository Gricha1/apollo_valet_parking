#!/bin/bash
#docker/scripts/dev_start.sh
#echo $1
if [ $1 == --s ] 
then
    #sudo bash docker/scripts/dev_start.sh --dist testing
    sudo bash docker/scripts/dev_start.sh
    sudo bash docker/scripts/dev_into.sh   
else   
    if [ $1 == --in ]
    then
        sudo bash docker/scripts/dev_into.sh
    else
        if [ $1 == --off ]
        then 
            sudo bash docker/scripts/dev_start.sh stop
        else
            echo "commands: --s --in --off"
        fi
    fi
fi

