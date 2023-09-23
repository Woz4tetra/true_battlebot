#!/bin/bash
set -e

while IFS="" read -r p || [ -n "$p" ]
do
    if [ -d "$p" ];
    then
        echo "copy directory $p"
        sudo rsync -azR "$p" /tmp/multistage_copy
    else
        echo "copy glob $p"
        list=($p)
        echo "${list[@]}"
        for file in "${list[@]}"; 
        do
            echo "$file";
            sudo rsync -azR "$file" /tmp/multistage_copy
        done
        
    fi
done < /tmp/include_from_base.txt
