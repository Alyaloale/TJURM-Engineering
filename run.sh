#!/bin/bash

blue="\033[1;34m"
yellow="\033[1;33m"
reset="\033[0m"

include_count=$(find include  -type f \( -name "*.cpp" -o -name "*.h" \) -exec cat {} \; | wc -l)
src_count=$(find src  -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.txt" \) -exec cat {} \; | wc -l)
total=$((include_count + src_count))



if [ ! -d "/etc/openrm_engineering" ]; then 
    mkdir /etc/openrm_engineering
    cp -r config/* /etc/openrm_engineering/
    chmod -R 777 /etc/openrm_engineering
fi


if [ ! -d "build" ]; then 
    mkdir build
fi


while getopts ":rcg:" opt; do
    case $opt in
        r)
            echo -e "${yellow}<--- delete 'build' --->\n${reset}"
            rm -rf build
            mkdir build
            shift
            ;;

        c)
            cp -r config/* /etc/openrm_engineering/
            chmod -R 777 /etc/openrm_engineering
            shift
            ;;
        g)
            git_message=$OPTARG
            echo -e "${yellow}\n<--- Git $git_message --->${reset}"
            git pull
            git add -A
            git commit -m "$git_message"
            git push
            exit 0
            shift
            ;;
        \?)
            echo -e "${red}\n--- Unavailable param: -$OPTARG ---\n${reset}"
            ;;
        :)
            echo -e "${red}\n--- param -$OPTARG need a value ---\n${reset}"
            ;;
        esac
    done


echo -e "${yellow}<--- Start CMake --->${reset}"
cd build
cmake ..


echo -e "${yellow}\n<--- Start Make --->${reset}"
max_threads=$(cat /proc/cpuinfo | grep "processor" | wc -l)
make -j "$max_threads"


echo -e "${yellow}\n<--- Total Lines --->${reset}"
echo -e "${blue}        $total${reset}"


echo -e "${yellow}\n<--- Run Code --->${reset}"
cp TJURM-Engineering /usr/local/bin/
./TJURM-Engineering

echo -e "${yellow}<----- OVER ----->${reset}"