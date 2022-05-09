#!/bin/bash
################################################################################
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
################################################################################

# Script to tune open vins

NAME="voxl-open-vins-server"
CONFIG_FILE="/etc/modalai/${NAME}.conf"
GROUND_TRUTH_SUFFIX="/run/mpa/qvio/data.csv"

## all dummy values for now
## raw noise
SIGMA_W=(0 1 2 3)
SIGMA_WB=(0 1 2 3)
SIGMA_A=(0 1 2 3)
SIGMA_AB=(0 1 2 3)

## covariances
SIGMA_W_2=(0 1 2 3)
SIGMA_WB_2=(0 1 2 3)
SIGMA_A_2=(0 1 2 3)
SIGMA_AB_2=(0 1 2 3)

## features
CHI2_MULT=(0 1 2 3)
# noise per pixel
SIGMA_PIX=(0 1 2 3)
# covariance per pixel
SIGMA_PIX_2=(0 1 2 3)

curr_paramlist=()



## set most parameters which don't have quotes in json
set_param () {
    if [ "$#" != "2" ]; then
        echo "set_param expected 2 args"
        exit 1
    fi

    # remove quotes if they exist
    var=$1
    var="${var%\"}"
    var="${var#\"}"
    val=$2
    val="${val%\"}"
    val="${val#\"}"

    sed -E -i "/\"$var\":/c\	\"$var\":	$val," ${CONFIG_FILE}
}

## set string parameters which need quotes in json
set_param_string () {
    if [ "$#" != "2" ]; then
        echo "set_param_string expected 2 args"
        exit 1
    fi
    var=$1
    var="${var%\"}"
    var="${var#\"}"
    sed -E -i "/\"$var\":/c\	\"$var\":	\"$2\"," ${CONFIG_FILE}
}

set_param_string_last () {
    if [ "$#" != "2" ]; then
        echo "set_param_string expected 2 args"
        exit 1
    fi
    var=$1
    var="${var%\"}"
    var="${var#\"}"
    sed -E -i "/\"$var\":/c\	\"$var\":	\"$2\"" ${CONFIG_FILE}
}

construct_curr_paramlist () {
    if [ "$#" != "1" ]; then
        echo "construct_curr_paramlist expected 1 arg"
        exit 1
    fi
    index=$1
    # clear param list
    curr_paramlist=()
    echo ${SIGMA_W[$index]}
    curr_paramlist+=(${SIGMA_W[$index]})
    curr_paramlist+=(${SIGMA_W_2[$index]})
    curr_paramlist+=(${SIGMA_WB[$index]})
    curr_paramlist+=(${SIGMA_WB_2[$index]})
    curr_paramlist+=(${SIGMA_A[$index]})
    curr_paramlist+=(${SIGMA_A_2[$index]})
    curr_paramlist+=(${SIGMA_AB[$index]})
    curr_paramlist+=(${SIGMA_AB_2[$index]})
}

set_ov_params_from_list () {
    set_param imu_sigma_w ${curr_paramlist[0]}
    set_param imu_sigma_w_2 ${curr_paramlist[1]}
    set_param imu_sigma_wb ${curr_paramlist[2]}
    set_param imu_sigma_wb_2 ${curr_paramlist[3]}
    set_param imu_sigma_a ${curr_paramlist[4]}
    set_param imu_sigma_a_2 ${curr_paramlist[5]}
    set_param imu_sigma_ab ${curr_paramlist[6]}
    set_param imu_sigma_ab_2 ${curr_paramlist[7]}

    set_param msckf_chi2_multiplier ${curr_paramlist[8]}
    set_param msckf_sigma_px ${curr_paramlist[9]}
    set_param msckf_sigma_px_sq ${curr_paramlist[10]}

    set_param slam_chi2_multiplier ${curr_paramlist[8]}
    set_param slam_sigma_px ${curr_paramlist[9]}
    set_param slam_sigma_px_sq ${curr_paramlist[10]}
}

print_usage(){
    echo ""
    echo " Automated Vio Evaluation"
}

# make sure one argument is given
if [ "$#" -ne 1 ]; then
    print_usage
    exit 1
fi

## convert arguments to lower case for robustness
LOG_DIR=$(echo "$1" | tr '[:upper:]' '[:lower:]')

## get list of all directories within the given dir
dirlist=(`ls ${LOG_DIR}`)
dirlist_len=${#dirlist[@]}

## get some helper vars
paramlist_len=${#SIGMA_W[@]}
# paramlist_len=${#SIGMA_W_2[@]}
# paramlist_len=${#CHI2_MULT[@]}


## also will need "iterations" here, so we can change parameters within the same log
## define an array? of params (per param) all of the same len, grab len of one array and thats our iteration inner loop
for (( i=0; i<$dirlist_len; i++ ));
do
    results_csv="${LOG_DIR}${dirlist[$i]}.csv"
    echo ${results_csv}
    for (( j=0; j<$paramlist_len; j++ ));
    do
        # construct the list of parameters we are going to test out
        construct_curr_paramlist $j
        # need to write into the file a "header" with all ov params in use
        echo "sigma_w,sigma_w_2,sigma_wb,sigma_wb_2,sigma_a,sigma_a_2,sigma_ab,sigma_ab_2,chi2_mult,sigma_pix,sigma_pix_2" >> ${results_csv}
        printf "%s," "${curr_paramlist[@]}" >> ${results_csv}
        sed -i '$ s/.$//' ${results_csv}
        printf "\n" >> ${results_csv}

        # now, actually set those parameters
        set_ov_params_from_list

        # now, fire up the evaluate-vio process
        ground_truth="${LOG_DIR}${dirlist[$i]}${GROUND_TRUTH_SUFFIX}"
        voxl-evaluate-vio -f ${results_csv} -g ${GROUND_TRUTH_SUFFIX} &
        eval_pid=$!

        # next, fire up the open vins process
        voxl-open-vins-server -e ${LOG_DIR}${dirlist[$i]} &
        ov_pid=$!

        # finally, start the log
        voxl-replay -p ${LOG_DIR}${dirlist[$i]} &
        replay_pid=$!

        while ps -p $eval_pid > /dev/null
        do
            echo "$eval_pid is running"
            sleep 0.5
        done

        # once ov_eval process stops, then we're safe to continue onwards in the loop?
        # kill everything else just in case
        sudo kill -9 ${replay_pid}
        sudo kill -9 ${ov_pid}
    done
done
