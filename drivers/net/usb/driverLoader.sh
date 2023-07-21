#!/bin/bash

echo "******************************************************************"
echo "******************************************************************"
echo "**             FIBOCOM Rmnet Driver Load Helper                 **"
echo "**       This script is used to guide the customer to           **"
echo "**       select the driver and set the driver mode.             **"
echo "**       The supported driver is the GobiNet driver.            **"
echo "** Support mode:                                                **"
echo "**  1. AT mode:used for AT command dialing                      **"
echo "**  2. QMI mode: used for IP aggregation and multi-pdn dialing  **"
echo "**     1. QMAP numner 1   : for IP aggregation                  **"
echo "**     1. QMAP number 2~5 : multi-pdn dialing                   **"
echo "******************************************************************"
echo "******************************************************************"

read -p "select mode (1:AT,2:QMI):" mode
if [[ $mode -eq 1 ]];
then
  echo "AT mode"
  export __QCRMCALL_MODE=1
  fibo_opts="EXTRA_CFLAGS+=-D__QCRMCALL_MODE"
elif [[ $mode -eq 2 ]];
then
  read -p "set QMAP number (1~5):" qmap_num
  if [[ $qmap_num -lt 1 ]] || [[ $qmap_num -gt 5 ]];
  then
    echo "qmap_num out of range."
    exit 0
  fi
  export __QMAP_NUM=$qmap_num
  fibo_opts="EXTRA_CFLAGS+=-D__QMAP_NUM=\"$qmap_num\""
else
  echo "mode must be 1 or 2."
  exit 0
fi

echo "current mode is $mode"
if [[ $mode -eq 2 ]];
then
  echo "current QMAP number is $qmap_num"
fi
read -p "Confirm(y/n):" confirm
if [[ "$confirm" != "y" ]] ;
then
  echo "Abort."
  exit 0
fi
make install $fibo_opts
