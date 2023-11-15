#!/bin/bash

reset_bonetag_serial

if ! command -v mc_rtc_ticker &> /dev/null
then
  echo "mc_rtc_ticker could not be found, trying with ros version instead"
  rosparam set /mc_rtc_ticker/conf @BONETAG_CONTROLLER_ETC@/mc_rtc.yaml
  rosrun mc_rtc_ticker mc_rtc_ticker
else
  mc_rtc_ticker -f @BONETAG_CONTROLLER_ETC@/mc_rtc.yaml
fi
