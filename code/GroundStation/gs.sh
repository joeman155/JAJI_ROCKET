#!/bin/sh
case "$1" in
  'start')  # Start Ground Station
    echo -n "Starting GroundStation: "
    /home/root/hope/groundStation.pl >> /home/root/hope/groundStation.log 2>&1 &
    ;;

  'stop')   # Stop Ground Station
    echo -n "Stopping GroundStation: "
    killall -9 groundStation.pl
    ;;
esac

