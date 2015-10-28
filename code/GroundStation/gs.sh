#!/bin/bash
#
# NAME:     gs
# PURPOSE:  INIT script for GroundStation Systems
# AUTHOR:   Joseph P Turner 
#
# REVISIONS:
# Ver     Date         Author              Description
# ------- ------------ ------------------- ---------------------------
# 1.0.0   28-OCT-2015  Joseph Turner       1. Initial release of script
#
### --------------------------------------------------------------------
#

NAME=groundStation
DAEMON=/data/gs/groundStation.sh

case "$1" in
  start)
        sleep 15;   # Waiting for Postgres to start up
        mv /data/gs/groundStation.log /data/gs/groundStation.log.old
	start-stop-daemon --start --user pi -b -c pi --verbose --make-pidfile --pidfile /var/run/$NAME.pid \
		--name "$NAME" --oknodo --exec $DAEMON 
	echo "Started GroundStation...";
	exit 1
	;;
  stop)
	killall -9 groundStation.pl
	# start-stop-daemon --stop --user pi --verbose --pidfile /var/run/$NAME.pid \
	#	--name "$NAME" --oknodo --exec $DAEMON
	echo Stopped
	;;
  *)
	echo "Usage: $0 start|stop" >&2
	exit 1
	;;
esac
