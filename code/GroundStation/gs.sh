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

NAME=groundstation
DAEMON=/data/gs/groundStation.sh

case "$1" in
  start)
        sleep 15;   # Waiting for Postgres to start up
        mv /data/gs/groundStation.log /data/gs/groundStation.log.old
	start-stop-daemon --start -b -c pi --verbose --pidfile /var/run/$NAME.pid \
		--oknodo --exec $DAEMON 
	echo "Started GroundStation...";
	exit 1
	;;
  stop)
	start-stop-daemon --stop --user pi --verbose --pidfile /var/run/$NAME.pid \
		--oknodo --exec $DAEMON
	;;
  *)
	echo "Usage: $0 start|stop" >&2
	exit 1
	;;
esac
