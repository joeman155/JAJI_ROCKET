#!/bin/bash

NAME=groundstation
DAEMON=/data/gs/groundStation.sh

case "$1" in
  start)
        sleep 15;   # Waiting for Postgres to start up
        mv /data/gs/groundStation.log /data/gs/groundStation.log.old
	# start-stop-daemon --start -b -c pi --verbose --pidfile /var/run/$NAME.pid \
	start-stop-daemon --start -b -c pi --verbose --pidfile /var/run/$NAME.pid \
		--oknodo --exec $DAEMON 
	# su - pi -c "/data/gs/groundStation.pl 1 > /data/gs/groundStation.log 2>&1 &"

	# su - pi -c "nohup /data/gs/groundStation.pl &"
	# su - pi -c "/data/gs/groundStation.pl > /data/gs/groundStation.log 2>&1"
	echo "Started GroundStation...";
	exit 1
	;;
  stop)
	# killall -9 groundStation.pl
	start-stop-daemon --stop --user pi --verbose --pidfile /var/run/$NAME.pid \
		--oknodo --exec $DAEMON
	;;
  *)
	echo "Usage: $0 start|stop" >&2
	exit 1
	;;
esac
