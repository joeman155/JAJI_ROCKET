#!/bin/sh
#
# Name:  install.sh
# Purpose: To install Ground station
#

[ -d /data/web ] || mkdir /data/web
[ -d /data/web/out ] || mkdir /data/web/out
[ -d /data/web/out/images ] || mkdir /data/web/out/images

cp -pr web/* /data/web/

[ -d /data/gs ] || mkdir /data/gs
[ -d /data/gs/run ] || mkdir /data/gs/run
chmod 777 /data/gs/run
cp air_data.txt /data/gs/
cp groundStation.pl /data/gs/
cp gs.sh /data/gs/
cp config.inc /data/gs/


[ -f /data/gs/gs.db ] && mv /data/gs/gs.db /data/gs/gs.db.bck
sqlite3 /data/gs/gs.db < tables.sql
chmod 777 /data/gs/gs.db



echo Finished

echo May need to update /data/web/config.inc
