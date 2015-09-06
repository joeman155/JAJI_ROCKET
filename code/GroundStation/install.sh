#!/bin/sh
#
# Name:  install.sh
# Purpose: To install Ground station
#



echo Creating GS directories
[ -d /data/web ] || mkdir /data/web
[ -d /data/web/out ] || mkdir /data/web/out
[ -d /data/web/out/images ] || mkdir /data/web/out/images
[ -d /data/gs ] || mkdir /data/gs
[ -d /data/gs/db ] || mkdir /data/gs/db
[ -d /data/gs/run ] || mkdir /data/gs/run
[ -d /data/gs/uploads ] || mkdir /data/gs/uploads

echo Permissions on files
touch /data/gs/run/download_file_status
chmod 777 /data/gs/db
chmod 777 /data/gs/run
chmod 777 /data/gs/uploads
chmod 777 /data/gs/run/download_file_status


# Compile files
gcc -o Voltage_Reader_Master  Voltage_Reader/Master/Voltage_Reader_Master.c

echo Copying across files
cp -pr web/* /data/web/
cp air_data.txt /data/gs/
cp groundStation.pl /data/gs/
cp gs.sh /data/gs/
cp Voltage_Reader_Master /data/gs/


echo Initialising the database...
[ -f /data/gs/db/gs.db ] && mv /data/gs/db/gs.db /data/gs/db/gs.db.bck
sqlite3 /data/gs/db/gs.db < tables.sql
chmod 777 /data/gs/db/gs.db



echo Finished

echo May need to update /data/web/config.inc
