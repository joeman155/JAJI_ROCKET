#!/bin/sh
#
# Name:  install.sh
# Purpose: To install Ground station
#



echo Creating GS directories
[ -d /data/web ] || mkdir /data/web
[ -d /data/web/out ] || mkdir /data/web/out
[ -d /data/gs ] || mkdir /data/gs
[ -d /data/gs/db ] || mkdir /data/gs/db
[ -d /data/gs/run ] || mkdir /data/gs/run
[ -d /data/gs/uploads ] || mkdir /data/gs/uploads
[ -d /data/gs/out/images ] || mkdir /data/gs/out/images
[ -d /data/gs/out/images/thumbnails ] || mkdir /data/gs/out/images/thumbnails

echo Permissions on files
touch /data/gs/run/download_file_status
chmod 777 /data/gs/db
chmod 777 /data/gs/run
chmod 777 /data/gs/uploads
chmod 777 /data/gs/run/download_file_status
chmod -R 777 /data/gs/out/images 


# Compile files
gcc -o Voltage_Reader_Master  Voltage_Reader/Master/Voltage_Reader_Master.c
cd ssdv
gcc -o ssdv main.c ssdv.c rs8.c
cd ../


echo Copying across files
cp -pr web/* /data/web/
cp air_data.txt /data/gs/
cp groundStation.sh /data/gs/
cp groundStation.pl /data/gs/
cp ssdv/ssdv /data/gs/
cp gs.sh /data/gs/
cp Voltage_Reader_Master /data/gs/

# DB Links
rm /data/web/out/images
ln -s /data/gs/out/images /data/web/out/images


echo Initialising the database...
[ -f /data/gs/db/gs.db ] && mv /data/gs/db/gs.db /data/gs/db/gs.db.bck
# Not using SQLITE3 database now. Will completely remove later.
# sqlite3 /data/gs/db/gs.db < tables.sql
# chmod 777 /data/gs/db/gs.db
psql rls -f pgsql_tables.sql



echo Finished

echo May need to update /data/web/config.inc
echo
echo Copy gs.sh to /etc/init.d/gs manually \(as user root\) 
