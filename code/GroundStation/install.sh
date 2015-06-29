#!/bin/sh
#
# Name:  install.sh
# Purpose: To install Ground station
#

cp -pr web/* /data/web/

mkdir /data/gs
mkdir /data/gs/run
chmod 777 /data/gs/run
cp air_data.txt /data/gs/
cp groundStation.pl /data/gs/
cp gs.sh /data/gs/
cp config.inc /data/gs/
