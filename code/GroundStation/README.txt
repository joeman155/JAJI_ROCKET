Summary
-------
This is code for Rocket Launch System utilisnig the Rasberry-Pi 2 Model B


What it provides
-------
* Configuration items for Rasberry-Pi to be an Access Point
  and provide IP's over this WiFi Link

* PHP web page files for GroundStation web pages

* PERL scripts to interogate the RFD900+ modem and write results to
  various SQLITE tables.


Dependencies
-------
* Rasberry-Pi 2 Model B installed with Raspbian Linux version released
  on 5th May 2015

* USB WiFi Dongle installed and running (EDIMAX EW-7811Un Wireless-N Network Adapte)

* Lighttpd Web server

* SQLLite3 installed

* PHP working

* PiFace Shim RTC

* RFD900 modem attached

* Install libnl-1.1 and HostAPD

* Append following to /etc/sudoers

www-data ALL = (root) NOPASSWD: /sbin/shutdown


Installation
-------
1. Ensure all hardward dependencies are met.

i.e. - Install of PiFace Shim RTC
     - Plugging in WIFI adapter
     - Attached RFD900+ to the Rasberry-Pi on UART pins
     - Power Supply created and attached
     - Antenna attached to RFD900+ modem

2. 
Since this is a lot of work, an image has been made available:-

http://leederville.net/hab/resources/GS-PI2-2015MMDD-01.img.gz


gzip -d GS-PI2-2015MMDD-01.img.gz

.... Put this into microSD using Window Imager executable ***


3. Ensure RFD900+ modem is attached to /dev/ttyO1 and has a speed of 57600

4. Log on as user pi

5. Run following commands to get latest copy.

cd /data/src/JAJI_ROCKET/code/GroundStation
git pull

./install.sh

That should be it!


Updates
---------
To get updates of the Groundstation scripts:-

1. Ensure Ethernet cable is attached and Internet is available from Rasberry-Pi

2. Run the following as user root:-

cd /data/src/JAJI_ROCKET/code/GroundStation
git pull
./install.sh

NOTE: This will overwrite existing database file /data/gs/db/gs.db. If you want to keep this file, back it up BEFORE running the update above.
