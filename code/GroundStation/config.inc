#!/usr/bin/perl
#
# Name:  groundStation.pl
# Purpose:  To run on ground station and act as go-between for the Rocket
#           Launcher System and the end user with their tablet/phone device.
#

# GENERAL CONFIG
$home_dir = "/data/gs/";

# DATABASE CONFIG
my $db_string = "dbi:SQLite:dbname=" . $home_dir . "db/gs.db";  # SQLIte DB file

# SERIAL CONFIG
my $serial_port = "/dev/ttyO1";
my $serial_speed = 57600;

# DATE/TIME FORMAT
my($day, $month, $year) = (localtime)[3,4,5];
$month = sprintf '%02d', $month+1;
$day   = sprintf '%02d', $day;
my $rrmmdd =  $year+1900 . $month . $day;

# FILES
# Pressure/altitude
my %data;  # Holds altitude/air pressure data
load_air_data($home_dir . "air_data.txt");

# GPS_file
$gps_file = $home_dir . "out/gps_data" . $rrmmdd . ".txt";

# Measurements_file
$measurements_file = $home_dir . "out/measurements.txt";

# Cutdown file
$cutdown_req_file  = $home_dir . "run/cutdown_requested.txt";
$cutdown_init_file = $home_dir . "run/cutdown_initiated.txt";
`rm -f $cutdown_req_file`;
`rm -f $cutdown_init_file`;
$cutdown_initiated = 0;      # Indicates if cutdown has been initiated.

# No Photos
$nophotos_file = $home_dir . "run/nophotos.txt";
`rm -f $nophotos_file`;


# X-MODEM
# X-Modem packet file
$download_file_status = $home_dir . "run/download_file_status";
$x_modem_packet_num = $home_dir . "run/x_modem_packet";
`echo "" > $x_modem_packet_num`;
`echo 0 > $download_file_status`;

# PICTURE CONFIGURATIONS
my $filename = "";  
my $taking_picture = 0;      # Indicates if we are taking a picture at Rocket Launch System
$pic_download_offered = 0;   # How many times the Launch System has offered a picture for download
$pic_dl_freq = 5;            # How often to download a pic.i.e. download every 'pic_dl_freq'th pic offered


# INITIALISATIONS
$mode = 0;          # Setting default operating mode
$DEBUG = 1;         # Enable/Disable debugging


# INITIALISATIONS
my $radio_stats_count = 0;
my $file_num = 1;


# SENSOR CONFIGURATIONS
my $voltage_multiplier = 5.7 * 3.3 /1024;   # For measuring voltage on Rocket Launch System

# ((r2 + r1)/r2) * (1.8/1800)
my $gs_psu1_voltage_pin_file = "/sys/devices/ocp.3/helper.15/AIN1";
my $gs_psu1_voltage_multipler = ((1.5 + 10.1)/1.5) * (1.8/1800);
my $gs_psu1_voltage_ctr = 0; # we only want to get the voltage every now and then...we keep
                          # count of # of iterations with this.





return 1;
