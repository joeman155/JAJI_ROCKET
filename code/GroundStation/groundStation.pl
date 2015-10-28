#!/usr/bin/perl
#
# Name:  groundStation.pl
# Purpose:  To run on ground station and act as go-between for the Rocket
#           Launcher System and the end user with their tablet/phone device.
#

# use lib '/home/root/hope/modules/lib/perl/5.14.2';
# use lib '/home/root/hope/modules/share/perl/5.14.2';


# LOAD MODULES
# use strict;
use warnings;
use IO::Socket;
use threads;
use Thread::Queue;
use Device::SerialPort qw( :PARAM :STAT 0.07 );
use DBI;
use POSIX;
use Switch;




#------------------ CONFIGURATION -------------------#
# GENERAL CONFIG
$home_dir = "/data/gs/";

# DATABASE CONFIG
my $db_string    = "dbi:SQLite:dbname=" . $home_dir . "db/gs.db";  # SQLIte DB file
my $pg_db_string = "dbi:Pg:dbname=rls";
my $dbh = initialise_db();

# SERIAL CONFIG
my $serial_port = "/dev/ttyAMA0";
# my $serial_port = "/dev/ttyACM0";
my $serial_speed = 57600;
# my $serial_speed = 9600;

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

# Sources
$RLS_SOURCE = "RLS";
$GS_SOURCE  = "GS";

# X-Modem packet file
$download_file_status = $home_dir . "run/download_file_status";
$picture_packet_num = $home_dir . "run/picture_packet";
`echo "" > $picture_packet_num`;
`echo 0 > $download_file_status`;

# SSDV
$ssdv_file = $home_dir . "out/ssdv";
unlink $ssdv_file;
$SSDV_EXEC = $home_dir . "ssdv";
$ssdv_transfer = 0;   # 1 = transferring, 0 = not transferring
$ssdv_packet_num = 0; # To keep track of how many packets we have received.

# PICTURE CONFIGURATIONS
my $saving_picture = 0;      # Indicates if we are taking a picture at Rocket Launch System
$pic_download_offered = 0;   # How many times the Launch System has offered a picture for download
$image_error = 0;            # Indicates if an issue with images
$image_dir   = $home_dir . "out/images/";


# INITIALISATIONS
$DEBUG = 1;         # Enable/Disable debugging


# INITIALISATIONS
my $radio_stats_count = 0;
my $file_num = 1;


# SENSOR CONFIGURATIONS
# ((r2 + r1)/r2) * (3.3/1024)
my $gs_voltage_measurements = 1;    # 1 = Gathering enabled, 0 = Disabled
my $gs_psu1_voltage_exec = $home_dir . "Voltage_Reader_Master";
my $gs_psu1_voltage_multiplier = ((3.6 + 1)/1) * (3.3/1024);
my $gs_psu1_voltage_ctr = 0; # we only want to get the voltage every now and then...we keep
                          # count of # of iterations with this.


#----------END OF CONFIGURATION -------------------#



print "GroundStation started....beginning Serial Port initialisation...\n";



# INITIALISE THE SERIAL PORT
my $port=Device::SerialPort->new($serial_port) || die "Can't open $serial_port: $!\n";;
$port->read_const_time(200);        # const time for read (milliseconds)
$port->read_char_time(5);           # avg time between read char
my $STALL_DEFAULT=10;               # how many seconds to wait for new input
my $timeout=$STALL_DEFAULT;

eval {
  $port->baudrate($serial_speed);
  $port->parity("none");
  $port->databits(8);
  $port->stopbits(1); 
  };
if (my $e = $@)
  {
   print("Issues initialising Serial modem. Check It is connected.\n");
   print("Error: " . $e . "\n");
   exit;
  }


$port->purge_tx;
$port->purge_rx;
print "Connected to Serial Port and Listening...\n";


$port->are_match("\r\n");


# Initalise Continuity Test result...so it is 3 (Invalidated ... requiring testing)
# If the "C" record indicates it needs testing...no need to insert.
$v_ct_status = get_last_status("C");
if (!defined $v_ct_status || $v_ct_status != 3) {
   set_launch_console_attribute("C", 3, "System Startup");
}

# Initialise Cutdown results...so it is 3 (Not been run yet...)
$v_cutdown_status = get_last_status("K");
if (!defined $v_cutdown_status || $v_cutdown_status != 3) {
   set_launch_console_attribute("K", 3, "System Startup");
}

# Initialise photos
$v_nophotos_status = get_last_status("N");
if (!defined $v_nophotos_status || $v_nophotos_status != 1) {
   set_launch_console_attribute("N", 1, "System Startup");
}

# Initialise Launch status
$v_launch_status = get_last_status("L");
if (!defined $v_launch_status || $v_launch_status != 9) {
   set_launch_console_attribute("L", 9, "System Startup");
}

# Initialise Profile status
$v_profile_status = get_last_status("I");
if (!defined $v_profile_status || $v_profile_status != 3) {
   set_launch_console_attribute("I", 3, "System Startup");
}


# Commence Serial Port monitoring
monitor_systems();





## MAIN SERIAL PORT MONITOR ROUTINE
sub monitor_systems()
{

while (1 == 1)
{
    my $serial_rx = "";
    until ("" ne $serial_rx) {
       $serial_rx = $port->lookfor;       # poll until data ready
       die "Aborted without match\n" unless (defined $serial_rx);

       # Get GS PSU1 voltage supply reading and put into table
       if ($gs_voltage_measurements == 1 && $gs_psu1_voltage_ctr > 300) {
          $v_voltage = get_gs_psu_voltage($gs_psu1_voltage_exec, $gs_psu1_voltage_multiplier);
          insert_gs_psu_voltage(1, $v_voltage);
          $gs_psu1_voltage_ctr = 0;
       } else  {
          $gs_psu1_voltage_ctr = $gs_psu1_voltage_ctr + 1;
       }
    }


    # Clean out any non-printable characters...just remove them.
    # $serial_rx =~ s/[^[:print:]]//g;

    $str = "** Decoding Serial RX: '" . $serial_rx . "'\n" if $DEBUG;
    print $str if $DEBUG;

    $image_error = 0; # Reset image error back to no error

    # Decode what we receive
    $result = decode_rx($serial_rx);
    
    if (length($result) > 0) {
       $str = $result;
       print "** LOGGING MESSAGE: $str \n" if $DEBUG;
       log_message($str);


## SEE IF MENU BEING PRESENTED BY RLS
      if ($result =~ /Menu/) {

        # Look for requests from Web Connected Systems.
        $v_request_processed = process_requests();

        # If in Power status, we want to disable activities that could
        # interfere (delay) the launch sequence. So we get the status here
        $v_power_status = get_last_status("P");


        # ALSO... we only want to process other requests IF no request was 
        # processed above
        if ($v_request_processed == 0) {



            # We have 3 second delay after getting heartbeat.... so we quickly get
            # stats on state of link
            # Every 15 iterations...get stats
            # NOTE: We only want to get status IF the power is not on...
            # stats gather is to time consuming and unnecessary during launches
            if ($v_power_status == 0) {
               if ($radio_stats_count > 50) {
                   get_radio_stats();
                   $radio_stats_count = 0;
               } else {
                   ++$radio_stats_count;
               }
            }

           # Got to the end, so we want to exit
           print "** No Requests, so exit the menu...\n" if $DEBUG;
           # sendModemRequest("R00", "A00", 0);
           sendModemRequest("R00", "", 0);

         }
       }
    }
  }
}

exit;



sub decode_rx()
{

  ($p_line) = @_;

  # See what data we have and respond to it
  if ($p_line =~ /^M$/)
  {
    $v_result = "Menu";
  } elsif ($p_line =~ /^H:([0-9]+)$/)
  {
    $v_result = "Heartbeat Count: " . $1;
    insert_heartbeat($1);
  } elsif ($p_line =~ /^L\/R(.*$)/)
  {
    $v_result = "Radio Signal: L/R: " . $1;
  } elsif ($p_line =~ /^S$/)
  {
    $v_result = "Powering up RLS";
  } elsif ($p_line =~ /^G$/)
  {
    $v_result = "RLS powered up";
  } elsif ($p_line =~ m/^D00:(.+),(.+),(.+),(.+),(.+)$/)
  {
    $v_air_pressure = $1;
    $v_internal_temp = $2;
    $v_external_temp = $3;
    $v_cpu_voltage = sprintf("%.2f", $4);
    $v_ign_voltage = sprintf("%.2f", $5);
    $v_alt = get_altitude($v_air_pressure);
    my %measurements = (
		"Internal Temperature"	=> $v_internal_temp,
		"External Temperature"	=> $v_external_temp,
		"Air Pressure"		=> $v_air_pressure,
		"Est Altitude"		=> $v_alt,
		"CPU Voltage"		=> $v_cpu_voltage,
		"IGN Voltage"		=> $v_ign_voltage
			);
    insert_measurements("D00", $RLS_SOURCE, \%measurements); 
    $v_result = "Measurements: " . $p_line;
  } elsif ($p_line =~ m/^D01:(.+),(.+),(.+),(.*),(.+),(.+),(.+),(.+)$/)
  {
    $v_lat         = $1;
    $v_long        = $2;
    $v_alt         = $3;
    $v_gps_date    = $4;
    $v_gps_time    = $5;
    $v_course      = $6;
    $v_speed       = $7;
    $v_satellites  = $8;
    $v_result = "GPS\nLatitude: " . $v_lat . "\nLongitude: " . $v_long . "\nAltitude: " . $v_alt . "\nDate: " . $v_gps_date . "\nTime: " . $v_gps_time . "\nSpeed: " . $v_speed . "\nCourse: " . $v_course . "\nSatellites: " . $v_satellites . "\n";
    $v_line = $4 . "," . $5 . "," . $v_lat . "," . $v_long . "," . $3 . "\n";
    open(my $gps_fh, '>>' . $gps_file) or die "issue opening gps file";
    print $gps_fh $v_line;
    close($gps_fh);

    insert_gps($v_lat, $v_long, $v_alt, $v_gps_date, $v_gps_time, $v_speed, $v_course, $v_satellites);

    # Generate the kml file each time we have more gps data
    create_kml($gps_file);
  } elsif ($p_line =~ m/^D02:(.+)$/)
  {
    $v_time = $1;
    $v_result = "Uptime of " . $v_time . "\n";
    my %measurements = (
         "Uptime"	=> $v_time
			);
    insert_measurements("D02", $RLS_SOURCE, \%measurements);
  } elsif ($p_line =~ /^D03$/)
  {
    $v_result = "Taking picture";
  } elsif ($p_line =~ /^D04:(.*$)/)
  {
    set_lc_profile_status($1);
    $v_result = "Profile setting: " . $1;
  } elsif ($p_line =~ /^D14:(.*$)/)
  {
    set_launch_console_attribute("N", 1, "Download photos");
    set_lc_photos_enabled_status($1);
    $v_result = "Photos Enabled setting: " . $1;
  } elsif ($p_line =~ /^D06:(.*),(.*),(.*),(.*),(.*),(.*),(.*),(.*),(.*),(.*)$/)
  {
    my %imu;
    
    $imu{'roll'}  = $1;
    $imu{'pitch'} = $2;
    $imu{'yaw'}   = $3;
    $imu{'gyrox'} = $4;
    $imu{'gyroy'} = $5;
    $imu{'gyroz'} = $6;
    $imu{'accx'}  = $7;
    $imu{'accy'}  = $8;
    $imu{'accz'}  = $9;
    $imu{'timer'} = $10;

    insert_imu(1, \%imu);
    
    $v_result = "IMU: Roll " . $1 . ", Pitch " . $2 . ", Yaw " . $3 . " ....";
  } elsif ($p_line =~ /^D07:(.*$)/)
  {
    set_lc_power_status($1);
    $v_result = "Power status: " . $1;
  } elsif ($p_line =~ /^D08:(.*$)/)
  {
    set_lc_arm_status($1);
    $v_result = "Arm status: " . $1;
  } elsif ($p_line =~ /^D10$/)
  {
    $v_result = "Finished taking picture";
  } elsif ($p_line =~ /^D11$/)
  {
    $v_result = "Finished writing picture to microSD";
    $saving_picture = 0;
  } elsif ($p_line =~ /^D12:(.*)$/)
  {
    $v_file = $1 . "-" . time();

    # Signify that transfer has finished
    $ssdv_transfer = 0;
    $ssdv_packet_num = 0;

    # update external file...which webpage relies upon to advise user of 
    # status
    `echo "" > $picture_packet_num`;
    `echo 0 > $download_file_status`;

    # Generate new file name
    $out_image_file = $image_dir . $v_file . ".jpg";

    # Convert ssdv file to iamge file
    `$SSDV_EXEC -d $ssdv_file > $out_image_file`;
    $ssdv_file_bk = $ssdv_file . "_" . $v_file;

    # Back up SSDV file incase we want to do some analysis on it later
    `cp $ssdv_file $ssdv_file_bk`;

    # Remove SSDV temp file so it is ready for next file
    unlink $ssdv_file;
    $v_result = "Finished sending picture";
  } elsif ($p_line =~ /^D13:(.*)$/)
  {
    $ssdv_transfer = 1;
    $ssdv_packet_num++;

    # update external file...which webpage relies upon to advise user of 
    # status
    `echo $ssdv_packet_num > $picture_packet_num`;
    `echo $ssdv_transfer > $download_file_status`;

    $ssdv_packet = $1;
    $val = pack "H*", $ssdv_packet;
    open (my $ssdv_fh, ">>" . $ssdv_file) or die "Cannot create ssdv";
    print $ssdv_fh $val;
    close($ssdv_fh);
  } elsif ($p_line =~ /^E00$/)
  {
    $v_result = "Error initialising microSD Card";
  } elsif ($p_line =~ /^E01$/)
  {
    $v_result = "Error writing GPS information to microSD Card";
  } elsif ($p_line =~ /^E02$/)
  {
    $v_result = "Error writing Log to microSD Card";
  } elsif ($p_line =~ /^E03$/)
  {
    $v_result = "Error writing image to microSD Card";
    $image_error = 1;
  } elsif ($p_line =~ /^E04$/)
  {
    $v_result = "Error/timeout taking picture. Took more than 90 seconds.";
    $image_error = 1;
  } elsif ($p_line =~ /^Q:(.*)$/)
  {
    $v_result = "Did not recognise response from station. Response: " . $1;
  } elsif ($p_line =~ /^W$/)
  {
    $v_result = "Timeout while waiting for user menu option to be made.";
  } elsif ($p_line =~ /^E05$/)
  {
    $v_result = "Error downloading picture to GroundStation.";
  } elsif ($p_line =~ /^B$/)
  {
    $v_result = "Reached Max Altitude - Cutdown initiated";
  } elsif ($p_line =~ m/^T:(.*)$/)
  {
    $v_seconds = $1/1000;
    $v_minutes = $v_seconds/60;
    $v_hours   = $v_minutes/60;
    $v_hours_rounded = floor($v_hours);
    $v_minutes_rounded = floor(60 * ($v_hours - $v_hours_rounded));
    $v_seconds_rounded = floor($v_seconds - (3600 * $v_hours + 60 * $v_minutes_rounded));
    $v_result = "Time since power turned on is " . $v_hours_rounded . "hours and " . $v_minutes_rounded . "minutes and " . $v_seconds_rounded . "seconds.\n";
  } elsif ($p_line =~ /^(\.)+$/)
  {
    if ($saving_picture == 1)
    {
      return "";
    }
    else
    {
      $saving_picture = 1;
      return "Saving pic.";
    }
  }
  else
  {
   $v_result = "Cannot Decode: " . $p_line . "\n";
  }

 return $v_result;
}



sub create_kml()
{
local ($gps_file) = @_;

open (my $kml_file, ">" . $home_dir . "out/hab_gps.kml") or die "Cannot create hab_gps.kml";

$startline = << "STARTLINE";
<?xml version="1.0" encoding="utf-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<Placemark>
<name>Cairns RLS launch - JJ Team</name>
<LookAt>
<longitude>145.65888</longitude>
<latitude>-16.75229</latitude>
<altitude>1000</altitude>
<range>500.882995</range>
<tilt>60.768762</tilt>
<heading>270.131493</heading>
</LookAt>
<styleUrl>#msn_icon12</styleUrl>
<LineString>
<tessellate>1</tessellate>
<altitudeMode>absolute</altitudeMode> 
<coordinates>
STARTLINE

print $kml_file $startline;

open FILE, "<", $gps_file or die "No GPS Data";
while ($line = <FILE>)
{
 
  $line =~ /^(.*),(.*),(.*),(.*),(.*)$/;

  print $kml_file $4 . "," . $3 . "," . $5 . "\n";
$points .= << "HERE"
   <Placemark>
     <name>$1, $2</name>
     <Point>
       <coordinates>$4,$3,$5</coordinates>
     </Point>
   </Placemark>
HERE

}

close(FILE);

$finishline = << "FINISHLINE";
</coordinates>
</LineString>
</Placemark>
FINISHLINE

print $kml_file $finishline;

print $kml_file $points;
$finishpoints = << "FINISHPOINTS";
</Document>
</kml>
FINISHPOINTS

print $kml_file $finishpoints;

close($kml_file);

}



# Place = 0 = Ground station
#       = 1 = RLS
sub log_radio_stats($$)
{
 local ($p_place, $p_stats) = @_;

    # Initialise DB connection
    # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

    # Put in DB
    $query = "INSERT INTO radio_stats_t (place, stats) values (?,?)";

    $sth = $dbh->prepare($query);
    $sth->execute($p_place, $p_stats);

    # $dbh->disconnect();
}



sub log_message($)
{
  local($message) = @_;

  # Only insert message, if one was provided
  if ($message)
  {
    # Initialise DB connection
    # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

    # Put in DB
    $query = "INSERT INTO messages_t (message) values (?)";

    $sth = $dbh->prepare($query);
    $sth->execute($message);

    $sth->finish();
    # $dbh->disconnect();

  }
}


# Insert Measurements
# - A group of measurements
sub insert_measurements()
{
 local ($group_name, $source, $measurements_hash) = @_;

 %measurements = %$measurements_hash;

 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Insert a Group measurement record
 $query = "INSERT INTO measurement_group_t (group_name, source)
           VALUES (?,?)";

 $sth = $dbh->prepare($query);
 $sth->execute($group_name, $source);
 $sth->finish();

 $query = "SELECT currval('measurement_group_id_seq')";
 $sth = $dbh->prepare($query);
 $sth->execute();

 ($v_group_id) = $sth->fetchrow_array();
 $sth->finish();

 # Insert Measurements
 @measurement_names = keys %measurements;
 for my $measurement (@measurement_names) {
   insert_measurement($v_group_id, $measurement, $measurements{$measurement});
 }

 # $dbh->disconnect();
}


# Insert Measurement
sub insert_measurement()
{
 local($group_id, $measurement_name, $measurement_value) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "INSERT INTO measurement_t (group_id, name, value)
                   values (?,?,?)";

 $sth = $dbh->prepare($query);
 $sth->execute($group_id, $measurement_name, $measurement_value);
 
 $sth->finish();
 # $dbh->disconnect();
}


sub insert_heartbeat()
{
 local($heartbeat) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;


 $query = "INSERT INTO heartbeat_t (heartbeat)
 	values (?)";

 $sth = $dbh->prepare($query);
 $sth->execute($heartbeat);
 
 $sth->finish();
 # $dbh->disconnect();
}


sub insert_gps()
{
 local($latitude, $longitude, $height, $gps_date, $gps_time, $gps_speed, $gps_course, $satellites) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "INSERT INTO gps_t (latitude, longitude, height, speed, course, satellites, gps_date, gps_time)
                   values (?,?,?,?,?,?,?,?)";

 $sth = $dbh->prepare($query);
 $sth->execute($latitude, $longitude, $height, $gps_speed, $gps_course, $satellites, $gps_date, $gps_time);
 
 $sth->finish();
 # $dbh->disconnect();
}



sub get_gs_psu_voltage($$)
{
 local($gs_exec, $p_multiplier) = @_;

 my $A0_value = `$gs_exec 2 | grep A0`;

 $A0_value =~ /^A0:(.*)$/;
 $val = $1;
 $voltage = $v_cpu_voltage = sprintf("%.2f", $val * $gs_psu1_voltage_multiplier);

 # DEBUGGING
 # print "Voltage: $voltage \n";

 return $voltage;
}


sub insert_gs_psu_voltage($$)
{
 local($p_psu_id, $p_voltage) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "INSERT INTO gs_psu_voltage_t (voltage, psu_id)
           values (?,?)";

 $sth = $dbh->prepare($query);
 $sth->execute($p_voltage, $p_psu_id);

 $sth->finish();
 # $dbh->disconnect(); 
}


sub run_at_command($;$)
{
 local($cmd, $p_delay) = @_;

 my $delay = (defined $p_delay) ? $p_delay : 2;

 $port->write($cmd . "\r\n");

 return get_modem_response($delay);
}



sub enter_at_mode()
{
   select(undef,undef,undef,1);
   $port->write("+++");
   select(undef,undef,undef,0.3);

   return get_modem_response(0.3);
}



sub exit_at_mode()
{
 return run_at_command("ATO", 0.3);
}



sub get_modem_response()
{
 local ($p_delay) = @_;

 my $delay = (defined $p_delay) ? $p_delay : 2;

  my $received = "";
  my $receive_start_time = time;
  my $done = 0;

  do {
    my $count_in_tmp = 0;
    my $received_tmp;
    ($count_in_tmp, $received_tmp) = $port->read(132);
    $received .= $received_tmp;
    $count_in += $count_in_tmp;

  if (time > $receive_start_time + 2) {
      # wait for timeout, give the message at least three second
      $done = 1;
    }

  } while(!$done);

 return $received;
}


sub get_radio_stats()
{

  enter_at_mode();

  $stats = run_at_command("ATI7", 1);
  log_message("GND: " . $stats);
  log_radio_stats (0, $stats);

  $stats = run_at_command("RTI7", 1.5);
  log_message("RLS: " . $stats);
  log_radio_stats (1, $stats);

  exit_at_mode();

}



# Load air-pressure vs Altitude, so we can use this to approximate altitude.
sub load_air_data($)
{
  local ($file) = @_;
  open FILE, $file or die $!;
  local $altitude;
  local $pressure;

  while ($line = <FILE>) {
     $line =~ /^(.*) (.*)$/;

     $altitude = $1;
     $pressure = $2;

     $data{$pressure} = $altitude;

  }
}


# Based on air pressure, get the approximate altitude 
sub get_altitude()
{
 local ($pressure) = @_;

 my $match = 0;
 $prev_key = 101.34;
 
 # Convert pressure (in Pa) to kPa
 $pressure = $pressure/1000;

 foreach my $key (sort {$b <=> $a} keys %data) {
   if ($pressure > $key && $pressure < $prev_key) {
      $altitude = $data{$key} -  ($data{$key} - $data{$prev_key}) * ($key - $pressure)/($key - $prev_key);
      last;
   }
   $match = 1;
   $prev_key = $key;
 }

 # Unable to find a match...off the charts
 if ($match == 0) {
   return 0;
 }


 return int($altitude/3.2808399);
}



# Get oldest request that hasn't been acted on yet
sub dequeue_request()
{
 $v_id = 0;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Get the earliest request (FIFO)
 $query = "SELECT min(id)
           FROM   requests_t
           WHERE  status_code = ?";

 $sth = $dbh->prepare($query);
 $sth->execute('C');
 
 ($v_id) = $sth->fetchrow_array();
 $sth->finish();

 if (defined($v_id)) {
    if ($v_id != "") {
       $query = "UPDATE requests_t set status_code = 'P' WHERE id = $v_id";

       $sth = $dbh->prepare($query);
       $sth->execute();
       $sth->finish();
    }
 }

 # $dbh->disconnect();

 return $v_id;
}



# Get the Request Code
sub get_request_code($)
{
 local ($p_request_id) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Get request_code for given request ID
 $query = "SELECT request_code
           FROM   requests_t
           WHERE  id = $p_request_id";


 $sth = $dbh->prepare($query);
 $sth->execute();

 ($v_request_code) = $sth->fetchrow_array();
 $sth->finish();

 # $dbh->disconnect();

 return $v_request_code;

}




# Look for requests to pick up and process
# Returns 0 if no processes
#         1 if there is a process
sub process_requests()
{
 $v_req_id = 0; # Default value.

 # Look for requests from Web Connected Systems.
 $v_req_id = dequeue_request();

 # IF we have got a request...attempt to process it.
 if (defined($v_req_id)) {
    # Perform request (transmission)
    print "** Got Request ID: $v_req_id to perform.\n" if $DEBUG;

    $v_request_code = get_request_code ($v_req_id);
    print "** Request Code is " . $v_request_code . "\n" if $DEBUG;

    if ($v_request_code =~ /P/) { 
       print "** Power request...\n" if $DEBUG;

       set_launch_console_attribute("P", -1, "Pending");
       $v_result = sendModemRequest("R01", "A01", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       set_lc_power_status ($v_result);     # Update database with new POWER status

       # If we turn power off, we want to set continuity test to 'un tested'
       if ($v_result == 0) {
          $v_ct_status = get_last_status("C");
          if (!defined $v_ct_status || $v_ct_status != 3) {
             set_launch_console_attribute("C", 3, "Power turned off");
          }
       }
    } elsif ($v_request_code =~ /^A/) {
       print "** Arm request...\n" if $DEBUG;

       set_launch_console_attribute("A", -1, "Pending");
       $v_result = sendModemRequest("R02", "A02", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       set_lc_arm_status ($v_result);       # Update database with new ARM status

       # If ARM successful (which means extra continuity test that was done
       # was successful), so we set it as tested fine here.
       if ($v_result == 1) {
          $v_ct_status = get_last_status("C");
          if (!defined $v_ct_status || $v_ct_status != 1) {
             set_launch_console_attribute("C", 1, "Success during Arm");
          }
       }

       # If the attempt to ARM returns value of 3, this means continuity test failed
       # We need to update accordingly
       if ($v_result == 3) {
          $v_ct_status = get_last_status("C");
          if (!defined $v_ct_status || $v_ct_status != 0) {
             set_launch_console_attribute("C", 0, "Failed during Arm");
          }
       }
    } elsif ($v_request_code =~ /^I/) {
       print "** Profile change request ...\n" if $DEBUG;

       set_launch_console_attribute("I", -1, "Pending");
       $v_result = sendModemRequest("R10", "A10", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       # Based on result, set appropriate INFO message.
       if ($v_result == 2) {
          $v_profile_msg = "IMU Profile set";
       } elsif ($v_result == 1) {
          $v_profile_msg = "Standard Profile set";
       } elsif ($v_result == 0) {
          $v_profile_msg = "Timed out waiting for response";
       } else {
          $v_ct_msg = "Unknown profile";
       }

       set_launch_console_attribute("I", $v_result, $v_profile_msg);

    } elsif ($v_request_code =~ /^C/) {
       print "** Continuity request...\n" if $DEBUG;

       set_launch_console_attribute("C", -1, "Pending");
       $v_result = sendModemRequest("R03", "A03", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       # Based on result, set appropriate INFO message.
       if ($v_result == 2) {
          $v_ct_msg = "Failed: Power not on";
       } elsif ($v_result == 0) {
          $v_ct_msg = "Failed. Dis-continuous";
       } elsif ($v_result == 1) {
          $v_ct_msg = "Successful";
       } else {
          $v_ct_msg = "Unknown result.";
       }


       set_launch_console_attribute("C", $v_result, $v_ct_msg);
    } elsif ($v_request_code =~ /^M/) {
       print "** Invalidating previous Launch...\n" if $DEBUG;

       set_launch_console_attribute("L", 9, "Resetting Launch Status");
       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       # Send 'exit' menu, so we get back as quickly as possible to enter launch command
       sendModemRequest("R00", "A00", 0);
    } elsif ($v_request_code =~ /^T/) {
       print "** Invalidating previous Continuity Test...\n" if $DEBUG;

       set_launch_console_attribute("C", 3, "Invalidate Continuity Test");
       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       # Send 'exit' menu, so we get back as quickly as possible to enter launch command
       sendModemRequest("R00", "A00", 0);
    } elsif ($v_request_code =~ /^L/) {
       print "** Launch request...\n" if $DEBUG;

       set_launch_console_attribute("L", -1, "Pending");
       $v_result = sendModemRequest("R04", "A04", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status of request to FINISHED

       # Based on results, set appropriate Info Message
       if ($v_result == 1) {
          $v_launch_msg = "Successful Launch";
       } elsif ($v_result == 2) {
          $v_launch_msg = "Failed: Power is off";
       } elsif ($v_result == 3) {
          $v_launch_msg = "Failed: Not Armed";
       } elsif ($v_result == 4) {
          $v_launch_msg = "Failed: Continuity failed";
       } 
       set_launch_console_attribute("L", $v_result, $v_launch_msg);


       # Update Continuity status after launch
       $v_ct_status = get_last_status("C");
       if (!defined $v_ct_status || $v_ct_status != 3) {
          set_launch_console_attribute("C", 3, "Reset after launch");
       }
    } elsif ($v_request_code =~ /^N/) {
       print "** Photos on/off request...\n" if $DEBUG;

       set_launch_console_attribute("N", -1, "Pending");
       $v_result = sendModemRequest("R11", "A11", $v_req_id);

       # $v_photos_status = get_last_status("N", 1);
       if ($v_result == 1) {
          set_launch_console_attribute("N", 1, "Download photos");
       } else {
          set_launch_console_attribute("N", 0, "No photos");
       }

       setRequestStatus  ($v_req_id, "F");  # Set status to finished
    } elsif ($v_request_code =~ /^K/) {
       print "** Cutdown request...\n" if $DEBUG;

       set_launch_console_attribute("K", -1, "Pending");
       $v_result = sendModemRequest("R07", "A07", $v_req_id);

       setRequestStatus  ($v_req_id, "F");  # Set status to finished

       # Based on result, set appropriate INFO message.
       if ($v_result == 1) {
          $v_cutdown_msg = "Cutdown initiated";
       } elsif ($v_result == 0) {
          $v_cutdown_msg = "Cutdown failed.";
       } else {
          $v_cutdown_msg = "Cutdown failed. Code: " . $v_result;
       }

       set_launch_console_attribute("K", $v_result, $v_cutdown_msg);
    }  else {
       print "** Unknown request " . $v_request_code . "\n" if $DEBUG;
       setRequestStatus  ($v_req_id, "F");  # Set status to finished
    }


 }

 # Indicate to calling routine if a request was processed.
 if (defined $v_req_id && $v_req_id > 0) {
   return 1;
 } else {
   return 0;
 }

}



# Send request to RLS and wait for some response (an acknowledgement)
sub sendModemRequest($$$)
{
 local ($p_request_string,$p_response_string, $p_request_id) = @_;
 $v_result = "";

 $str = "Sending request string $p_request_string to RLS  (Request ID: $p_request_id)\n";
 log_message($str);
 print "** " . $str if $DEBUG;
 $count_out = $port->write($p_request_string . "\r\n");

 # If no response string requested...then we don't want to wait for other end
 # to exit.
 if ($p_response_string eq "") {
    return;
 }

 my $gotit = "";
 $ismatch = 0;
 my $start_time = time;
 my $timeout = 0;
 until (("" ne $gotit && $ismatch != 0) || $timeout == 1) {
    $gotit = $port->lookfor;       # poll until data ready

    # Make sure what we got resembles what we are after.
    if ($gotit =~ qr/^(${p_response_string})(.*)/) {
       $ismatch = 1;
    } else {
       print "DAMN. NOT A MATCH - " . $gotit . "\n";
    }
 
    # Allow a MAX of 2 seconds to get what we expect.
    if (time > $start_time + 2) {
       $timeout = 1;
       print "TIMED OUT!!!!!!!!!!!!!!!!!11\n";
    }
    select(undef,undef,undef,0.1);
    die "Aborted without match\n" unless (defined $gotit);
 }
 if ($gotit =~ qr/^(${p_response_string})(.*)/) {
    $data_component = $2;
    if ($data_component !~ /^$/) {
       $data_component =~ /(:)(.*)/;
       $data = $2;
          if ($data !~ /^$/) {
             $v_result = $data;
          } else {
             $v_result = 1;  
          }
    } else {
       $v_result = 1;
    }
    $str = "RLS received request (" . $p_response_string . ") and actioning. Responded with " . $gotit . "\n";
    print "** " . $str if $DEBUG;
    print "**    Data: " . $data . "\n" if $DEBUG && $data;
    updateRequestDetails ($p_request_id, $str);
    log_message($str);
 } elsif ($gotit =~ /W/) {
    $str = "(while sending $p_request_string) - Timeout waiting for response from ground station.\n";
    updateRequestDetails ($p_request_id, $str);
    log_message($str);
    print "** " . $str if $DEBUG;
    $v_result = 0;
 } elsif ($timeout == 1) {
    $str = "(while sending $p_request_string) - Timeout waiting for response from ground station. Waited ages.\n";
    updateRequestDetails ($p_request_id, $str);
    log_message($str);
    print "** " . $str if $DEBUG;
    $v_result = 0;
 } elsif ($gotit =~ /^Q:(.*)$/) {
    $str = "(while sending $p_request_string) - Did not recognise response from station. Response was: " . $1 . "\n";
    updateRequestDetails ($p_request_id, $str);
    log_message($str);
    print "** " . $str if $DEBUG;
    $v_result = 0;
 } else {
    $str = "RLS never responded as expected....perhaps it didnt get request $p_request_string. Got $gotit \n";
    updateRequestDetails ($p_request_id, $str);
    log_message($str);
    print "** " . $str if $DEBUG;
    $v_result = 0;
 }

 
 return $v_result;
}




# Update request with details of what happened with transmission of request
sub updateRequestDetails($$)
{
 local ($p_request_id, $p_notes) = @_;

 # No user request associated with command sent to modem. 
 if ($p_request_id == 0) {
   return;
 }

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "UPDATE requests_t
                  SET notes = '" . $p_notes . "'
           WHERE  id = $p_request_id";


 $sth = $dbh->prepare($query);
 $sth->execute();
 $sth->finish();

 # $dbh->disconnect();

}


# Set Request Status
sub setRequestStatus($$)
{
 local ($p_request_id, $p_status_code) = @_;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "UPDATE requests_t
                  SET status_code = '" . $p_status_code . "'
           WHERE  id = $p_request_id";


 $sth = $dbh->prepare($query);
 $sth->execute();
 $sth->finish();

 # $dbh->disconnect();

}



# Set latest launch status attribute
sub set_launch_console_attribute($$;$)
{
 local ($p_attribute, $p_status, $p_notes) = @_;

    # Initialise DB connection
    # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

    # Put in DB
    if (defined $p_notes) {
       $query = "INSERT INTO launch_system_status_t (attribute, status, notes) values (?,?,?)"; 
       $sth = $dbh->prepare($query);
       $sth->execute($p_attribute, $p_status, $p_notes);
    } else {
       $query = "INSERT INTO launch_system_status_t (attribute, status) values (?,?)"; 
       $sth = $dbh->prepare($query);
       $sth->execute($p_attribute, $p_status);
    }

    $sth->finish();

#    $dbh->disconnect();
}


# Set Power status
sub set_lc_power_status($)
{
 local ($p_status) = @_;

 if ($p_status == 2) {
    $v_notes = "Unknown error";
 } elsif ($p_status == 1) {
    $v_notes = "On";
 } elsif ($p_status == 0) {
    $v_notes = "Off";
 }

 set_launch_console_attribute("P", $p_status, $v_notes);

}


# Set Profile Setting
sub set_lc_profile_status($)
{
 local ($p_status) = @_;
 if ($p_status == 2) {
    $v_notes = "IMU";
 } elsif ($p_status == 1) {
    $v_notes = "Standard";
 } else {
    $v_notes = "Unknown";
 }
 set_launch_console_attribute("I", $p_status, $v_notes);
}


# Set Photo Setting
sub set_lc_photos_enabled_status($)
{
 local ($p_status) = @_;
 if ($p_status == 1) {
    $v_notes = "Photos Enabled";
 } elsif ($p_status == 0) {
    $v_notes = "No Photos";
 } else {
    $v_notes = "Unknown";
 }
 set_launch_console_attribute("N", $p_status, $v_notes);
}




# Set Arm status
sub set_lc_arm_status($)
{
 local ($p_status) = @_;

 if ($p_status == 9) {
    $v_notes = "Unknown error";
 } elsif ($p_status == 3) {
    $v_notes = "Dis-Armed (Continuity failed)";
 } elsif ($p_status == 2) {
    $v_notes = "Dis-Armed (Power not on)";
 } elsif ($p_status == 1) {
    $v_notes = "Armed";
 } elsif ($p_status == 0) {
    $v_notes = "Dis-armed";
 }

 set_launch_console_attribute("A", $p_status, $v_notes);

}


# Get last status for particular attribute of the launch console
sub get_last_status($;$)
{
 local ($p_attribute, $p_exclude_pending) = @_;

 $p_exclude_pending //= 0;

 $v_status = 0; # Default status

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 if ($p_exclude_pending == 1) {
    $query = "SELECT status
              FROM   launch_system_status_t
              WHERE  id = (select max(id) 
                           FROM launch_system_status_t
                           WHERE attribute = ?
                           AND    status >= 0)";
 } else {
    $query = "SELECT status
              FROM   launch_system_status_t
              WHERE  id = (select max(id) 
                           FROM launch_system_status_t
                           WHERE attribute = ?)";
 }

 $sth = $dbh->prepare($query);
 $sth->execute($p_attribute);

 ($v_status) = $sth->fetchrow_array();
 $sth->finish();

 # $dbh->disconnect();

 return $v_status;

}



# Return 1 if photo downloads enabled, 0 if photo downloads disabled
sub is_photo_downloads_enabled()
{

 $v_result = get_last_status('N', 1);

 return $v_result;

}


# indicates current cutdown status
# - 0 - cutdown failed
# - 1 - cutdown initiated
# - 3 - No cutdown request made yet (original state)
sub is_cutdown_request_made()
{

 $v_result = get_last_status('K', 1);

 return $v_result;

}


# Insert IMU figures
sub insert_imu($$)
{
 local($p_instance, $imu_hash) = @_;

 %imu = %$imu_hash;

 # Initialise DB connection
 # my $dbh = DBI->connect($pg_db_string,"","",{AutoCommit => 1, RaiseError => 1},) or die $DBI::errstr;

 # Put in DB
 $query = "INSERT INTO imu_t (instance_id,roll,pitch,yaw,gyrox,gyroy,gyroz,accx,accy,accz,timer)
                   values (?,?,?,?,?,?,?,?,?,?,?)";

 $sth = $dbh->prepare($query);
 $sth->execute($p_instance, $imu{'roll'}, $imu{'pitch'}, $imu{'yaw'}, $imu{'gyrox'}, $imu{'gyroy'}, $imu{'gyroz'},
               $imu{'accx'}, $imu{'accy'}, $imu{'accz'}, $imu{'timer'});

 $sth->finish();
 # $dbh->disconnect();
}


sub initialise_db()
{
 # Initialise DB connection
 my $dbh = DBI->connect($pg_db_string,"","",
                     {AutoCommit => 1, RaiseError => 0,
                      PrintError => 1},) 
                     or die $DBI::errstr;


 return $dbh;
}

