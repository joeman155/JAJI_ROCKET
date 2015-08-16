<?

# CONFIGUIRATION
include "config.inc";

# PARAMETERS (phone GPS)
$v_local_lat       = isset($_REQUEST['local_lat']) ? $_REQUEST['local_lat'] : NULL;
$v_local_long      = isset($_REQUEST['local_long']) ? $_REQUEST['local_long'] : NULL;
$v_local_alt       = isset($_REQUEST['local_alt']) ? $_REQUEST['local_alt'] : NULL;
$v_local_got_gps   = isset($_REQUEST['local_got_gps']) ? $_REQUEST['local_got_gps'] : NULL;
$v_local_timestamp = isset($_REQUEST['local_timestamp']) ? $_REQUEST['local_timestamp'] : NULL;


# Get all the latest measurements
try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }

# Get latest Ground Radio Stats
$sql = "select * from radio_stats_t where id = (select max(id) from radio_stats_t where place = 0)";
$sth = $dbh->prepare($sql);
$sth->execute();

$row = $sth->fetch();
$radio_stats_ground = $row['stats'];
$radio_stats_ground_date =  date("Y-m-d H:i:s", strtotime($row['creation_date']));


# Get latest HAB Radio Stats
$sql = "select * from radio_stats_t where id = (select max(id) from radio_stats_t where place = 1)";
$sth = $dbh->prepare($sql);
$sth->execute();

$row = $sth->fetch();
$radio_stats_hab = $row['stats'];
$radio_stats_hab_date =  date("Y-m-d H:i:s", strtotime($row['creation_date']));


# Get latest HAB heartbeat
$sql = "select * from heartbeat_t where id = (select max(id) from heartbeat_t)";
$sth = $dbh->prepare($sql);
$sth->execute();

$row = $sth->fetch();
$heartbeat = $row['heartbeat'];
$heartbeat_date_raw = $row['creation_date'];
$heartbeat_date =  date("Y-m-d H:i:s", strtotime($heartbeat_date_raw));

# Get latest beaglebone voltage
$sql = "select * from gs_psu_voltage_t where id = (select max(id) from gs_psu_voltage_t)";
$sth = $dbh->prepare($sql);
$sth->execute();

$row = $sth->fetch();
$gs_psu_voltage = $row['voltage'];
$gs_psu_voltage_date = date("Y-m-d H:i:s", strtotime($row['creation_date']));

# HAB GPS
$sql = "select * from gps_t where id = (select max(id) from gps_t)";
$sth = $dbh->prepare($sql);
$sth->execute();
$row = $sth->fetch();

$latitude  = $row['latitude'];
$longitude = $row['longitude'];
$height    = $row['height'];
$speed     = $row['speed'];
$course    = $row['course'];
$satellites = $row['satellites'];
$gps_date = $row['gps_date'];
$gps_time = $row['gps_time'];
$gps_creation_date = date("Y-m-d H:i:s", strtotime($row['creation_date']));


# Get the previous altitude
$prev_id = $row['id'] - 1;

$sql = "select height, gps_time from gps_t where id = " . $prev_id;
$sth = $dbh->prepare($sql);
$sth->execute();
$row = $sth->fetch();

$prev_height = $row['height'];
$prev_time   = $row['gps_time'];

# NOW!!.. we calculate the vertical velocity
if ($gps_time != null && $prev_time != null) {
  $tdiff = time2seconds($gps_time) - time2seconds($prev_time);
} else  {
  $tdiff = 0;
}

if ($tdiff <> 0) {
   $v_vertical_velocity = 60 * ($height - $prev_height)/$tdiff;
} else {
   $v_vertical_velocity = 0;
}

$v_vertical_velocity = round($v_vertical_velocity, 0);



# Calculate distance between LOCAL and HAB GPS 
if ($latitude != "" && $longitude != "" && $v_local_lat != "" && $v_local_long != "") {
  $v_horizontal_distance = calculateDistance($v_local_lat, $v_local_long, $latitude, $longitude, "K");
  $v_direction = calculateDirection($v_local_lat, $v_local_long, $latitude, $longitude);
  $v_los_distance = pow(pow($v_horizontal_distance, 2) + pow($height/1000 - $v_local_alt/1000, 2), 0.5);
  $v_direction = round($v_direction);
  $v_los_distance = round($v_los_distance,3);
  $v_horizontal_distance = round($v_horizontal_distance,3);
} else {
  $v_horizontal_distance = "Not enough info to calculate.";
  $v_direction = $v_horizontal_distance;
  $v_los_distance = $v_horizontal_distance;
}


# Pressure, internal temp, external temp
list ($cpu_voltage, $cv_date)    = getMeasurement("RLS", "CPU VOLTAGE");
list ($ign_voltage, $iv_date)    = getMeasurement("RLS", "IGN VOLTAGE");
list ($air_pressure, $ap_date)   = getMeasurement("RLS", "AIR PRESSURE");
list ($internal_temp, $it_date)  = getMeasurement("RLS", "INT TEMP");
list ($external_temp, $et_date)  = getMeasurement("RLS", "EXT TEMP");
list ($estimated_altitude, $ea_date)  = getMeasurement("RLS", "ESTIMATED ALT");



# ALERTS
# REset alert css
$alert_css = "";
# temperature
if ($internal_temp < 273 + $threshold_temperature_low) {
	$alert_temperature = "Temperature Alert - Below " . $threshold_temperature_low;
        $alert_css = "style=\"color: red;\"";
} else if ($internal_temp > (273 + $threshold_temperature_high)) {
	$alert_temperature = "Temperature Alert - Above " . $threshold_temperature_high;
        $alert_css = "style=\"color: red;\"";
} else {
	$alert_temperature = "None";
}


# Satellites
$alert_satellites = "None";
if ($satellites < $threshold_satellites) {
	$alert_satellites = "Number of satellites less that " . $threshold_satellites;
        $alert_css = "style=\"color: red;\"";
}

# altitude
$alert_altitude = "None";
if ($height > $threshold_altitude) {
	$alert_altitude = "Exceeded " . $threshold_altitude . "m!!";
        $alert_css = "style=\"color: red;\"";
}

# radio loss of contact
$alert_loss_heartbeat = "None";
if (time() - strtotime($heartbeat_date_raw) > $threshold_heartbeat) {
	$alert_loss_heartbeat = "No heartbeat for more than " . $threshold_heartbeat . " seconds!!";
        $alert_css = "style=\"color: red;\"";
}


# Distance exceeds 30km
$alert_distance = "None";
if ($v_los_distance > $threshold_distance) {
	$alert_distance = "Exceeded distance of " . $threshold_distance . "km!!";
        $alert_css = "style=\"color: red;\"";
}


?>
<script>
  $(function() {

            // Initialise fuzzy timeago
            $("abbr.timeago").timeago();

            var act = 0;
            $( "#accordion" ).accordion({
                create: function(event, ui) {
                    //get index in cookie on accordion create event
                    if($.cookie('saved_index') != null){
                       act =  parseInt($.cookie('saved_index'));
		    } else {
		       $.cookie('saved_index', 0);
		    }
                },
                activate: function(event, ui) {
                    //set cookie for current index on change event
		    var active = $( "#accordion" ).accordion( "option", "active" );
                    $.cookie('saved_index', null);
                    $.cookie('saved_index', active);
                },
		active:parseInt($.cookie('saved_index')),
		heightStyle: "content"
            });
  });
</script>
<div>
Heartbeat: <?= $heartbeat?> - <abbr class="timeago" title="<?= $heartbeat_date?>"></abbr>
</div>
<div id="accordion">
<h3>GPS Information - <abbr class="timeago" title="<?= $gps_creation_date?>"></abbr></h3>
<div>
<h2>HAB GPS Information (<?= $gps_creation_date?>)</h2>
<table id="gps" class="horizontal">
<tr>
  <th>Lat</th>
  <th>Long</th>
  <th>Alt</th>
</tr>
<tr>
  <td><?= $latitude?></td>
  <td><?= $longitude?></td>
  <td><?= $height?></td>
</tr>
<tr>
  <th>Satellites</th>
  <th>Date</th>
  <th>Time</th>
</tr>
<tr>
  <td><?= $satellites?></td>
  <td><?= $gps_date?></td>
  <td><?= $gps_time?></td>
</tr>
<tr>
  <th>Spd (km/hr)</th>
  <th>Crs (deg)</th>
</tr>
<tr>
  <td><?= $speed?></td>
  <td><?= $course?></td>
</tr>
</table>

<h2>Local GPS Information (<?= $v_local_timestamp?>)</h2>
<table id="local_gps" class="horizontal">
<tr>
  <th>Latitude</th>
  <th>Longitude</th>
  <th>Altitude</th>
</tr>
<tr>
  <td><?= $v_local_lat?></td>
  <td><?= $v_local_long?></td>
  <td><?= round($v_local_alt,0)?></td>
</tr>
</table>

<h2>GPS calculated values</h2>
<table id="relational" class="horizontal">
<tr>
  <th>Distance (Great circle) (km)</th>
  <th>Approx Distance (LOS) (km)</th>
  <th>Direction (degrees)</th>
  <th>Vertical velocity (m/min)</th>
</tr>
<tr>
  <td><?= $v_horizontal_distance?></td>
  <td><?= $v_los_distance?></td>
  <td><?= $v_direction?></td>
  <td><?= $v_vertical_velocity?></td>
</tr>
</table>
</div>

<h3 <?= $alert_css?>>Alerts - <abbr class="timeago" title="<?= $gps_creation_date?>"></abbr></h3>
<div>
<h2>HAB Alerts</h2>
<table>
<tr>
  <th>HAB Temperature</th>
  <td><?= $alert_temperature?></td>
</tr>
<tr>
  <th>Satellites</th>
  <td><?= $alert_satellites?></td>
</tr>
<tr>
  <th>Altitude - <?= $threshold_altitude?>m</th>
  <td><?= $alert_altitude?></td>
</tr>
<tr>
  <th>No heartbeat for more than <?= $threshold_heartbeat?> seconds</th>
  <td><?= $alert_loss_heartbeat?></td>
</tr>
<tr>
  <th>Distance exceeds <?= $threshold_distance?>km</th>
  <td><?= $alert_distance?></td>
</tr>
</table>

</div>

<h3>HAB Measurements - <abbr class="timeago" title="<?= $gps_creation_date?>"></abbr></h3>
<div>
<h2>Latest Measurements (<?= $cv_date?>)</h2>
<table id="measurements">
<tr>
  <th>CPU Voltage</th>
  <td><?= $cpu_voltage?></td>
</tr>
<tr>
  <th>IGN Voltage</th>
  <td><?= $ign_voltage?></td>
</tr>
<tr>
  <th>Air Pressure (Pa)</th>
  <td><?= $air_pressure?></td>
</tr>
<tr>
  <th>Internal Temp (K)</th>
  <td><?= $internal_temp?></td>
</tr>
<tr>
  <th>External Temp (K)</th>
  <td><?= $external_temp?></td>
</tr>
<tr>
  <th>Estimated Altitude (m)</th>
  <td><?= $estimated_altitude?></td>
</tr>
</table>
</div>


<h3>Ground Health - <abbr class="timeago" title="<?= $gs_psu_voltage_date?>"></abbr></h3>
<div>
<h2>Beaglebone Voltage (<?= $gs_psu_voltage_date?>)</h2>
<table id="gs_psu_measurements">
<tr>
  <th>Ground Station Voltage</th>
  <td><?= round($gs_psu_voltage,2)?></td>
</tr>
</table>
</div>


<h3>Radio Status - <abbr class="timeago" title="<?= $radio_stats_ground_date?>"></abbr></h3>
<div>
<h2>Ground Radio Stats (<?= $radio_stats_ground_date?>)</h2>
<table id="radio_stats_ground">
<tr>
  <th>Stats</th>
  <td><?= $radio_stats_ground?></td>
</tr>
</table>
<h2>HAB Radio Stats (<?= $radio_stats_hab_date?>)</h2>
<table id="radio_stats_hab">
<tr>
  <th>Stats</th>
  <td><?= $radio_stats_hab?></td>
</tr>
</table>
</div>

<?
# Get download date
if (file_exists($download_file_status)) {
  $download_date = date ("Y-m-d H:i:s", filemtime($download_file_status));
}
?>

<h3>Camera Image Downloads - <abbr class="timeago" title="<?= $download_date?>"></h3>
<div>
<h2>X-Modem Download Progress</h2>
<?
if (file_exists($download_file_status)) {
  $download_file_status = `cat $download_file_status`;
} else {
  $download_file_status = 0;
}

if ($download_file_status == 1) {
  print "Download in progress....<br>\n";
} else {
  print "NO download at present.<br>\n";
}

if ($download_file_status == 1) {
   if (file_exists($x_modem_packet_file)) {
     $packet_num = `cat $x_modem_packet_file`;
   ?>
     Packet: <?= $packet_num ?>
   <? } 
}
?> 
</div>


<h3>File Downloads</h3>
<div>
<a href="/groundStation.log">Log File</a>
<br />
<a href="/table.php?table=messages_t">Messsages Table</a>
<br />
<a href="/table.php?table=gs_psu_voltage_t">Ground Station PSU Voltages Table</a>
<br />
<a href="/table.php?table=gps_t">GPS Table</a>
<br />
<a href="/table.php?table=heartbeat_t">Heartbeat Table</a>
<br />
<a href="/table.php?table=measurement_t">Measurement Table</a>
<br />
<a href="/table.php?table=radio_stats_t">Radio Stats Table</a>
</div>

<?
function calculateDistance($lat1, $lon1, $lat2, $lon2, $unit) {
  $theta = $lon1 - $lon2;
  $dist = sin(deg2rad($lat1)) * sin(deg2rad($lat2)) +  cos(deg2rad($lat1)) * cos(deg2rad($lat2)) * cos(deg2rad($theta));
  $dist = acos($dist);
  $dist = rad2deg($dist);
  $miles = $dist * 60 * 1.1515;
  $unit = strtoupper($unit);

  if ($unit == "K") {
    return ($miles * 1.609344);
  } else if ($unit == "N") {
      return ($miles * 0.8684);
    } else {
        return $miles;
      }
}

function _deg2rad_multi() {
    // Grab all the arguments as an array & apply deg2rad to each element
    $arguments = func_get_args();
    return array_map('deg2rad', $arguments);
}



function calculateDirection($p_lat1, $p_lon1, $p_lat2, $p_lon2) {
    // Convert our degrees to radians:
    list($lat1, $lon1, $lat2, $lon2) =
        _deg2rad_multi($p_lat1, $p_lon1, $p_lat2, $p_lon2);

    // Run the formula and store the answer (in radians)
    $rads = atan2(
            sin($lon2 - $lon1) * cos($lat2),
            (cos($lat1) * sin($lat2)) -
                  (sin($lat1) * cos($lat2) * cos($lon2 - $lon1)) );

    // Convert this back to degrees to use with a compass
    $degrees = rad2deg($rads);

    // If negative subtract it from 360 to get the bearing we are used to.
    $degrees = ($degrees < 0) ? 360 + $degrees : $degrees;

    return $degrees;
}




function time2seconds($time='00:00:00')
{
    list($hours, $mins, $secs) = explode(':', $time);
    return ($hours * 3600 ) + ($mins * 60 ) + $secs;
}


function getMeasurement($p_source, $p_name) {
    
    $sql = "select * 
            from measurement_t 
            where id = (select max(id) from measurement_t 
                        where source = ?
                        and   name   = ?)";
    $sth = $dbh->prepare($sql);
    $sth->execute(array($p_source, $p_name));
    $row = $sth->fetch();

    $data = $row['value'];
    $data_date = date("Y-m-d H:i:s", strtotime($row['creation_date']));

    return array($data, $data_date);
}
