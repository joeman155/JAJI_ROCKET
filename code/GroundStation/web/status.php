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
     $dbh = new PDO("pgsql:user=www-data dbname=rls");
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


# Get latest RLS Radio Stats
$sql = "select * from radio_stats_t where id = (select max(id) from radio_stats_t where place = 1)";
$sth = $dbh->prepare($sql);
$sth->execute();
$row = $sth->fetch();
$radio_stats_hab = $row['stats'];
$radio_stats_hab_date =  date("Y-m-d H:i:s", strtotime($row['creation_date']));


# Get latest RLS heartbeat
$sql = "select * from heartbeat_t where id = (select max(id) from heartbeat_t)";
$sth = $dbh->prepare($sql);
$sth->execute();
$row = $sth->fetch();
$heartbeat = $row['heartbeat'];
$heartbeat_date_raw = $row['creation_date'];
$heartbeat_date =  date("Y-m-d H:i:s", strtotime($heartbeat_date_raw));


# Get latest groundstation voltage
$sql = "select * from gs_psu_voltage_t where id = (select max(id) from gs_psu_voltage_t)";
$sth = $dbh->prepare($sql);
$sth->execute();
$row = $sth->fetch();
$gs_psu_voltage = $row['voltage'];
$gs_psu_voltage_date = date("Y-m-d H:i:s", strtotime($row['creation_date']));


# RLS GPS
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


# Get Measurements Group D00 (pressure, temps, voltages)
$measurements_group_d00 = getMeasurements("D00");
$internal_temp = isset($measurements_group_d00['measurements']['Internal Temperature']) ? $measurements_group_d00['measurements']['Internal Temperature'] : NULL;
$ign_voltage = isset($measurements_group_d00['measurements']['IGN Voltage']) ? $measurements_group_d00['measurements']['IGN Voltage'] : NULL;
$cpu_voltage = isset($measurements_group_d00['measurements']['CPU Voltage']) ? $measurements_group_d00['measurements']['CPU Voltage'] : NULL;

$v_now = date("Y-m-d H:i:s");



# Calculate distance between LOCAL and RLS GPS 
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



# ALERTS
$alerts = array();
$alerts['alerts'] = array();
$alert_css = "style=\"color: red;\"";

# Temperature
if ($internal_temp < 273 + $threshold_temperature_low) {
	$alert_temperature = "Temperature Alert - Below " . $threshold_temperature_low;
	$alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_temperature, 'title' => "RLS Temperature"));
} else if ($internal_temp > (273 + $threshold_temperature_high)) {
	$alert_temperature = "Temperature Alert - Above " . $threshold_temperature_high;
	$alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_temperature, 'title' => "RLS Temperature"));
}

# CPU Voltages
if ($cpu_voltage < $threshold_cpu_voltage) {
	$alert_voltage = "Voltage below safe minimum of " . $threshold_cpu_voltage;
	$alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_voltage, 'title' => "CPU Voltage"));
}

# IGN Voltages
if ($ign_voltage < $threshold_ign_voltage) {
	$alert_voltage = "Voltage below safe minimum of " . $threshold_ign_voltage;
	$alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_voltage, 'title' => "IGN Voltage"));
}


# GS Voltage
if ($gs_psu_voltage < $threshold_gs_voltage) {
        $alert_voltage = "Voltage below safe minimum of " . $threshold_gs_voltage;
        $alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_voltage, 'title' => "GS Voltage"));
}


# Satellites
if ($satellites < $threshold_satellites) {
	$alert_satellites = "Number of satellites less that " . $threshold_satellites;
	$alerts['css'] =  $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_satellites, 'title' => "Satellites"));
}

# Altitude
if ($height > $threshold_altitude) {
	$alert_altitude = "Exceeded " . $threshold_altitude . "m!!";
        $alerts['css']  = $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_altitude, 'title'  => "Altitude"));
}

# radio loss of contact
if (time() - strtotime($heartbeat_date_raw) > $threshold_heartbeat) {
	$alert_loss_heartbeat = "No heartbeat for more than " . $threshold_heartbeat . " seconds!!";
        $alerts['css']  = $alert_css;
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'], array('text'  => $alert_loss_heartbeat, 
                                            'title' => "No heartbeat for more than " . $threshold_heartbeat . " seconds?"));
}


# Distance exceeds 30km
if ($v_los_distance > $threshold_distance) {
	$alert_distance = "Exceeded distance of " . $threshold_distance . "km!!";
        $alerts['css']  = $alert_css;  
        $alerts['creation_date'] = $v_now;
        array_push($alerts['alerts'] = array('text'  => $alert_distance,
        			             'title' => "Distance exceeds " . $threshold_distance . "km?"));
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
<h2>RLS GPS Information (<?= $gps_creation_date?>)</h2>
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

<? 
if (count($alerts['alerts']) > 0 ) {
  $v_alert_creation_date = $alerts['creation_date'];
  $v_alert_css           = $alerts['css'];
?>
<h3 <?= $v_alert_css?>>Alerts - <abbr class="timeago" title="<?= $v_alert_creation_date?>"></abbr></h3>
<div>
<h2>RLS Alerts</h2>
<table>

<?
  foreach ($alerts['alerts'] as $key => $val) {
?>
    <tr>
      <th><?= $val['title']?></th>
      <td><?= $val['text']?></td>
    </tr>
<?
  }
?>
  </table>
  </div>
<?
} else {
?>
  <h3>Alerts - None present </h3>
  <div>
    None
  </div>
<?
}
?>

<h3>RLS Measurements - <abbr class="timeago" title="<?= $measurements_group_d00['date_time']?>"></abbr></h3>
<div>
<h2>Latest Measurements (<?= $measurements_group_d00['date_time']?>)</h2>
<table id="measurements">
<?
foreach ($measurements_group_d00['measurements'] as $key => $val) {
?>
<tr>
  <th><?= $key?></th>
  <td><?= $val?></td>
</tr>
<?
}
?>
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
<h2>RLS Radio Stats (<?= $radio_stats_hab_date?>)</h2>
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


// Get measurements (for a particular group)
// Returns array of results as a hash
function getMeasurements($p_group_name) {
   global $dbh;

   # Get latest id of group
   $sql = "select id, creation_date
           from measurement_group_t
           where id = (select max(id)
                       from measurement_group_t
                       where group_name = ?)";

   $sth = $dbh->prepare($sql);
   $sth->execute(array($p_group_name));
   $row = $sth->fetch();

   $v_group_id = $row['id'];
   $creation_date = date("Y-m-d H:i:s", strtotime($row['creation_date']));

   # Initialise array to hold all measurements
   $measurement = array();

   # Now get all the measurements
   $sql = "select name, value
           from   measurement_t
           where  group_id = ?";

   $sth = $dbh->prepare($sql);
   $sth->execute(array($v_group_id));

   # Cycle trhough all rows
   while ($row = $sth->fetch(PDO::FETCH_ASSOC)) {
     $measurement[$row['name']] = $row['value'];
   }

   # Now put all the measurements together in a neat hash and return to calling
   # routine.
   $measurements = array();
   $measurements['date_time'] = $creation_date;
   $measurements['measurements'] = $measurement;

   return $measurements;
}


function getMeasurement($p_source, $p_name) {
    global $dbh;
    
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
   
