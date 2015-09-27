<?
include "config.inc";

# Get all the latest measurements
try {
     $dbh = new PDO("pgsql:user=www-data dbname=rls");
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }

   # GPS
   $sql = "select * from gps_t where id = (select max(id) from gps_t)";

   $sth = $dbh->prepare($sql);

   $sth->execute();

   $row = $sth->fetch();

   $latitude = $row['latitude'];
   $longitude = $row['longitude'];
   $height = $row['height'];
   $gps_date = $row['gps_date'];
   $gps_time = $row['gps_time'];
   $gps_creation_date = date("d-M-Y H:i:s", $row['creation_date']);


$v['latitude'] = $latitude;
$v['longitude'] = $longitude;
$v['height'] = $height;
$v['gps_date'] = $gps_date;
$v['gps_time'] = $gps_time;
$v['gps_creation_date'] = $gps_creation_date;

$json = json_encode($v);
print $json;

$dbh = null;


?>
