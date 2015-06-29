<?
include "config.inc";

# Get all the latest measurements
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
     }

   # GPS
   $now = date("Y-m-d");
   $sql = "select latitude, longitude, height, dtime, creation_date
           from gps_prediction_t
           order by id";

   $sth = $dbh->prepare($sql);

   $sth->execute();

   # $row = $sth->fetch();

   $results = array();
   while ($row = $sth->fetch(PDO::FETCH_ASSOC)) {
      $latitude = $row['latitude'];
      $longitude = $row['longitude'];
      $height = $row['height'];
      $gps_prediction_datetime = $row['dtime'];
      $gps_creation_date = $row['creation_date'];


      $v['latitude'] = $latitude;
      $v['longitude'] = $longitude;
      $v['height'] = $height;
      $v['dtime'] = $gps_prediction_datetime;
      $v['gps_creation_date'] = $gps_creation_date;

      array_push($results, $v);

   }
$json = json_encode($results);
print $json;

$dbh = null;


?>
