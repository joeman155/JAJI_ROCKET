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

   # Determine how many points there are. We don't want at most 30 points
   $now = date("Y-m-d");
   $sql = "select count(*) as cnt
           from gps_t
           where creation_date >= '" . $now . "'";

   $sth = $dbh->prepare($sql);
   $sth->execute();
   $row = $sth->fetch();
   $v_row_count = $row[0];


   # Determine spacing between points if we have more then v_max_row_count points
   $v_spacing = 1;
   if ($v_row_count > $v_max_row_count) {
     $v_spacing = round(($v_row_count - ($v_max_row_count - $v_num_spaced_out_points))/$v_num_spaced_out_points,0);
   }

   # GPS
   $sql = "select latitude, longitude, height, gps_date, gps_time, time(creation_date) as creation_date
           from gps_t 
           where creation_date >= '" . $now . "'
           order by id ASC";

   $sth = $dbh->prepare($sql);

   $sth->execute();

   $results = array();

   $iteration = 0;
   while ($row = $sth->fetch(PDO::FETCH_ASSOC)) {
     $iteration++;
      # Add points if we meet max points criteria
      if ($v_row_count > $v_max_row_count && $iteration > ($v_row_count - $v_max_row_count)) {
         $latitude = $row['latitude'];
         $longitude = $row['longitude'];
         $height = $row['height'];
         $gps_date = $row['gps_date'];
         $gps_time = $row['gps_time'];
         $gps_creation_date = $row['creation_date'];

         $v['latitude'] = $latitude;
         $v['longitude'] = $longitude;
         $v['height'] = $height;
         $v['gps_date'] = $gps_date;
         $v['gps_time'] = $gps_time;
         $v['gps_creation_date'] = $gps_creation_date;

         array_push($results, $v);
      } else  {
        if ($iteration % $v_spacing == 0) {
           $latitude = $row['latitude'];
           $longitude = $row['longitude'];
           $height = $row['height'];
           $gps_date = $row['gps_date'];
           $gps_time = $row['gps_time'];
           $gps_creation_date = $row['creation_date'];

           $v['latitude'] = $latitude;
           $v['longitude'] = $longitude;
           $v['height'] = $height;
           $v['gps_date'] = $gps_date;
           $v['gps_time'] = $gps_time;
           $v['gps_creation_date'] = $gps_creation_date;

           array_push($results, $v);

        }

      }
   }
$json = json_encode($results);
print $json;

$dbh = null;


?>
