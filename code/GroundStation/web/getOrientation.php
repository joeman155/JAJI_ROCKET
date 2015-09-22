<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest IMU angles
try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


 $sql = "SELECT *
         FROM   imu_t 
         WHERE  id = (SELECT max(id)
                      FROM   imu_t)";


 $sth = $dbh->prepare($sql);
 $sth->execute();

 $row = $sth->fetch();

 // Json 
 $json = json_encode($row);
 print $json;

$dbh = null;

