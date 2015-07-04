<?

# CONFIGURATION
include "config.inc";

# Get all the latest measurements
try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


$lcRequest = $_REQUEST['request'];

if ($lcRequest == "P") {
  $v_msg = "Requesting Power on/off...";
} else if ($lcRequest == "A") {
  $v_msg = "Requesting Arm on/off...";
} else if ($lcRequest == "C") {
  $v_msg = "Requesting Continuity Test...";
} else if ($lcRequest == "L") {
  $v_msg = "Requesting Launch...";
} else {
  $v_msg = "Unknown Request '" . $lcRequest . "'";
}
 


echo $v_msg;



