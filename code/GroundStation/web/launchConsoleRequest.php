<?

# CONFIGURATION
include "config.inc";
include "functions.php";

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
  # Get current status

  # Submit Request
  $v_req_id = insert_request($lcRequest);
  if (is_null($v_req_id)) {
    $v_msg = "Error occured while Requesting Power on/off...";
  } else {
    $v_msg = "Requesting Power on/off - Req ID: " . $v_req_id;
  }


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



