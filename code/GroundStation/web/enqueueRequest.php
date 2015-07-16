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


$request_code = $_REQUEST['request'];

if (! in_array($request_code, array("P","A","C","L","N","K", "X"))) {
  $v_msg = "Unknown Request - Code: $v_request_code;";
  echo $v_msg;
  exit;
}
 



# Got this far, so it must be a valid request code

# Submit Request
$v_req_id = insert_request($request_code);
if (is_null($v_req_id)) {
   $v_msg = "Error occured while enqueing request...";
} else {
   $v_msg = "Requesting submitted - Req ID: " . $v_req_id;
}


echo $v_msg;



