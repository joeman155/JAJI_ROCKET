<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest measurements
try {
     $dbh = new PDO("pgsql:user=www-data dbname=rls");
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


$request_code    = $_REQUEST['request'];
$exclude_pending = $_REQUEST['exclude_pending'];


if (! in_array($request_code, array("P","A","C","L","N","K", "X", "I"))) {
  $v_msg = "Unknown Request - Code: $v_request_code;";
  echo $v_msg;
  exit;
}


# Get latest launch systms status
$request_status = get_rls_status($request_code, $exclude_pending);


echo $request_status["status"];
