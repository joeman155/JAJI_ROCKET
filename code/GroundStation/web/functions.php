<?


function insert_request($p_request_code)
{
 global $request_source, $request_destination;

 $v_ip = $_SERVER['REMOTE_ADDR'];

 $v_req_id = insert_request_internal($p_request_code, $request_source, $request_destination, $v_ip);

 return $v_req_id;
}


// Given reqeuest code, get the request name
function get_request_name($p_request_code)
{
 global $db_file;

 # Initialise DB connection
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
      return NULL;
     }

 $sql = "SELECT request_name
         FROM request_types_t
         WHERE request_code = ?";

 $sth = $dbh->prepare($sql);
 $sth->execute(array($p_request_code));

 $row = $sth->fetch();
 $v_request_name = $row['request_name'];

 return $v_request_name;
}



function insert_request_internal($p_request_code, $p_source, $p_destination, $p_ip)
{
 global $db_file;
 global $REQUEST_ENQUEUE_STATUS;

 $v_status_code = $REQUEST_ENQUEUE_STATUS;

 # Initialise DB connection
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
      return NULL;
     }

 $sql = "INSERT INTO requests_t (source, destination, request_code, ip, status_code, creation_date)
         VALUES ('" . $p_source . "', '" . $p_destination . "', '" . $p_request_code . "',
                 '" . $p_ip . "', '" . $v_status_code . "',  datetime('now', 'localtime'))";

 # print $sql . "<br><br>\n";
file_put_contents("/tmp/sql.txt", $sql);

 if (! $dbh->exec($sql)) {
    print "Error when executing statement to insert request\n";
    return NULL;
 }

 $v_id = $dbh->lastInsertId();

 return $v_id;
}



function set_request_status($p_request_id, $p_status_code)
{
 global $db_file;

 # Initialise DB connection
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
     }


 $sql = "UPDATE requests_t 
         SET    status_code = '" . $p_status_code . "',
                last_update_date = datetime('now', 'localtime')
         WHERE  id = $p_request_id";

  if (! $dbh->exec($sql)) {
    print "Error when executing statement to update request status\n";
 }

}



# Get the last known status of particular state of rocket launch system
#
# Pending is a special state...it is a quasi state between changes.
# Because we can't ALWAYS rely upon the radio transmission...there is some
# level of uncertainty, and we wish to reflect that here.
function get_rls_status($p_attribute, $p_exclude_pending = 0)
{
 global $db_file;
 $result = array();

 # Initialise DB connection
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
     }


 # If p_exclude_pending == 0, this means we don't care what sttus
 if ($p_exclude_pending == 0) {
    $sql = "SELECT *
            FROM   launch_system_status_t
            WHERE  id = (SELECT MAX(id) 
                         FROM   launch_system_status_t 
                         WHERE  attribute = ?)";

 } else {
 # Else we DO wish to skip the pending status
    $sql = "SELECT *
            FROM   launch_system_status_t
            WHERE  id = (SELECT MAX(id) 
                         FROM   launch_system_status_t 
                         WHERE  attribute = ?
                         AND    status >= 0)";


 }

 $sth = $dbh->prepare($sql);
 $sth->execute($p_attribute);

 $row = $sth->fetch();
 $result["status"] = $row['status'];
 $result["notes"]  = $row['notes'];
 $result["creation_date"] = $row['creation_date'];

 return $result;
}



