<?


function insert_request($p_request_code, $p_source, $p_destination, $p_ip)
{
 global $db_file;
 global $REQUEST_ENQUEUE_STATUS;

 $v_status = $REQUEST_ENQUEUE_STATUS;

 # Initialise DB connection
 try {
      $dbh = new PDO("sqlite:" . $db_file);
     }
 catch (PDOException $e)
     {
      echo $e->getMessage();
     }

 $sql = "INSERT INTO requests_t (source, destination, request_code, ip, status, creation_date)
         VALUES ('" . $p_source . "', '" . $p_destination . "', '" . $request_code . "',
                 '" . $p_ip . "', '" . $v_status . "',  datetime('now', 'localtime'))";

 # print $sql . "<br><br>\n";

 if (! $dbh->exec($sql)) {
    print "Error when executing statement to insert request\n";
    return;
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
         SET    status = '" . $p_status_code . "',
                last_update_date = datetime('now', 'localtime')
         WHERE  id = $p_request_id";

  if (! $dbh->exec($sql)) {
    print "Error when executing statement to update request status\n";
 }

}



function get_arm_status()
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


 $sql = "select arm_status
         from   launch_system_status_t
         WHERE  id = (select max(id) FROM launch_system_status_t)";

 $sth = $dbh->prepare($sql);
 $sth->execute();

 $row = $sth->fetch();
 $arm_status = $row['arm_status'];

 return $arm_status;
}



function get_power_status()
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


 $sql = "select power_status
         from   launch_system_status_t
         WHERE  id = (select max(id) FROM launch_system_status_t)";

 $sth = $dbh->prepare($sql);
 $sth->execute();

 $row = $sth->fetch();
 $power_status = $row['power_status'];

 return $power_status;
}


