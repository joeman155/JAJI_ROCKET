<?


function insert_request($p_request_code)
{
 global $request_source, $request_destination;

 $v_ip = $_SERVER['REMOTE_ADDR'];

 $v_req_id = insert_request_internal($p_request_code, $request_source, $request_destination, $v_ip);

 return $v_req_id;
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



function get_rls_status($p_attribute)
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


 $sql = "select *
         from   launch_system_status_t
         WHERE  id = (select max(id) FROM launch_system_status_t where attribute = '" . $p_attribute . "')";

 $sth = $dbh->prepare($sql);
 $sth->execute();

 $row = $sth->fetch();
 $result["status"] = $row['status'];
 $result["notes"]  = $row['notes'];
 $result["creation_date"] = $row['creation_date'];

$n = $row['notes'];
`echo $n > /tmp/whatthe`;
 return $result;
}



