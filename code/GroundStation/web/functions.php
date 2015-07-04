<?


function insert_request($p_request_code, $p_source, $p_destination, $p_ip)
{
 global $db_file;
 global $REQUEST_ENQUEUE_STATUS;

 $v_status = $REQUEST_ENQUEUE_STATUS;

 # Get all the latest measurements
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
 }



}

