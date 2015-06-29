<?
include "config.inc";

try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }

$id = isset($_GET['id']) ? $_GET['id'] : 0;

if ($id == 0)
{
  $sql = "select * from messages_t where id > (select max(id) -20  from messages_t) ORDER BY id ASC";
}
else
{
  $sql = "select * from messages_t where id > " . $id . " ORDER BY id ASC";
}

$rows = $dbh->query($sql);

$result = array();

foreach ($rows as $row)
{

$v['id'] = $row['id'];
$v['message'] = $row['message'];
$v['creation_date'] = $row['creation_date'];

array_push($result, $v);

}


$json = json_encode($result);
print $json;

$dbh = null;
