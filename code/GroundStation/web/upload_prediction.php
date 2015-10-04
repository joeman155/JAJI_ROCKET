<b>Upload Balloon Path Prediction file</b><br/>
(CSV File from http://habhub.org/predict)<br /><br />

<?
include "config.inc";

# Get all the latest measurements
try {
     $dbh = new PDO("pgsql:user=www-data dbname=rls");
     $dbh->setAttribute( PDO::ATTR_ERRMODE, PDO::ERRMODE_WARNING );
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


$allowedExts = array("csv","txt");
$temp = explode(".", $_FILES["file"]["name"]);
$extension = end($temp);


if ($_SERVER['REQUEST_METHOD'] === 'POST') {
   if (
       (
        ($_FILES["file"]["type"] == "text/csv") || ($_FILES["file"]["type"] == "text/plain")
       )
       && 
       ($_FILES["file"]["size"] < 20000)
       && 
       in_array($extension, $allowedExts)) {
       if ($_FILES["file"]["error"] > 0) {
           echo "Error: " . $_FILES["file"]["error"] . "<br>";
       } else {
	   $v_file = $home_dir . "/uploads/" . $_FILES["file"]["name"];
           if (move_uploaded_file($_FILES["file"]["tmp_name"], $v_file)) {
		// Remove any previous prediction points
		remove_predictions();
		echo "Removed previous prediction points...<br />\n";

		echo "Starting upload of points...<br />\n";
		import_prediction($v_file);
		echo "Finished upload of points...<br />\n";

		print "<a href=\"/\">Return</a>\n<br />\n";

           } else {
              echo "Error moving file into position.";
           }
       }
   } else {
     echo "Invalid file";
     echo "<br>";
     echo $_FILES["file"]["type"];
     echo "<br>";
   }
} else {
?>

<form action="upload_prediction.php" method="post"
enctype="multipart/form-data">
<label for="file">Filename:</label>
<input type="file" name="file" id="file"><br>
<input type="submit" name="submit" value="Submit">
</form>
<?

}


function remove_predictions()
{
  global $dbh;

  $sql = "DELETE FROM gps_prediction_t";
  $rows_deleted = $dbh->exec($sql) . "<br>";
  print "# of rows deleted: " . $rows_deleted . "<br>\n";
}


function import_prediction($p_file)
{
  global $dbh;

  $rows = 0;
  if (($handle = fopen($p_file, "r")) !== FALSE) {
    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
        $num = count($data);
		$rows = $rows + 1;
		$sql = "INSERT INTO gps_prediction_t (dtime, latitude, longitude, height) VALUES ";
		$sql .= "(" . $data[0] . ", " . $data[1] . ", " . $data[2] . ", " . $data[3] . ")";
		# print $sql . "<br><br>\n";

		if (! $dbh->exec($sql)) {
			print "Error when executing statement\n";
                } 
    }
    fclose($handle);
  }


print "# of rows imported: " . $rows . "<br />\n";


}
