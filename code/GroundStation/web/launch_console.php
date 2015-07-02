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

?>

<script>
// AJAX stuff for buttons

</script>
<h3>Launch Console</h3>
<div>
<ul>
  <li>Power-On</li>
  <li>Arm</li>
  <li>Continuinity test</li>
  <li>Launch</li>
</ul>
</div>

