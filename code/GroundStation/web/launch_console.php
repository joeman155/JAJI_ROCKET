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


# Get latest launch systms status
$sql = "select * from launch_system_status_t where id = (select max(id) from launch_system_status_t)";
$sth = $dbh->prepare($sql);
$sth->execute();

$row = $sth->fetch();
$rls_power_status = $row['power_status'];
$rls_arm_status   = $row['arm_status'];
$rls_date_raw     = $row['creation_date'];
$rls_date         =  date("Y-m-d H:i:s", strtotime($rls_date_raw));




?>

<script>
// AJAX stuff for buttons
        $("#powertoggle").click(function() {
        alert("About to toggle power");
        $.ajax({
                url: "request.php",
		data: {
                       request: "P"
                      },
                success: function(s,x) {
                        $("#hab_contol").html(s);
                }
            });
        });


        $("#arm").click(function() {
        alert("About to toggle arm");
        $.ajax({
                url: "request.php",
                data: {  
                       request: "A"
                      },
                success: function(s,x) {
                        $("#hab_contol").html(s);
                }
            });
        });

        $("#continuitytest").click(function() {
        alert("About to perform continuity Test");
        $.ajax({
                url: "request.php",
                data: {  
                       request: "C"
                      },
                success: function(s,x) {
                        $("#hab_contol").html(s);
                }
            });
        });

        $("#launch").click(function() {
        alert("About to Launch");
        $.ajax({
                url: "request.php",
                data: {  
                       request: "L"
                      },
                success: function(s,x) {
                        $("#hab_contol").html(s);
                }
            });
        });


</script>
<h3>Launch Console</h3>
<div>
<ul>
  <li><p id="powertoggle">Power-On</p>
  <?= $rls_power_status?></li>
  <li><p id="arm">Arm</p>
  <?= $rls_arm_status?></li>
  <li><p id="continuitytest">Continuinity test</p></li>
  <li><p id="launch">Launch</p></li>
</ul>
</div>

