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
        $.ajax({
                url: "launchConsoleRequest.php",
		data: {
                       request: "P"
                      },
                success: function(s,x) {
                        $("#msglc").html(s);
                }
            });
        });


        $("#arm").click(function() {
        $.ajax({
                url: "launchConsoleRequest.php",
                data: {  
                       request: "A"
                      },
                success: function(s,x) {
                        $("#msglc").html(s);
                }
            });
        });

        $("#continuitytest").click(function() {
        $.ajax({
                url: "launchConsoleRequest.php",
                data: {  
                       request: "C"
                      },
                success: function(s,x) {
                        $("#msglc").html(s);
                }
            });
        });

        $("#launch").click(function() {
        alert("Proceed to Launch?");
        $.ajax({
                url: "launchConsoleRequest.php",
                data: {  
                       request: "L"
                      },
                success: function(s,x) {
                        $("#msglc").html(s);
                }
            });
        });


</script>
<h3>Launch Console</h3>
<div>
<ul>
<?
// Power
if (is_null($rls_power_status)) {
  $v_power_status = "NA";
} else if ($rls_power_status == 1) {
  $v_power_status = "Power On";
} else if ($rls_power_status == 0) {
  $v_power_status = "Power Off";
}

// Arm
if (is_null($rls_arm_status)) {
  $v_arm_status = "NA";
} else if ($rls_arm_status == 1) {
  $v_arm_status = "Armed";
} else if ($rls_arm_status == 0) {
  $v_arm_status = "DIS-Armed";
}

?>
  <li><p id="powertoggle">Power-On</p>
  <?= $v_power_status?></li>
  <li><p id="arm">Arm</p>
  <?= $v_arm_status?></li>
  <li><p id="continuitytest">Continuinity test</p></li>
  <li><p id="launch">Launch</p></li>
</ul>
</div>

