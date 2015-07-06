<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest measurements
try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


# Get latest launch systms status
$power_status              = get_rls_status("P");
$rls_power_status          = $power_status["status"];
$rls_power_status_date_raw = $power_status["creation_date"];
$rls_power_status_date     =  date("Y-m-d H:i:s", strtotime($rls_power_status_date_raw));

$arm_status              = get_rls_status("A");
$rls_arm_status          = $arm_status["status"];
$rls_arm_status_date_raw = $arm_status["creation_date"];
$rls_arm_status_date     =  date("Y-m-d H:i:s", strtotime($rls_arm_status_date_raw));




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
                        $("#msgText").html(s);
			showMsg();
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
                        $("#msgText").html(s);
			showMsg();
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
                        $("#msgText").html(s);
			showMsg();
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
                        $("#msgText").html(s);
			showMsg();
                }
            });
        });


</script>
<h3>Launch Console</h3>
<div id="list_wrapper">
<ul class="multiple_columns">
<?
// Power
if (is_null($rls_power_status)) {
  $v_power_status = "NA";
  $v_power_status_css = "un";
} else if ($rls_power_status == 1) {
  $v_power_status = "Power On";
  $v_power_status_css = "on";
} else if ($rls_power_status == 0) {
  $v_power_status = "Power Off";
  $v_power_status_css = "off";
}

// Arm
if (is_null($rls_arm_status)) {
  $v_arm_status = "NA";
  $v_arm_status_css = "un";
} else if ($rls_arm_status == 1) {
  $v_arm_status = "Armed";
  $v_arm_status_css = "on";
} else if ($rls_arm_status == 0) {
  $v_arm_status = "DIS-Armed";
  $v_arm_status_css = "off";
}

?>
    <li><input type="button" id="powertoggle" class="styled-button-<?= $v_power_status_css?>" value="Power-On" />
    <?= $v_power_status?></li>
    <li><input type="button" id="arm" class="styled-button-<?= $v_arm_status_css?>" value="Arm" />
    <?= $v_arm_status?></li>
    <li><input type="button" id="continuitytest" class="styled-button-on" value="Continuinity test" />
    </li>
    <li><input type="button" id="launch" class="styled-button-off" value="Launch" />
    </li>
  </ul>
</div>
