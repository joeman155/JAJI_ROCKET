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
$rls_power_notes           = $power_status["notes"];
$rls_power_status_date_raw = $power_status["creation_date"];
$rls_power_status_date     =  date("Y-m-d H:i:s", strtotime($rls_power_status_date_raw));

$arm_status                = get_rls_status("A");
$rls_arm_status            = $arm_status["status"];
$rls_arm_notes             = $arm_status["notes"];
$rls_arm_status_date_raw   = $arm_status["creation_date"];
$rls_arm_status_date       =  date("Y-m-d H:i:s", strtotime($rls_arm_status_date_raw));


$ct_status                 = get_rls_status("C");
$rls_ct_status             = $ct_status["status"];
$rls_ct_notes              = $ct_status["notes"];
$rls_ct_status_date_raw    = $ct_status["creation_date"];
$rls_ct_status_date        =  date("Y-m-d H:i:s", strtotime($rls_ct_status_date_raw));


?>

<?= $rls_power_status_date_raw?>
<script>
	// Initialise fuzzy timeago
	$("abbr.timeago").timeago();


	// AJAX stuff for buttons
        $("#powertoggle").click(function() {
        if (reload_paused == 2) {
           return; // The page reload is running
        }

        $("#powertoggle").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        reload_paused = 1;
        $.ajax({
                url: "enqueueRequest.php",
                async: false,
                cache: false,
		data: {
                       request: "P"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                        v_current_status = getRlsStatus('P', 1);
			checkStatus('powertoggle', v_current_status, 'P');
                }
            });
        });


        $("#arm").click(function() {
        if (reload_paused == 2) {
           return; // The page reload is running
        }

        $("#arm").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        reload_paused = 1;
        $.ajax({
                url: "enqueueRequest.php",
                async: false,
                cache: false,
                data: {  
                       request: "A"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                        v_current_status = getRlsStatus('A', 1);
			checkStatus('arm', v_current_status, 'A');
                }
            });
        });

        $("#continuitytest").click(function() {
        if (reload_paused == 2) {
           return; // The page reload is running
        }

        $("#continuitytest").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        reload_paused = 1;
        $.ajax({
                url: "enqueueRequest.php",
                async: false,
                cache: false,
                data: {  
                       request: "C"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                        v_current_status = getRlsStatus('C', 1);
			checkStatus('continuitytest', v_current_status, 'C');
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
  $v_power_status = "Unknown";
  $v_power_status_css = "un";
} else if ($rls_power_status == 1) {
  $v_power_status = $rls_power_notes;
  $v_power_status_css = "on";
} else if ($rls_power_status == 0) {
  $v_power_status = $rls_power_notes;
  $v_power_status_css = "off";
} else if ($rls_power_status == -1) {
  $v_power_status = $rls_power_notes;
  $v_power_status_css = "un";
}


// Launch 
// (default status)
$v_launch_msg = "Not ready";
$v_launch_css = "off";


// Arm
if (is_null($rls_arm_status)) {
  $v_arm_status = "Unknown";
  $v_arm_status_css = "un";
} else if ($rls_arm_status == 3) {
  $v_arm_status = $rls_arm_notes;
  $v_arm_status_css = "off";
} else if ($rls_arm_status == -1) {
  $v_arm_status = $rls_arm_notes;
  $v_arm_status_css = "un";
} else if ($rls_arm_status == 2) {
  $v_arm_status = $rls_arm_notes;
  $v_arm_status_css = "off";
} else if ($rls_arm_status == 1) {
  $v_arm_status = $rls_arm_notes;
  $v_arm_status_css = "on";
  $v_launch_msg = "Ready to Launch!";
  $v_launch_css = "ready";
?>
<script>
        $("#launch").click(function() {
        if (reload_paused == 2) {
           return; // The page reload is running
        }

        $("#launch").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        reload_paused = 1;
        var r = confirm("Proceed to Launch?");
        if (r == true) {
           $.ajax({
                   url: "enqueueRequest.php",
                   async: false,
                   cache: false,
                   data: {
                          request: "L"
                         },
                   success: function(s,x) {
                           $("#msgText").html(s);
                           showMsg();
                           v_current_status = getRlsStatus('L', 1);
			   checkStatus('launch', v_current_status, 'L');
                   }
               });
        }
        });
</script>
<?

} else if ($rls_arm_status == 0) {
  $v_arm_status = $rls_arm_notes;
  $v_arm_status_css = "off";
}



// Continuity Test
if (is_null($rls_ct_status)) {
  $v_ct_status = "Unknown";
  $v_ct_status_css = "un";
} else if ($rls_ct_status == 1) {
  $v_ct_status = $rls_ct_notes;
  $v_ct_status_css = "on";
} else if ($rls_ct_status == 0) {
  $v_ct_status = $rls_ct_notes;
  $v_ct_status_css = "off";
} else if ($rls_ct_status == 2) {
  $v_ct_status = $rls_ct_notes;
  $v_ct_status_css = "off";
} else if ($rls_ct_status == -1) {
  $v_ct_status = $rls_ct_notes;
  $v_ct_status_css = "un";
}


?>
    <li>
       <a id="powertoggle" class="styled-button-<?= $v_power_status_css?>">Power</a>
       <div>
          Currently: <?= $v_power_status?> - <abbr class="timeago" title="<?= $rls_power_status_date?>"></abbr>
       </div>
    </li>
    <li>
       <a id="continuitytest" class="styled-button-<?= $v_ct_status_css?>">Continuity Test</a>
       <div>
          Info: <?= $v_ct_status?> - <abbr class="timeago" title="<?= $rls_ct_status_date?>"></abbr> 
       </div>
    </li>
    <li>
       <a id="arm" class="styled-button-<?= $v_arm_status_css?>">Arm</a>
       <div>
          Currently: <?= $v_arm_status?> - <abbr class="timeago" title="<?= $rls_arm_status_date?>"></abbr> 
       </div>
    </li> 
    <li>
       <a id="launch" class="styled-button-<?= $v_launch_css?>"><?= $v_launch_msg?></a>
    </li>
  </ul>
</div>
