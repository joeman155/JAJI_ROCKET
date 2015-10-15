<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest measurements
try {
     $dbh = new PDO("pgsql:user=www-data dbname=rls");
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


# Get latest launch systems status
## PHOTO
$nophotos_status              = get_rls_status("N");
$rls_nophotos_status          = $nophotos_status["status"];
$rls_nophots_notes            = $nophotos_status["notes"];
$rls_nophotos_status_date_raw = $nophotos_status["creation_date"];
$rls_nophots_status_date      =  date("Y-m-d H:i:s", strtotime($rls_nophotos_status_date_raw));
$rls_nophots_is_pending       = is_pending_request("N");

## CUTDOWN
$cutdown_status                = get_rls_status("K");
$rls_cutdown_status            = $cutdown_status["status"];
$rls_cutdown_notes             = $cutdown_status["notes"];
$rls_cutdown_status_date_raw   = $cutdown_status["creation_date"];
$rls_cutdown_status_date       =  date("Y-m-d H:i:s", strtotime($rls_cutdown_status_date_raw));
$rls_cutdown_is_pending        = is_pending_request("K");

## PROFILE
$profile_status                = get_rls_status("I");
$rls_profile_status            = $profile_status["status"];
$rls_profile_notes             = $profile_status["notes"];
$rls_profile_status_date_raw   = $profile_status["creation_date"];
$rls_profile_status_date       =  date("Y-m-d H:i:s", strtotime($rls_profile_status_date_raw));
$rls_profile_is_pending        = is_pending_request("I");



# print "rls_nophotos_status: " . $rls_nophotos_status . "<br>\n";
# print "rls_cutdown_status: "  . $rls_cutdown_status . "<br>\n";

?>

<script>
        $("#nophotos").click(function() {
        reload_paused = 1;
        $("#nophotos").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        $.ajax({
                url: "enqueueRequest.php",
                data: {
                       request: "N"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                        v_current_status = getRlsStatus('N', 1);
                        checkStatus('nophotos', 'N', v_current_status);
                }
            });
        });
</script>
<h3>HAB control</h3>


<?

# NO PHOTOS
 if ($rls_nophotos_status == 1) {
    $nophotos_msg = "Enabled";
    $nophotos_button_msg = "Disable";
    $rls_nophotos_status_css = "on";
 } else if ($rls_nophotos_status == 0) {
    $nophotos_msg = "Disabled";
    $nophotos_button_msg = "Enable";
    $rls_nophotos_status_css = "off";
 } else {
    $nophotos_msg = "Unknown";
    $nophotos_button_msg = "Enable";
    $rls_nophotos_status_css = "un";
 } 



# CUTDOWN
 if ($rls_cutdown_status == 1) {
    $rls_cutdown_status_css = "on";
 } else if ($rls_cutdown_status == 2) {
    $rls_cutdown_status_css = "on";
 } else if ($rls_cutdown_status == 0) {
    $rls_cutdown_status_css = "off";
 } else { 
    $rls_cutdown_status_css = "un";
 }

 $cutdown_msg = $rls_cutdown_notes;


# PROFILE
 $profile_msg = $rls_profile_notes;
 if ($rls_profile_status == 1) {
    $profile_msg = "Standard";
    $profile_button_text = "IMU";
    $rls_profile_status_css = "on";
 } else if ($rls_profile_status == 2) {
    $profile_msg = "IMU";
    $profile_button_text = "Standard";
    $rls_profile_status_css = "on";
 } else {
    $rls_profile_status_css = "un";
    $profile_msg = "Unknown";
 }


# Only allow cutdown button to work IF the cutdown status <> 1...i.e. cutdown not initiated yet.
if ($rls_cutdown_status != 1) {
?>
<script>
        $("#cutdown").click(function() {
           reload_paused = 1;

           var r = confirm("Proceed with Cutdown?");
           if (r == true) {
              $("#cutdown").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
              $.ajax({
                      url: "enqueueRequest.php",
                      data: {
                             request: "K"
                            },
                      success: function(s,x) {
                              $("#msgText").html(s);
                              showMsg();
                              v_current_status = getRlsStatus('K', 1);
                              checkStatus('cutdown', 'K', v_current_status);
                      }
                  });
           } else {
              reload_paused = 0;
           }
        });
</script>
<?
}
?>


<script>
        $("#profile").click(function() {
        reload_paused = 1;
        $("#profile").css("background", "url(/images/ajax-loader.gif) no-repeat center center");
        $.ajax({
                url: "enqueueRequest.php",
                data: {
                       request: "I"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
                        showMsg();
                        v_current_status = getRlsStatus('I', 1);
                        checkStatus('profile', 'I', v_current_status);
                }
            });
        });
</script>




<div id="list_wrapper">
   <ul class="multiple_columns">
      <li>
         <h2>Cutdown</h2>
         <a id="cutdown" class="styled-button-<?= $rls_cutdown_status_css?>">Initiate Cutdown</a>
           <div>
              <?= $cutdown_msg?>
           </div>
      </li>
      <li>
         <h2>Enable/Disable Photo Taking</h2>
         <a id="nophotos" class="styled-button-<?= $rls_nophotos_status_css?>"><?=$nophotos_button_msg?> Photos Taking</a> 
         <div>
           (Currently <?=$nophotos_msg?>)
         </div>
      </li>
      <li>
         <h2>Profile</h2>
         <a id="profile" class="styled-button-<?= $rls_profile_status_css?>">Set Profile to <?= $profile_button_text?></a>
           <div>
              (Currently: <?= $profile_msg?>)
           </div>
      </li>
   </ul>
</div>

