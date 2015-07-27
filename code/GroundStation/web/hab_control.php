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
$nophotos_status              = get_rls_status("N");
$rls_nophotos_status          = $nophotos_status["status"];
$rls_nophots_notes            = $nophotos_status["notes"];
$rls_nophotos_status_date_raw = $nophotos_status["creation_date"];
$rls_nophots_status_date      =  date("Y-m-d H:i:s", strtotime($rls_nophotos_status_date_raw));
$rls_nophots_is_pending       = is_pending_request("N");

$cutdown_status                = get_rls_status("K");
$rls_cutdown_status            = $cutdown_status["status"];
$rls_cutdown_notes             = $cutdown_status["notes"];
$rls_cutdown_status_date_raw   = $cutdown_status["creation_date"];
$rls_cutdown_status_date       =  date("Y-m-d H:i:s", strtotime($rls_cutdown_status_date_raw));
$rls_cutdown_is_pending        = is_pending_request("K");

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
                        setTimeout(function() { 
                                               $("#msg").dialog("option", "hide", "fade").dialog("close"); 
                                               reload_paused = 1; 
                                               $("#nophotos").css("background", "");
                        }, 2000);
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
    $rls_nophotos_status_css = "un";
 } 



# CUTDOWN
 if ($rls_cutdown_status == 1) {
    $rls_cutdown_status_css = "on";
 } else if ($rls_cutdown_status == 0) {
    $rls_cutdown_status_css = "off";
 } else { 
    $rls_cutdown_status_css = "un";
 }



# Only allow cutdown button to work IF the cutdown status <> 1...i.e. cutdown not initiated yet.
if ($rls_cutdown_status != 1) {
?>
<script>
        $("#cutdown").click(function() {
        reload_paused = 1;
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
        });
</script>
<?
}
?>




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
         <h2>Enable/Disable Photo Downloads</h2>
         <a id="nophotos" class="styled-button-<?= $rls_nophotos_status_css?>"><?=$nophotos_button_msg?> Photos Download</a> 
         <div>
           (Currently <?=$nophotos_msg?>)
         </div>
      </li>
   </ul>
</div>

