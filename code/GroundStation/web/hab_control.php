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
        $("#cutdown").click(function() {
        alert("About to request cutdown");
        $.ajax({
                url: "cutdown.php",
                success: function(s,x) {
                        $("#hab_contol").html(s);
                }
            });
        });


        $("#nophotos").click(function() {
        alert("About to disable/enable photo downloads");
        $.ajax({
                url: "nophotos.php",
                success: function(s,x) {
                        $("#hab_control").html(s);
                }
            });
        });
</script>
<h3>HAB control</h3>
<div>
<h2>Cutdown</h2>
<?
if (file_exists($cutdown_init_file)) {
  $cutdown_msg = "Cutdown initiated";
} else if (file_exists($cutdown_req_file)) {
  $cutdown_msg = "Cutdown requested";
} else {
  $cutdown_msg = "";
}

if ($cutdown_msg != "") {
?>
        <b>Cutdown Status</b>: <?= $cutdown_msg ?>
<?
} else {
?>
<input id="cutdown" type="button" value="Initiate Cutdown"/>
<?
}
?>

<h2>Enable/Disable Photo Downloads</h2>
<?
if (file_exists($nophotos_file)) {
  $nophotos_msg = "Disabled";
  $nophotos_button_msg = "Enable";
} else {
  $nophotos_msg = "Enabled";
  $nophotos_button_msg = "Disable";
}
?>
<input id="nophotos" type="button" value="<?=$nophotos_button_msg?> Photo Download"/>
(Currently <?=$nophotos_msg?>)
</div>

