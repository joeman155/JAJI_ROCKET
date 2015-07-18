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
        $.ajax({
                url: "enqueueRequest.php",
                data: {
                       request: "K"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                }
            });
        });


        $("#nophotos").click(function() {
        $.ajax({
                url: "enqueueRequest.php",
                data: {
                       request: "N"
                      },
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                }
            });
        });
</script>
<h3>HAB control</h3>

<div id="list_wrapper">
<ul class="multiple_columns">
<li>
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
<input id="cutdown" type="button" class="styled-button-on" value="Initiate Cutdown"/>
<?
}
?>
</li>
<li>
<h2>Enable/Disable Photo Downloads</h2>
<?
if (file_exists($nophotos_file)) {
  $nophotos_msg = "Disabled";
  $nophotos_button_msg = "Enable";
  $nophotos_css = "off";
} else {
  $nophotos_msg = "Enabled";
  $nophotos_button_msg = "Disable";
  $nophotos_css = "on";
}
?>
<input id="nophotos" type="button" class="styled-button-<?= $nophotos_css?>" value="<?=$nophotos_button_msg?> Photo Download"/>
(Currently <?=$nophotos_msg?>)
</li>
</ul>
</div>

