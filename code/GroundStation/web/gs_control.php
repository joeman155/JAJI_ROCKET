<script>
        $("#shutdown").click(function() {
        $.ajax({
                url: "gs_control.php",
                data: {
                       action: "shutdown"
                      },
                method: 'POST',
                success: function(s,x) {
                        $("#msgText").html(s);
			showMsg();
                }
            });
        });
</script>
<b>Manage Ground Station</b><br/>

<?
include "config.inc";



$action = $_REQUEST['action'];


if ($_SERVER['REQUEST_METHOD'] === 'POST') {
   if ($action == "shutdown") {
     print "Shutting down Groundstation...";
     system("sudo -u root /sbin/shutdown -h now");

   }
} else {
?>

<form action="gs_control.php" method="post">
<input type="button" name="shutdown" id="shutdown" class="styled-button-on" value="shutdown"/><br>
</form>
<?

}
