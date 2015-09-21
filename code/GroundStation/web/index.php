<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>GroundStation Client Web Portal</title>
  <link rel="stylesheet" href="images.css">
  <link rel="stylesheet" href="jq/themes/ui-lightness/jquery-ui.css">


  <script src="api/OpenLayers.js"></script>
  <script src="js/map.js"></script>
  <script src="jq/jquery-1.9.1.js"></script>
  <script src="jq/ui/jquery.ui.core.js"></script>
  <script src="jq/ui/jquery.ui.widget.js"></script>
  <script src="jq/ui/jquery.ui.mouse.js"></script>
  <script src="jq/ui/jquery.ui.button.js"></script>
  <script src="jq/ui/jquery.ui.accordion.js"></script>
  <script src="jq/ui/jquery.ui.position.js"></script>
  <script src="jq/ui/jquery.ui.effect.js"></script>
  <script src="jq/ui/jquery.ui.effect-slide.js"></script>
  <script src="jq/ui/jquery.ui.tabs.js"></script>
  <script src="jq/ui/jquery.ui.dialog.js"></script>
  <script src="js/jquery.cookie.js"></script>
  <script src="js/jquery.timeago.js"></script>
  <script src="js/three.min.js"></script>
  <script src="js/common.js" type="text/javascript" charset="utf-8"></script>

  <meta name="viewport" content="initial-scale=1.0, user-scalable=no">
  <meta charset="utf-8">

  <link rel="stylesheet" type="text/css" href="map.css">
  <link rel="stylesheet" type="text/css" href="main.css">


  <script>
	// Timer to intialize the map and re-initialize every 120 seconds
	var map_active = 0;
	setInterval(function(){
		if (map_active == 1) {
			map.destroy();
        		initmap();
     		}	
		}, 
	120000);

	// GPS
        if (navigator.geolocation) {
             navigator.geolocation.getCurrentPosition(gps_success_callback,gps_error_callback,{enableHighAccuracy:true});
        } else {
            $("#gps_error").html("Geolocation is not supported by this browser.");
        }

	$(function() {
	// Initiailise the tab area
		$( "#tabs" ).tabs( {
			activate: function(e,ui) {
			var active = $("#tabs").tabs("option", "active");
				if (active == 3) {
					initmap();
					map_active = 1;
				} else {
					map_active = 0;
                                }
			}
		});

		$ ("#addrow").click(function () {
		slideDown();
		return false;
		});

		var last_id;
		var processing = 0;

		function getMessages() 
		{
		// Poll Server
		$.ajax({
			url: 'getMessage.php',
			data: { id: last_id },
			type: "GET",
			dataType: "json",
			async: false,
			cache: false,
			success: function(data) {
				$.each(data, function(index,element) {
				// Foreach message...add it to the browser page.
				last_id = element.id;
				addMessage("<li>" + element.id + " (" + element.creation_date + ") - " + element.message + "</li>");
				});
				},
			failure: function() {
				alert('failure occured');
				}

			});
		}

	function addMessage(message)
	{
		$("#logs").prepend(message);
		$("#logs li:first").effect("slide", {direction: "down"}, "fast");
	}

	function  load_datetime()
	{
	$.ajax({
	url: 'getdatetime.php',
	cache: false,
	async: false,
	success: function(s,x) {
		$('body').css('background', 'white');
		$("#datetime").html('<h5>' + s + '</h5>');
		},
	error: function() {
		$('body').css('background', 'red');
		}
	});
	}

	function  load_pid()
	{
	$.ajax({
	url: 'getpid.php',
	cache: false,
	async: false,
	success: function(s,x) {
	$("#pid").html('<h5>Daemon Status: ' + s + '</h5>');
		}

	});
	}



	// Initial load
	load_datetime();
	load_pid();

	setInterval(function(){
	        load_datetime();
	        load_pid();
	        }, 5000);


	setInterval(function(){
	  if (processing == 0) {
		processing = 1;
		getMessages();
		}
	  processing = 0;
	  }, 2000);
	});

        // Initialise the Messages dialog
        $(function() {
          $( "#msg" ).dialog({
            resizable: false,
            height:140,
            autoOpen: false,
            closeOnEscape: false,
            modal: true,
            hide: {effect: "fade", duration: 500}
          });
        });

        // Initialise the Countdown dialog
        $(function() {
          $( "#countdown_dialog" ).dialog({
            resizable: false,
            width:400,
            height:200,
            autoOpen: false,
            closeOnEscape: false,
            modal: true,
            hide: {effect: "fade", duration: 500}
          });
        });

	</script>
</head>
<body>

<div id="header" class="header-class" style="width: 500px; padding-top: 3px;">
    <div id="datetime" style="float: left; width: 200px;">
    </div>

    <div id="pid" style="float: left; width: 300px;">
    </div>
    
    <div id="msg" title="Messages">
      <p id="msgText"/>
    </div>

    <div id="countdown_dialog" title="Countdown">
      <p id="countdown">5</p>
    </div>
</div>

<div id="tabs">
	<ul>
		<li><a href="status.html">Status</a></li>
		<li><a href="#logs">Logs</a></li>
		<li><a href="images.php">Images</a></li>
		<li><a href="#map-canvas">Map</a></li>
		<li><a href="hab_control.html">Hab Control</a></li>
		<li><a href="orientation.php">Orientation</a></li>
		<li><a href="gs_control.php">GS Control</a></li>
		<li><a href="launch_console.html">Launch console</a></li>
		<li><a href="upload_prediction.php">Upload Prediction</a></li>
	</ul>
	<div id="logs" class="container">
		<ul id="logs">
		</ul>
	</div>

	<div id="map-canvas" style="height: 1000px">
	</div>

	<div id="status">
	</div>

	<div id="images">
	</div>

	<div id="habcontrol">
	</div>

	<div id="gscontrol">
	</div>

	<div id="orientation">
	</div>

	<div id="launchconsole">
	</div>

	<div id="prediction">
	</div>
</div>

</body>
</html>
