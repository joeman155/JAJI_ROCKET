function hideAddressBar()
{
  if(!window.location.hash)
  {
      if(document.height < window.outerHeight)
      {
          document.body.style.height = (window.outerHeight + 50) + 'px';
      }

      setTimeout( function(){ window.scrollTo(0, 1); }, 50 );
  }
}

window.addEventListener("load", function(){ if(!window.pageYOffset){ hideAddressBar(); } } );
window.addEventListener("orientationchange", hideAddressBar );

var v_local_lat;
var v_local_long;
var v_local_alt;
var v_got_gps=0;
var v_local_timestamp;

function gps_success_callback(p)
{
 v_local_lat=p.coords.latitude.toFixed(8);
 v_local_long=p.coords.longitude.toFixed(8);
 v_local_alt=p.coords.altitude;
 v_local_timestamp=p.coords.timestamp;
 v_got_gps=1;
 v_local_timestamp = new Date();
 v_local_timestamp.format("dd/mm/yy h:mm:ss");

 $("#latitude").html(v_local_lat);
// alert('lat='+v_local_lat+';lon='+v_local_long+';alt='+v_local_alt);
}

function gps_error_callback(p)
{
 v_got_gps=0;

 switch (p.code)
   {
    case p.PERMISSION_DENIED:
      $("#gps_error").html("User denied the request for Geolocation.");
      break;
    case p.POSITION_UNAVAILABLE:
      $("#gps_error").html("Location information is unavailable.");
      break;
    case p.TIMEOUT:
      $("#gps_error").html("The request to get user location timed out.");
      break;
    case p.UNKNOWN_ERROR:
      $("#gps_error").html("An unknown error occurred.");
      break;
   }
}


var executeOnce = (function (fn, delay) {
  var executed = false;
  return function (/* args */) {
    var args = arguments;
    if (!executed) {
      setTimeout(function () {
        fn.apply(null, args); // preserve arguments
      }, delay);
      executed = true;
    }
  };
});


function hideMsg()
{
  $("#msg").dialog("close");
}


function showMsg()
{
  $("#msg").dialog("open");
  // setTimeout(function(){ $("#msg").dialog("option", "hide", "fade").dialog("close"); reload_paused = 0; }, 2000);
}



function getRlsStatus(p_request_code, p_exclude_pending)
{
 var v_status;
        $.ajax({
                url: "getRlsStatus.php",
                async: false,
                cache: false,
		data: {
                       request: p_request_code,
		       exclude_pending: p_exclude_pending
                      },
                success: function(s,x) {
			v_status = s;
                }
            });

 return v_status;
}


// Check to see if status changes over specified interval (where we repeatly
// query the groundStation for updates...)
//
// 0 = no change
// 1 = change occured
//
// Note: We are only dealing with 'on/off' logic here...
//       p_old_status will be 1 (on) or 0 (off)
//
function checkStatus(buttonName, p_request_code, p_old_status)
{
 var timesRun = 0;
 var status_changed = -1;
 var refreshId = setInterval(function() {

    v_new_status = getRlsStatus(p_request_code, 1)
    // alert('old status: ' + p_old_status + ', new status: ' + v_new_status);

    // See if status has changed.... if so...return 1
    if (p_old_status != v_new_status) {
       clearInterval(refreshId);
       toggle(buttonName, p_old_status, v_new_status);
       reload_paused = 0;                    // Re-enable page reloads
       hideMsg();                            // Hide the Message Box
    }

    if (timesRun > 20) {
       // Got to here with no suggestion that the status changed. It _could_ have changed
       // but we got no indication that it did change
       // Setting to state to indicate we aren't sure.
       clearInterval(refreshId);
       $("#" + buttonName).css("background", "");
       $('#' + buttonName).removeClass("styled-button-on");
       $('#' + buttonName).removeClass("styled-button-off");
       $('#' + buttonName).addClass("styled-button-un");
       reload_paused = 0;                    // Re-enable page reloads
       hideMsg();                            // Hide the Message Box
    }

    timesRun = timesRun + 1; 
    

 }, 500);

}


// Toggle colours on button and remove spinner
// Notes: 
//  -- buttonName is name of button
//  -- oldState is old state  0 or 1 or something else. 
//  -- newState is the new state 0 or 1 or something else
//
// State 0 is a special state (off)
// State 1 is a special state (on)
// State X (where X > 1) is usually an error...or unknown
//
function toggle(buttonName, oldState, newState)
{

 if (oldState == 0 && (newState == 1 || newState == 0)) {
    $('#' + buttonName).removeClass("styled-button-off");
    $('#' + buttonName).addClass("styled-button-on");
 } else if (oldState == 1 && (newState == 1 || newState == 0)) {
    $('#' + buttonName).removeClass("styled-button-on");
    $('#' + buttonName).addClass("styled-button-off");
 } else {
    $('#' + buttonName).removeClass("styled-button-on");
    $('#' + buttonName).removeClass("styled-button-off");
    $('#' + buttonName).addClass("styled-button-un");
 }

 $("#" + buttonName).css("background", "");

}
