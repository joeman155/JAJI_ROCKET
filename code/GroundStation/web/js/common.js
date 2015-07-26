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



function getRlsStatus(p_request_code)
{
 var v_status;
        $.ajax({
                url: "getRlsStatus.php",
                async: false,
                cache: false,
		data: {
                       request: p_request_code
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
function checkStatus(buttonName, p_old_status,p_request_code)
{
 var timesRun = 0;
 var status_changed = -1;
 var refreshId = setInterval(function() {

    v_new_status = getRlsStatus(p_request_code)
    // alert('old status: ' + p_old_status + ', new status: ' + v_new_status);

    // See if status has changed.... if so...return 1
    if (p_old_status != v_new_status) {
       clearInterval(refreshId);
       toggle(buttonName, p_old_status);
    }

    if (timesRun > 5) {
       clearInterval(refreshId);
    }

    timesRun = timesRun + 1; 
    

 }, 1000);

 // Got to here with no suggestion that the status changed. It _could_ have changed
 // but we got no indication that it did change
}


function toggle(buttonName, oldState)
{

 if (oldState == 0) {
    $('#' + buttonName).addClass("styled-button-on");
    alert('ON');
 } else if (oldState == 1) {
    $('#' + buttonName).addClass("styled-button-off");
    alert('OFF');
 } else {
    $('#' + buttonName).addClass("styled-button-un");
 }

 $("#" + buttonName).css("background", "");

}
