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


// Hide Dialog Box
function hideMsg()
{
  $("#msg").dialog("close");
}

// Show Dialog Box
function showMsg()
{
  $("#msg").dialog("open");
}

// Show the Countdown Box
function showCountdown()
{
  $("#countdown_dialog").dialog("open");
  startTimer();
}

// Hide the countdown Box
function hideCountdown()
{
  $("#countdown_dialog").dialog("close");
}

// Get Status of particular attribute
// Parameters are:-
// - p_request_code   - the particular attribute we are interested in
// - p_exclude_pendng - 1 if we wish to exclude PENDING statuses
//                      0 if we do wish to include PENDING statuses.    
//
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
 var refreshId = setInterval(function() {

    v_new_status = getRlsStatus(p_request_code, 1)
    // alert('old status: ' + p_old_status + ', new status: ' + v_new_status);

    // See if status has changed.... if so...return 1
    if (p_old_status != v_new_status) {
       clearInterval(refreshId);
       toggle(buttonName, v_new_status);
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


// Handle the 'Invalidation' of previous test and then fire off test again
function performContinuityTest(buttonName, p_request_code)
{
 var timesRun = 0;
 var refreshId = setInterval(function() {

    v_new_status = getRlsStatus(p_request_code, 1)

    // See if status is changed to Invalidated...then we can now continue to re-test
    if (v_new_status == 3) {
       clearInterval(refreshId);
       submitContinuityTest();
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


// Handle the 'Invalidation' of previous launch and then perform launch
function performLaunch(buttonName, p_request_code)
{
 var timesRun = 0;
 var refreshId = setInterval(function() {

    v_new_status = getRlsStatus(p_request_code, 1)

    // See if status is changed to Invalidated...then we can now continue to re-test
    if (v_new_status == 9) { 
       clearInterval(refreshId);
       submitLaunch();
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



// Submit request to perform continuity test
function submitContinuityTest()
{

 // Submit request to perform the Continuity Test
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
                 checkStatus('continuitytest', 'C', v_current_status);
         }
     });
}



// Submit request to perform Launch
function submitLaunch()
{

 // Submit request to perform the Continuity Test
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
                 checkStatus('launch', 'L', v_current_status);
//joe
                 showCountdown();
         }
     });
}


// Toggle colours on button and remove spinner
// Notes: 
//  -- buttonName is name of button
//  -- newState is the new state 0 or 1 or something else
//
// State 0 is a special state (off)
// State 1 is a special state (on)
// State X (where X > 1) is usually an error...or unknown
//
function toggle(buttonName, newState)
{

 if (newState == 1 ) {
    $('#' + buttonName).removeClass("styled-button-un");
    $('#' + buttonName).removeClass("styled-button-off");
    $('#' + buttonName).addClass("styled-button-on");
 } else if (newState == 0) {
    $('#' + buttonName).removeClass("styled-button-un");
    $('#' + buttonName).removeClass("styled-button-on");
    $('#' + buttonName).addClass("styled-button-off");
 } else {
    $('#' + buttonName).removeClass("styled-button-on");
    $('#' + buttonName).removeClass("styled-button-off");
    $('#' + buttonName).addClass("styled-button-un");
 }

 $("#" + buttonName).css("background", "");

}


// Countdown Timer
function startTimer() {
    var timeLeft = 5,
        cinterval;
        document.getElementById('countdown').innerHTML = '5';

    var timeDec = function (){
        timeLeft--;
        document.getElementById('countdown').innerHTML = timeLeft;
        if(timeLeft === 0){
            document.getElementById('countdown').innerHTML = 'Blastoff';
        }
        if(timeLeft ===-1) {
            clearInterval(cinterval);
            timeLeft = 5;
            hideCountdown();
        }
    };

    cinterval = setInterval(timeDec, 1000);
}
