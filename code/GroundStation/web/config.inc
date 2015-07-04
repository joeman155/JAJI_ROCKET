<?
$home_dir = "/data/gs";

$db_file = $home_dir . "/db/gs.db";
$cutdown_req_file = $home_dir . "/run/cutdown_requested.txt";
$cutdown_init_file = $home_dir . "/run/cutdown_initiated.txt";
$nophotos_file     = $home_dir . "/run/nophotos.txt";
$x_modem_packet_file = $home_dir . "/run/x_modem_packet";
$download_file_status = $home_dir . "/run/download_file_status";


$threshold_heartbeat = 300;
$threshold_altitude = 27000;
$threshold_distance = 30;
$threshold_temperature_low = 0;
$threshold_temperature_high = 50;
$threshold_satellites = 4;

$v_max_row_count = 35;  
$v_num_spaced_out_points = 20; 
# Logic is this:-
# if less then $v_max_row_count points, show all GPS points
# if more than $v_max_row_count points, show the last ($v_max_row_count - $v_num_spaced_out_points)
#     points AND show $v_num_spaced_out_points 'spaced' out points for the beginning of journey


# REQUEST STATUS's
$REQUEST_ENQUEUE_STATUS    = "V";
$REQUEST_PROCESSING_STATUS = "P";
$REQUEST_FINISHED_STATUS   = "F";

$request_source      = "OPERATOR";
$request_destination = "RLS";

