<?
$pid = `ps -ef | grep groundStation.pl | grep -v grep | awk '{print $2}'`;
if (is_null($pid)) {
$msg = "GroundStation not running.";
} else {
$msg = "Running (" . $pid . ")";
}

echo $msg;
