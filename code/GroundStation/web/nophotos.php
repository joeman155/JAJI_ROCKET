<?
# CONFIGUIRATION
include "config.inc";

if (file_exists($nophotos_file)) {
  `rm -f $nophotos_file`;
  print "Enabling photo downloads...";
} else {
   `touch $nophotos_file`;
   print "Disabling photo downloads...";
}
