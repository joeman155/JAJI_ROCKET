#!/bin/sh

osmosis --read-xml file="australia-latest.osm" --bounding-polygon file="qld.poly" --write-xml file="qld.australia-latest.osm"

bzip2 qld.australia-latest.osm
