    function initmap() {
           // Get Viewers GPS info
          if (navigator.geolocation)
            {
          navigator.geolocation.watchPosition(gps_success_callback);
            }
          else
            {
            $("#gps_error").html("Geolocation is not supported by this browser.");
            }

	// Initialise the map
           var options = {
                projection: new OpenLayers.Projection("EPSG:900913"),
                displayProjection: new OpenLayers.Projection("EPSG:4326"),
                units: "m",
                maxResolution: 156543.0339,
                maxExtent: new OpenLayers.Bounds(-20037508.34, -20037508.34,
                                                 20037508.34, 20037508.34),
                numZoomLevels: 20,
                controls: [
                        new OpenLayers.Control.Navigation(),
                        new OpenLayers.Control.PanZoomBar(),
                        new OpenLayers.Control.Permalink(),
                        new OpenLayers.Control.ScaleLine(),
                        new OpenLayers.Control.MousePosition(),
                        new OpenLayers.Control.KeyboardDefaults(),
			new OpenLayers.Control.Graticule({
				numPoints: 1,
				labelled: true,
				visible: true
                            })
 
                  ]
            };
        var fromProjection = new OpenLayers.Projection("EPSG:4326");   // Transform from WGS 1984
        var toProjection   = new OpenLayers.Projection("EPSG:900913"); // to Spherical Mercator Projection
        var start_position       = new OpenLayers.LonLat(145.3,-17.3838).transform( fromProjection, toProjection);
        var zoom           = 10;


	// Balloon tracking icon
        var size = new OpenLayers.Size(21,25);
        var offset = new OpenLayers.Pixel(-(size.w/2), -size.h);
        var balloon_start_icon = new OpenLayers.Icon('/api/img/start.png', size, offset);
        var balloon_icon = new OpenLayers.Icon('/api/img/balloon.png', size, offset);

	// Tracking Vehicle icon
	var vehicle_icon = new OpenLayers.Icon('/api/img/vehicle.png', size, offset);

	// Create layer for markers
        var markers = new OpenLayers.Layer.Markers( "Markers" );
        var static_markers = new OpenLayers.Layer.Markers( "Markers_Predict" );

	// New Map
        map = new OpenLayers.Map("map-canvas",options);
        var newL = new OpenLayers.Layer.OSM("Default", "/osm_tiles/${z}/${x}/${y}.png", {numZoomLevels: 19});


	// Overlay for balloon flight path
        // allow testing of specific renderers via "?renderer=Canvas", etc
        var renderer = OpenLayers.Util.getParameters(window.location.href).renderer;
        renderer = (renderer) ? [renderer] : OpenLayers.Layer.Vector.prototype.renderers; 
	var vec = new OpenLayers.Layer.Vector("Overlay", {
                styleMap: new OpenLayers.StyleMap({'default':{
                    strokeColor: "#00FF00",
                    strokeOpacity: 1,
                    strokeWidth: 3,
                    fillColor: "#FF5500",
                    fillOpacity: 0.5,
                    pointRadius: 6,
                    pointerEvents: "visiblePainted",
                    // label with \n linebreaks
                    label : "${gps_time} (${creation_date})\n${pos}",
                    
                    fontColor: "${favColor}",
                    fontSize: "10px",
                    fontFamily: "Courier New, monospace",
                    fontWeight: "bold",
                    labelAlign: "${align}",
                    labelXOffset: "${xOffset}",
                    labelYOffset: "${yOffset}",
                    labelOutlineColor: "white",
                    labelOutlineWidth: 3
                }}),
                renderers: renderer
        });


        // Overlay for balloon flight path prediction
        // allow testing of specific renderers via "?renderer=Canvas", etc
        var renderer = OpenLayers.Util.getParameters(window.location.href).renderer;
        renderer = (renderer) ? [renderer] : OpenLayers.Layer.Vector.prototype.renderers;
        var static_vec = new OpenLayers.Layer.Vector("Overlay_Prediction", {
                styleMap: new OpenLayers.StyleMap({'default':{
                    strokeColor: "#00FF00",
                    strokeOpacity: 1,
                    strokeWidth: 3,
                    fillColor: "#FF5500",
                    fillOpacity: 0.5,
                    pointRadius: 6,
                    pointerEvents: "visiblePainted",
                    // label with \n linebreaks
                    label : "${time}\n${pos}",

                    fontColor: "${favColor}",
                    fontSize: "10px",
                    fontFamily: "Courier New, monospace",
                    fontWeight: "bold",
                    labelAlign: "${align}",
                    labelXOffset: "${xOffset}",
                    labelYOffset: "${yOffset}",
                    labelOutlineColor: "white",
                    labelOutlineWidth: 3
                }}),
                renderers: renderer
        });



        // Get GPS co-ordinates for the current day.
	var balloon_gps;
	var points = [];
	var p_flag = 0;
	var flight_style = {strokeColor:"#0500bd", strokeWidth:3};
        $.ajax({
                url: 'get1DGPS.php',
                type: "GET",
                dataType: "json",
                async: false,
                cache: false,
                success: function(data) {
			$.each(data, function(index,element) {
				v_total = data.length; // Total # of points. Only want position on LAST point
                 		v_lat=element.latitude;
                	        v_long=element.longitude;
				v_pos = v_lat + "," + v_long + "," + element.height;
        			balloon_gps = new OpenLayers.LonLat(v_long,v_lat).transform( fromProjection, toProjection);
				if (p_flag == 0) {
        				markers.addMarker(new OpenLayers.Marker(balloon_gps, balloon_start_icon));
					p_flag = 1;
				} else {
        				markers.addMarker(new OpenLayers.Marker(balloon_gps, balloon_icon.clone()));
				}

				// Add Position and time of recording for LAST point
				if (index == v_total - 1) {
				   var labelOffsetPoint = new OpenLayers.Geometry.Point(v_long,v_lat).transform( fromProjection, toProjection);
			 	   var labelOffsetFeature = new OpenLayers.Feature.Vector(labelOffsetPoint);
				   labelOffsetFeature.attributes = {
				   creation_date: element.gps_creation_date,
				   gps_time: element.gps_time,
				   pos: v_pos,
				   favColor: 'blue',
				   align: "cm",
				   // positive value moves the label to the right
				   xOffset: 0,
				   // negative value moves the label down
				   yOffset:-20
				   };
				   vec.addFeatures(labelOffsetFeature);
				}

				points.push( new OpenLayers.Geometry.Point(v_long,v_lat).transform( fromProjection, toProjection));
				});
                        },
                failure: function() {
                        alert('failure when getting latest gps data');
                        }
                });


	// Draw balloon path
	var line = new OpenLayers.Geometry.LineString(points);
	var fea = new OpenLayers.Feature.Vector(line, {}, flight_style);


        // Get predicted flight plan
        var predicted_flight_path_gps;
        var points1 = [];
        var p_flag = 0;
        var flight_prediction_style = {strokeColor:"#FF0000", strokeWidth:3};
        $.ajax({
                url: 'getPredictedFlightPath.php',
                type: "GET",
                dataType: "json",
                async: false,
                cache: false,
                success: function(data) {
                        $.each(data, function(index,element) {
                                v_lat=element.latitude;
                                v_long=element.longitude;
        			balloon_gps = new OpenLayers.LonLat(v_long,v_lat).transform( fromProjection, toProjection);
				if (p_flag == 0) {
        				static_markers.addMarker(new OpenLayers.Marker(balloon_gps, balloon_start_icon.clone()));
					p_flag = 1;
				} 
                                points1.push( new OpenLayers.Geometry.Point(v_long,v_lat).transform( fromProjection, toProjection));
                                });
                        },
                failure: function() {
                        alert('failure when getting flight prediction path data');
                        }
                });


	// Draw predicted flight path
	var line1 = new OpenLayers.Geometry.LineString(points1);
	var fea1 = new OpenLayers.Feature.Vector(line1, {}, flight_prediction_style);


	// Add all features to vectory
	vec.addFeatures([fea]);          // non-static features
	static_vec.addFeatures([fea1]);  // static features


	// Draw marker of vehicle
	if (v_local_lat) {
        	vehicle_gps = new OpenLayers.LonLat(v_local_long,v_local_lat).transform( fromProjection, toProjection);
		markers.addMarker(new OpenLayers.Marker(vehicle_gps, vehicle_icon));
	}

	// Add Layers
        map.addLayer(newL);
        map.addLayer(static_markers);
        map.addLayer(markers);
        map.addLayer(static_vec);
        map.addLayer(vec);

	// Center map and zoom in
	map.setCenter(start_position, zoom);
        map.zoomIn();
 }
