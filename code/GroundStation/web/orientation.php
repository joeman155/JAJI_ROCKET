<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest measurements
try {
     $dbh = new PDO("sqlite:" . $db_file);
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


# Orientation queries....
?>
<script>
    var roll;
    var pitch;
    var yaw;
    yaw = roll = pitch = 0;

    // Function to get Orientation
    function getOrientation()
    {
    // Poll Server
    $.ajax({
            url: 'getOrientation.php',
            type: "GET",
            dataType: "json",
            async: false,
            cache: false,
            success: function(data) {
                       if (data.id !== undefined) {
                          // alert('id: ' + data.id);
                          pitch = data.pitch;
                          roll  = data.roll;
                          yaw   = data.yaw;
                       }
                    },
            failure: function() {
                    alert('failure occured');
                    }

            });
    }

    // Schedule routine to get orientation regularly
    // setInterval(function(){
    //       getOrientation();
    //       }, 1000);


    // 3d Drawing
    var ch = 0.01;
    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera( 75, window.innerWidth/window.innerHeight, 0.1, 1000 );

    var renderer = new THREE.WebGLRenderer();
    renderer.setSize( window.innerWidth, window.innerHeight );
    // document.body.appendChild( renderer.domElement );
    container = document.getElementById( 'canvas' );
    container.appendChild(renderer.domElement);

    // var geometry = new THREE.BoxGeometry( 1, 1, 1 );
    var geometry = new THREE.CylinderGeometry(1,1,10,16);
    // var material = new THREE.MeshBasicMaterial( { color: 0x00ff00} );
    var material = new THREE.MeshBasicMaterial( { vertexColors: THREE.FaceColors} );
    var material2 = new THREE.LineBasicMaterial( { color: 0xff0000, linewidth: 1 } );
    var a = new THREE.Vector3(0,1,0);
    var c = new THREE.Vector3();
    for (var i = 0; i < geometry.faces.length; i++) {
      c.crossVectors(a, geometry.faces[i].normal);
      if (c.x == 0 && c.y == 0 && c.z == 0) {
         geometry.faces[i].color.set( 0xaaaa00 );
      } else {
        if (i == 20 || i == 21) {
          geometry.faces[i].color.set( 0x0000aa );
        } else {
          geometry.faces[i].color.set( 0x00aa00 );
        }
      }
    }
    var cube = new THREE.Mesh( geometry, material );
    var line = new THREE.Line( geometry, material2 );

    scene.add( cube );
    // scene.add( line );

    camera.position.z = 15;

    var render = function () {
            requestAnimationFrame( render );
        var current_index = $("#tabs").tabs("option","active");
        if (current_index != 5) return;


/*
            cube.rotation.y += 0.02;
            line.rotation.y += 0.02;
            if (cube.rotation.x > .8) {
               ch = -0.02;
            }
            if (cube.rotation.x < -.8) {
               ch = 0.02;
            }
            cube.rotation.x += ch;
            line.rotation.x += ch;
            cube.rotation.z += ch;
            line.rotation.z += ch;
*/
            // Get Latest rotation angles
            getOrientation();

            // var m = new THREE.Matrix4();
            // m.makeRotationX(pitch * 3.1415/180);
            cube.rotation.x = 3.141592 * pitch / 180;
            line.rotation.x = 3.141592 * pitch / 180;
            cube.rotation.z = 3.141592 * roll / 180;
            line.rotation.z = 3.141592 * roll / 180;
            


            renderer.render(scene, camera);
    };

    render();


</script>
<h3>Orientation</h3>
<div id="canvas">

</div>
