<?

# CONFIGURATION
include "config.inc";
include "functions.php";

# Get all the latest measurements
try {
     $dbh = new PDO("pgsql:user=www-data dbname=rls password=joeman");
    }
catch (PDOException $e)
    {
     echo $e->getMessage();
    }


# Orientation queries....
?>
<script>
    var y = 0;
    var rcc = 0;
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

    var geometry_cube = new THREE.BoxGeometry( 10, 10, 10 );
    var geometry_rls = new THREE.CylinderGeometry(1,1,10,16);
    var material = new THREE.MeshBasicMaterial( { vertexColors: THREE.FaceColors} );
    var material2 = new THREE.LineBasicMaterial( { color: 0xff0000, linewidth: 1 } );
    var material3 = new THREE.MeshBasicMaterial( { color: 0xff0000, linewidth: 2, wireframe: true } );
    var xn = new THREE.Vector3(1,0,0);
    var yn = new THREE.Vector3(0,1,0);
    var zn = new THREE.Vector3(0,0,1);
    var tx = new THREE.Vector3();
    var ty = new THREE.Vector3();
    var tz = new THREE.Vector3();
    for (var i = 0; i < geometry_rls.faces.length; i++) {
      tx.crossVectors(xn, geometry_rls.faces[i].normal);
      ty.crossVectors(yn, geometry_rls.faces[i].normal);
      tz.crossVectors(zn, geometry_rls.faces[i].normal);
      if (ty.x == 0 && ty.y == 0 && ty.z == 0) {
         geometry_rls.faces[i].color.set( 0xaaaa00 );
      } else if (i ==0 || i == 1 ) {
        geometry_rls.faces[i].color.set( 0xff0000 );
      } else if (i == 16 || i == 17) {
        geometry_rls.faces[i].color.set( 0x999999 );
      } else if (i == 8 || i == 9 || i == 24 || i == 25) {
        geometry_rls.faces[i].color.set( 0x00ff00 );
      } else {
        geometry_rls.faces[i].color.set( 0x0000aa );
      }
    }
    var static_cube = new THREE.Mesh( geometry_cube, material3 );
    var rls = new THREE.Mesh( geometry_rls, material );
    var line = new THREE.Line( geometry_rls, material2 );

    scene.add( rls );
    scene.add( static_cube );

    camera.position.x = 8;
    camera.position.y = 8;
    camera.position.z = 15;
    camera.lookAt( new THREE.Vector3(-8, -8, -15));

    // geometry_rls.rotateZ(Math.PI/2);

    var render = function () {
            requestAnimationFrame( render );
        var current_index = $("#tabs").tabs("option","active");
        if (current_index != 5) return;

    // Don't want a refresh ALL the time....there aren't enough updates
    // to warrant this.
    if (rcc < 2) {
       rcc++;
       return;
    } else  {
      rcc = 0;
    }


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
//	    cube.rotation.z = Math.PI/2;
//	    line.rotation.z = Math.PI/2;

            // We update Quaternion ourselves
            rls.matrixAutoUpdate = false;
            line.matrixAutoUpdate = false;

            // Get Latest rotation angles
            getOrientation();

// ADJUSTMENTS THAT NEED TO ULTIMATELY FIND THEIR WAY BACK IN ARDUINO
yaw = - yaw;
pitch = - pitch;

            var q = new THREE.Quaternion(); // For Yaw
            var r = new THREE.Quaternion(); // For Pitch
            var s = new THREE.Quaternion(); // For Roll
            var zi = new THREE.Quaternion(); // For Intermediate Quaternion
            var z = new THREE.Quaternion(); // For Resultant Quaternion

// joe
// y = y + 5;
            q.setFromAxisAngle(new THREE.Vector3(0,1,0), yaw * Math.PI/180);
            r.setFromAxisAngle(new THREE.Vector3(0,0,1), pitch * Math.PI/180);
            s.setFromAxisAngle(new THREE.Vector3(1,0,0), roll * Math.PI/180);
            zi.multiplyQuaternions(q,r)
            z.multiplyQuaternions(zi,s)
            rls.matrix.makeRotationFromQuaternion(z);
            line.matrix.makeRotationFromQuaternion(z);
//            rls.matrix.makeRotationFromQuaternion(r);
//            line.matrix.makeRotationFromQuaternion(r);



            // rls.rotation.y = 3.141592 * yaw / 180;
            // line.rotation.y = 3.141592 * yaw / 180;
            // rls.rotation.z = 3.141592 * pitch / 180;
            // line.rotation.z = 3.141592 * pitch / 180;
            // rls.rotation.x = 3.141592 * roll / 180;
            // line.rotation.x = 3.141592 * roll / 180;
            


            renderer.render(scene, camera);
    };

    render();


</script>
<h3>Orientation</h3>
<div id="canvas">

</div>
