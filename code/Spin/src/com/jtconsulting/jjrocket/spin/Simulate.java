package com.jtconsulting.jjrocket.spin;
import java.math.BigDecimal;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

import javax.media.j3d.Appearance;
import javax.media.j3d.BoundingSphere;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.DirectionalLight;
import javax.media.j3d.Material;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.media.j3d.ViewPlatform;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3f;

import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import com.jtconsulting.jjrocket.spin.*;
import com.sun.j3d.utils.applet.MainFrame;
import com.sun.j3d.utils.geometry.ColorCube;
import com.sun.j3d.utils.geometry.Cylinder;
import com.sun.j3d.utils.universe.SimpleUniverse;
import com.sun.j3d.utils.universe.ViewingPlatform;

/*
 * 
 * GLOBAL CO-ORDINATE SYSTEM
 * The x-axis is left/right
 * The y-axis is up-down
 * The z-axis comes out of the page
 * 
 * LOCAL CO-ORDINATE SYSTEM
 * The local co-ordinate system is the same as the GLOBAL CO-ORDINATE SYSTEM at time = 0
 * 
 * This simulates two small rocket engines located half way up the length of a tube, each pointing in opposite directions 
 * (+ve X and -ve X) to make the cylinder spin. 
 * The cylinder is upright, its axis is parallel to y-axis and the bottom end of the cylinder (in the centre) is placed at
 * the origin at time = 0
 * 
 * 
 */
public class Simulate {


	
	/**
	 * @param args
	 */
	/**
	 * @param args
	 */
	/**
	 * @param args
	 */
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		TransformGroup rocket_system = new TransformGroup();
		Transform3D trans = new Transform3D();
		float x_pos = 0;
		
		System.out.println("Running Spin Simulation...");	
		
		// Constants
		double time_slice = 0.001; // How long each time slice is. 
		double total_time = 2;     // How long we do the simulation for
		int    num_intervals = (int) (total_time/time_slice);
		int    distance_test = 10; 

		
		
		// TIMING
		BigDecimal interval = new BigDecimal(String.valueOf(time_slice));
		BigDecimal time = new BigDecimal("0.0");
		
		
		
		// A8 ENGINES
		// SPIN MOTOR 1
		Motor m1 = new Motor();
		m1.setLen(0.07);
		m1.setMass(0.05);
		m1.setPeak_thrust(10);
		m1.setPeek_thrust_start_time(0.2);
		m1.setPeek_thrust_end_time(0.24);
		m1.setNorm_thrust(2);
		m1.setNorm_thrust_start_time(0.28);
		m1.setNorm_thrust_end_time(0.675);
		m1.setIgnition_delay(0.0);

		// SPIN MOTOR 2
		Motor m2 = new Motor();
		m2.setLen(0.07);
		m2.setMass(0.05);
		m2.setPeak_thrust(10);
		m2.setPeek_thrust_start_time(0.2);
		m2.setPeek_thrust_end_time(0.24);
		m2.setNorm_thrust(2);
		m2.setNorm_thrust_start_time(0.28);
		m2.setNorm_thrust_end_time(0.675);
		m2.setIgnition_delay(0.1);  //0.2
		
		
		/*
		// B4 ENGINES
		// SPIN MOTOR 1
		Motor m1 = new Motor();
		m1.setLen(0.07);
		m1.setMass(0.05);
		m1.setPeak_thrust(12);
		m1.setPeek_thrust_start_time(0.11);
		m1.setPeek_thrust_end_time(0.12);
		m1.setNorm_thrust(3);
		m1.setNorm_thrust_start_time(0.28);
		m1.setNorm_thrust_end_time(1);
		m1.setIgnition_delay(0.0);

		// SPIN MOTOR 2
		Motor m2 = new Motor();
		m2.setLen(0.07);
		m2.setMass(0.05);
		m2.setPeak_thrust(12);
		m2.setPeek_thrust_start_time(0.11);
		m2.setPeek_thrust_end_time(0.12);
		m2.setNorm_thrust(3);
		m2.setNorm_thrust_start_time(0.28);
		m2.setNorm_thrust_end_time(1);
		m2.setIgnition_delay(0.1);  //0.2
		*/
				

		/*
		// D12 ENGINES		 
		// SPIN MOTOR 1
		Motor m1 = new Motor();
		m1.setLen(0.07);
		m1.setMass(0.07);
		m1.setPeak_thrust(10);
		m1.setPeek_thrust_start_time(0.3);
		m1.setPeek_thrust_end_time(0.35);
		m1.setNorm_thrust(10);
		m1.setNorm_thrust_start_time(0.45);
		m1.setNorm_thrust_end_time(1.65);
		m1.setIgnition_delay(0.0);

		// SPIN MOTOR 2
		Motor m2 = new Motor();
		m2.setLen(0.07);
		m2.setMass(0.07);
		m2.setPeak_thrust(10);
		m2.setPeek_thrust_start_time(0.3);
		m2.setPeek_thrust_end_time(0.35);
		m2.setNorm_thrust(10);
		m2.setNorm_thrust_start_time(0.45);
		m2.setNorm_thrust_end_time(1.65);
		m2.setIgnition_delay(0.0);  //0.2
*/
		
		
		
		// ROCKET
		Rocket r = new Rocket();
		r.setMass(1);
		r.setLen(2);
		r.setBalance_mass(0.00001);    // 0.5
		r.setBalance_mass_distance(-1);     // Distance from CG along y-axis
		r.setPosition(0, .50, 0);
		r.setRadii(0.05,  0.046);          // 0.023, 0.025
		r.setOrientation(0, 0, 0);         // i.e. upright. Pretend we have IMU on top, flat. (Though axis of IMU would need to be
		r.setRotationSpeed(0, 0, 0);
		r.setSpinRadius(0.015);
		r.setSpinLength(0.1);
		
		// MAIN Rocket engine
		r.setEngine_mass(0);   // 1.75
		r.setEngine_len(0.4);
		r.setEngine_radius(0.029);	
		
		// Derive other properties of rocket
		r.computeCg();
		r.computeInertias();	
		
		
		
		// Forces - simple constant force over whole of the time.
		double[] cg = {r.getCgx(), r.getCgy(), r.getCgz()};
		
		double angle_deviation  =   2;
		double angle_deviation1 =  -2;
		double angle_deviation2 =  -2;
		double angle_deviation3 =  -2;
		
		// SPIN1 FORCES
		double[] spin1_force_angles = {Math.PI * angle_deviation1/180, Math.PI * angle_deviation2/180, Math.PI * angle_deviation3/180 + -Math.PI/2};
		r.setSpin1_rotation_matrix(utils.createRotationMatrix(spin1_force_angles[0], spin1_force_angles[1], spin1_force_angles[2]));
		double[] s1_force = {0, 1, 0};
		RealVector spin1_force_vector_raw = MatrixUtils.createRealVector(s1_force);
		RealVector spin1_force_vector = utils.matrixVectorMultiply(r.getSpin1_rotation_matrix(),  spin1_force_vector_raw);
		System.out.println("Spin1 Force Vector is:   "  + spin1_force_vector.getEntry(0) + ", " + spin1_force_vector.getEntry(1) + ", " + spin1_force_vector.getEntry(2));			

		
		
		// SPIN2 FORCES
		double[] spin2_force_angles = {Math.PI * angle_deviation/180, Math.PI * angle_deviation/180, Math.PI * angle_deviation/180 + Math.PI/2};
		r.setSpin2_rotation_matrix(utils.createRotationMatrix(spin2_force_angles[0], spin2_force_angles[1], spin2_force_angles[2]));
		double[] s2_force = {0, 1, 0};
		RealVector spin2_force_vector_raw = MatrixUtils.createRealVector(s2_force);
		RealVector spin2_force_vector = utils.matrixVectorMultiply(r.getSpin2_rotation_matrix(),  spin2_force_vector_raw);
		System.out.println("Spin2 Force Vector is:   "  + spin2_force_vector.getEntry(0) + ", " + spin2_force_vector.getEntry(1) + ", " + spin2_force_vector.getEntry(2));
		
		
		// POSITION OF SPIN ROCKETS MOTORS
		double spin1_length_deviation =  0.002;
		double spin2_length_deviation =  0.002;
		double spin_dist = r.getRadius_external() + r.getSpinRadius();
		spin_dist = 0.04;
		double[] spin1_pos_data   = {   0, spin1_length_deviation + r.getCgy(),  spin_dist };		
		double[] spin2_pos_data   = {   0, r.getCgy() + spin2_length_deviation, -spin_dist };
		
		r.setSpin1_pos(spin1_pos_data);
		r.setSpin2_pos(spin2_pos_data);
		RealVector spin1_pos_vector   = MatrixUtils.createRealVector(spin1_pos_data);
		RealVector spin2_pos_vector   = MatrixUtils.createRealVector(spin2_pos_data);
		
		
		
		// Initialise 3D Canvas
		threedCanvas canvas = new threedCanvas(rocket_system, r);
		new MainFrame(canvas, 1280, 1024);
		
		
		// Create Vectors of CG...to be used later			
		RealVector cg_vector          = MatrixUtils.createRealVector(cg);
		

		
		// Print out properties of rocket
		System.out.println("Mass:              " + r.getMass());
		System.out.println("Length:            " + r.getLen());
		System.out.println("External Radius:   " + r.getRadius_external());
		System.out.println("Interal Radius:    " + r.getRadius_internal());
		System.out.println("Centre of Gravity: " + r.getCgx() + ", " + r.getCgy() + ", " + r.getCgz());


		
		System.out.println("Starting Iterations...");

		for (int n = 0; n < num_intervals; n++) {
			
			time = time.add(interval);
			// System.out.println("Interval" + n + ", Time = "+ time.toString());
			
			
			double angle_x, angle_y, angle_z;
			angle_x  = r.getRoll();
			angle_y  = r.getPitch();
			angle_z  = r.getYaw();
			
			// CONVERT ALL VECTORS INTO GLOBAL REFERENCE FRAME
			// Derive correct CG location in Global system.
			// RealVector cg_vector_global = utils.revolveVector(r.getRoll(), r.getPitch(), r.getYaw(), cg_vector);		
            RealVector cg_vector_global = utils.revolveVector(angle_x, angle_y, angle_z, cg_vector);
            utils.debug("CG Vector Local  is:   "  + cg_vector.getEntry(0) + ", " + cg_vector.getEntry(1) + ", " + cg_vector.getEntry(2));			
            utils.debug("CG Vector Global is:   "  + cg_vector_global.getEntry(0) + ", " + cg_vector_global.getEntry(1) + ", " + cg_vector_global.getEntry(2));
			
			
			// GET THRUST and given time
			Double thrust1 = m1.getThrust(time.doubleValue());
			Double thrust2 = m2.getThrust(time.doubleValue());
			System.out.println("Spin Thrust1 is:   " + thrust1);
			System.out.println("Spin Thrust2 is:   " + thrust2);
			
			
			// SPIN1 MOTOR			
			// Derive correct Spin1 Pos Vector location in Global system.
			// RealVector spin1_pos_vector_global = utils.revolveVector(r.getRoll(), r.getPitch(), r.getYaw(), spin1_pos_vector);		
            RealVector spin1_pos_vector_global = utils.revolveVector(angle_x, angle_y, angle_z, spin1_pos_vector);
            utils.debug("Spin1 Pos Local  is:   "  + spin1_pos_vector.getEntry(0) + ", " + spin1_pos_vector.getEntry(1) + ", " + spin1_pos_vector.getEntry(2));
            utils.debug("Spin1 Pos Global is:   "  + spin1_pos_vector_global.getEntry(0) + ", " + spin1_pos_vector_global.getEntry(1) + ", " + spin1_pos_vector_global.getEntry(2));
			
			
			// RealVector spin1_force_vector_global = utils.revolveVector(r.getRoll(), r.getPitch(), r.getYaw(), spin1_force_vector);
			double[] spin1_force_vector_tmp_data =  {thrust1 * spin1_force_vector.getEntry(0), thrust1 * spin1_force_vector.getEntry(1), thrust1 * spin1_force_vector.getEntry(2)};
			RealVector spin1_force_vector_tmp = MatrixUtils.createRealVector(spin1_force_vector_tmp_data);
            RealVector spin1_force_vector_global = utils.revolveVector(angle_x, angle_y, angle_z, spin1_force_vector_tmp);
            //System.out.println("Spin1 Force Vector Local  is:   "  + spin1_force_vector.getEntry(0) + ", " + spin1_force_vector.getEntry(1) + ", " + spin1_force_vector.getEntry(2));
            //System.out.println("Ax, Ay, Az = " + angle_x + ", " + angle_y + ", " + angle_z);
            utils.debug("Spin1 Force Vector Global is:   "  + spin1_force_vector_global.getEntry(0) + ", " + spin1_force_vector_global.getEntry(1) + ", " + spin1_force_vector_global.getEntry(2));			
			
			

			// Deduce the distance vector (from the point of force from Spin1) relative to CG
			RealVector spin1_distance_vector_global = spin1_pos_vector_global.subtract(cg_vector_global);
			utils.debug("Spin1 Distance Vector is:  "  + spin1_distance_vector_global.getEntry(0) + ", " + spin1_distance_vector_global.getEntry(1) + ", " + spin1_distance_vector_global.getEntry(2));
			
			
			// Compute the Torque from Spin1 
			RealVector spin1_torque_vector = utils.crossProduct(spin1_force_vector_global, spin1_distance_vector_global);
			utils.debug("Spin1 Torque Vector is:  "  + spin1_torque_vector.getEntry(0) + ", " + spin1_torque_vector.getEntry(1) + ", " + spin1_torque_vector.getEntry(2));
			
			
			
			
			
			
			// SPIN2 MOTOR			
			// Dervive correct Spin1 Pos Vector location in Global system.
			// RealVector spin1_pos_vector_global = utils.revolveVector(r.getRoll(), r.getPitch(), r.getYaw(), spin1_pos_vector);		
            RealVector spin2_pos_vector_global = utils.revolveVector(angle_x, angle_y, angle_z, spin2_pos_vector);
            utils.debug("Spin2 Pos Local  is:   "  + spin2_pos_vector.getEntry(0) + ", " + spin2_pos_vector.getEntry(1) + ", " + spin2_pos_vector.getEntry(2));			
            utils.debug("Spin2 Pos Global is:   "  + spin2_pos_vector_global.getEntry(0) + ", " + spin2_pos_vector_global.getEntry(1) + ", " + spin2_pos_vector_global.getEntry(2));
			
			
			// RealVector spin1_force_vector_global = utils.revolveVector(r.getRoll(), r.getPitch(), r.getYaw(), spin1_force_vector);
			double[] spin2_force_vector_tmp_data =  {thrust2 * spin2_force_vector.getEntry(0), thrust2 * spin2_force_vector.getEntry(1), thrust2 * spin2_force_vector.getEntry(2)};
			RealVector spin2_force_vector_tmp = MatrixUtils.createRealVector(spin2_force_vector_tmp_data);			
            RealVector spin2_force_vector_global = utils.revolveVector(angle_x, angle_y, angle_z, spin2_force_vector_tmp);
            utils.debug("Spin2 Force Vector Local  is:   "  + spin2_force_vector.getEntry(0) + ", " + spin2_force_vector.getEntry(1) + ", " + spin2_force_vector.getEntry(2));			
            utils.debug("Spin2 Force Vector Global is:   "  + spin2_force_vector_global.getEntry(0) + ", " + spin2_force_vector_global.getEntry(1) + ", " + spin2_force_vector_global.getEntry(2));			
			

			// Deduce the distance vector (from the point of force from Spin1) relative to CG
			RealVector spin2_distance_vector_global = spin2_pos_vector_global.subtract(cg_vector_global);
			utils.debug("Spin2 Distance Vector is:  "  + spin2_distance_vector_global.getEntry(0) + ", " + spin2_distance_vector_global.getEntry(1) + ", " + spin2_distance_vector_global.getEntry(2));
			
			
			// Compute the Torque from Spin2 
			RealVector spin2_torque_vector = utils.crossProduct(spin2_force_vector_global, spin2_distance_vector_global);
			utils.debug("Spin2 Torque Vector is:  "  + spin2_torque_vector.getEntry(0) + ", " + spin2_torque_vector.getEntry(1) + ", " + spin2_torque_vector.getEntry(2));
			
			
			
			// COMBINE THE TWO TORQUES
			RealVector total_torque_vector = spin1_torque_vector.add(spin2_torque_vector);
			utils.debug("Resultant Torque: " + total_torque_vector.getEntry(0) + ", " + total_torque_vector.getEntry(1) + ", " + total_torque_vector.getEntry(2));
			
			
			// COMBINE THE FORCES
			RealVector total_force = spin1_force_vector_global.add(spin2_force_vector_global);
			System.out.println("Resultant Force: " + total_force.getEntry(0) + ", " + total_force.getEntry(1) + ", " + total_force.getEntry(2));
			
			
			// INERTIA CALCULATIONS
			utils.debug("BR Inertia ROW 1:  "  + r.getInertia().getEntry(0, 0) + ", " + r.getInertia().getEntry(0,1) + ", " + r.getInertia().getEntry(0,2));
			utils.debug("BR Inertia ROW 2:  "  + r.getInertia().getEntry(1, 0) + ", " + r.getInertia().getEntry(1,1) + ", " + r.getInertia().getEntry(1,2));			
			utils.debug("BR Inertia ROW 3:  "  + r.getInertia().getEntry(2, 0) + ", " + r.getInertia().getEntry(2,1) + ", " + r.getInertia().getEntry(2,2));						
			
			// Get the rotation matrix
			RealMatrix rotation_matrix = r.getRotationMatrix();
			
			utils.debug("Rotation Matrix ROW 1:  "  + rotation_matrix.getEntry(0, 0) + ", " + rotation_matrix.getEntry(0,1) + ", " + rotation_matrix.getEntry(0,2));
			utils.debug("Rotation Matrix ROW 2:  "  + rotation_matrix.getEntry(1, 0) + ", " + rotation_matrix.getEntry(1,1) + ", " + rotation_matrix.getEntry(1,2));			
			utils.debug("Rotation Matrix ROW 3:  "  + rotation_matrix.getEntry(2, 0) + ", " + rotation_matrix.getEntry(2,1) + ", " + rotation_matrix.getEntry(2,2));			
			
			RealMatrix rotation_matrix_inverse = new LUDecomposition(rotation_matrix).getSolver().getInverse();
			RealMatrix rotation_matrix_inverse_transpose = rotation_matrix_inverse.transpose();
			
			// Compute new Inertia Matrix (in Global Reference System)
			RealMatrix inertia_global = rotation_matrix_inverse_transpose.multiply(r.getInertia()).multiply(rotation_matrix_inverse);
			System.out.println("Inertia ROW 1:  "  + inertia_global.getEntry(0, 0) + ", " + inertia_global.getEntry(0,1) + ", " + inertia_global.getEntry(0,2));
			System.out.println("Inertia ROW 2:  "  + inertia_global.getEntry(1, 0) + ", " + inertia_global.getEntry(1,1) + ", " + inertia_global.getEntry(1,2));			
			System.out.println("Inertia ROW 3:  "  + inertia_global.getEntry(2, 0) + ", " + inertia_global.getEntry(2,1) + ", " + inertia_global.getEntry(2,2));			

			
		
			RealMatrix inertia_global_inverse = new LUDecomposition(inertia_global).getSolver().getInverse();
			
			// Calculate the Angular Acceleration Vector
			RealVector rotation_acceleration_vector_global =  utils.matrixVectorMultiply(inertia_global_inverse, total_torque_vector);
			System.out.println("Time: " + time.toString());
			System.out.println("Torque:                  "  + total_torque_vector.getEntry(0) + ", " + total_torque_vector.getEntry(1) + ", " + total_torque_vector.getEntry(2));
			System.out.println("Rotation Acceleration:   "  + rotation_acceleration_vector_global.getEntry(0) + ", " + rotation_acceleration_vector_global.getEntry(1) + ", " + rotation_acceleration_vector_global.getEntry(2));
			System.out.println("Rotation Velocity:       "  + 180 * r.getWx()/Math.PI + ", " + 180 * r.getWy()/Math.PI + ", " + 180 * r.getWz()/Math.PI + " degrees/sec");
			System.out.println("Rotation Orientation:    "  + 180 * r.getRoll()/Math.PI + ", " + 180 * r.getPitch()/Math.PI + ", " + 180 * r.getYaw()/Math.PI + " degrees");
			System.out.println("                         "  + r.getWx()/2/Math.PI + ", " + r.getWy()/2/Math.PI + ", " + r.getWz()/2/Math.PI + " Revs per Second");
			
			
			System.out.println("Translation Velocity:    "  + r.getVelocity_x() + ", " + r.getVelocity_y() + ", " + r.getVelocity_z());
			System.out.println("Translation orientation: "  + r.getPosition_x() * 1000 + ", " + r.getPosition_y() * 1000 + ", " + r.getPosition_z() * 1000 + " mm");
			
			double x_distance = 1000 * Math.sin(r.getYaw())  * (r.getLen() - r.getCgy());
			double z_distance = 1000 * Math.sin(r.getRoll()) * (r.getLen() - r.getCgy());
			double total_distance = Math.sqrt(Math.pow(x_distance,2) + Math.pow(z_distance,  2));
			System.out.println("Precession Distance:   " + total_distance + " mm");
			
			
/*			if (total_distance > distance_test ) {
				distance_test++;
				try {
					TimeUnit.SECONDS.sleep(5);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}*/
			
			
			r.updateState(rotation_acceleration_vector_global, total_force, time_slice);
			//trans.rotX(r.getRoll());
			//trans.rotY(r.getPitch());
			//trans.rotZ(r.getYaw());
			RealMatrix rotationMatrix = utils.createRotationMatrix(r.getRoll(), r.getPitch(), r.getYaw());
			Matrix3d matrix3d = new Matrix3d();
			matrix3d.setColumn(0,  rotationMatrix.getColumn(0));
			matrix3d.setColumn(1,  rotationMatrix.getColumn(1));
			matrix3d.setColumn(2,  rotationMatrix.getColumn(2));
			trans.setRotation(matrix3d);
				
			
		 	Transform3D translation = new Transform3D();
		 	Vector3f translation_vector = new Vector3f((float) r.getPosition_x(), (float) r.getPosition_y(), (float) r.getPosition_z());
		 	translation.setTranslation(translation_vector);
		 	
		 	translation.mul(trans);
		 	   
			// trans.setTranslation(new Vector3f(x_pos, 0f, 0f));
			rocket_system.setTransform(translation);
			// x_pos = (float) (x_pos + 0.001);		 	
			
			
			try {
				TimeUnit.MILLISECONDS.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			// Exit if time > 0 and no more thrust.
			if (time.doubleValue() > 0 && thrust1 == 0 && thrust2 == 0) {
				break;
			}
			
			
			
			System.out.println("");
			
		}
		
		System.out.println("Finished Iterations.");
		
	}
	
	
}
