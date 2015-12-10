package com.jtconsulting.jjrocket.spin;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * @author joeman
 *
 */
public class Rocket {
	private double engine_mass;
	private double engine_len;
	private double engine_radius;
	private double balance_mass;
	private double balance_mass_distance;
	private double mass;
	private double len;
	private double radius_internal;
	private double radius_external;
	private double position_x, position_y, position_z;
	private double yaw, pitch, roll;                   // Angles around X, Y and Z axis. We use Tai
	private double velocity_x, velocity_y, velocity_z;
	private double wx, wy, wz;
	private double cgx, cgy, cgz;
	private RealMatrix inertia;
	private double[] spin1_pos;
	private double[] spin2_pos;
	private double spinRadius;    // Radius of the Spin rockets (used to ensure mounted at correct distance from rocket axis)
	private double spinLength;    // Used to show it's proper length in the animation
    private double spin1_force;   // Not used at present
    private double spin2_force;   // Not used at present
    private RealMatrix spin1_rotation_matrix; // The matrix used to rotate the spin1 rocket engine into position
    private RealMatrix spin2_rotation_matrix; // The matrix used to rotate the spin2 rocket engine into position
	
	
    
	

	/**
	 * @return the balance_mass
	 */
	public final double getBalance_mass() {
		return balance_mass;
	}
	/**
	 * @param balance_mass the balance_mass to set
	 */
	public final void setBalance_mass(double balance_mass) {
		this.balance_mass = balance_mass;
	}
	/**
	 * @return the balance_mass_distance
	 */
	public final double getBalance_mass_distance() {
		return balance_mass_distance;
	}
	/**
	 * @param balance_mass_distance the balance_mass_distance to set
	 */
	public final void setBalance_mass_distance(double balance_mass_distance) {
		this.balance_mass_distance = balance_mass_distance;
	}
	/**
	 * @return the spin1_rotation_matrix
	 */
	public final RealMatrix getSpin1_rotation_matrix() {
		return spin1_rotation_matrix;
	}
	/**
	 * @param spin1_rotation_matrix the spin1_rotation_matrix to set
	 */
	public final void setSpin1_rotation_matrix(RealMatrix spin1_rotation_matrix) {
		this.spin1_rotation_matrix = spin1_rotation_matrix;
	}
	/**
	 * @return the spin2_rotation_matrix
	 */
	public final RealMatrix getSpin2_rotation_matrix() {
		return spin2_rotation_matrix;
	}
	/**
	 * @param spin2_rotation_matrix the spin2_rotation_matrix to set
	 */
	public final void setSpin2_rotation_matrix(RealMatrix spin2_rotation_matrix) {
		this.spin2_rotation_matrix = spin2_rotation_matrix;
	}
	/**
	 * @return the spin1_force
	 */
	public final double getSpin1_force() {
		return spin1_force;
	}
	/**
	 * @param spin1_force the spin1_force to set
	 */
	public final void setSpin1_force(double spin1_force) {
		this.spin1_force = spin1_force;
	}
	/**
	 * @return the spin2_force
	 */
	public final double getSpin2_force() {
		return spin2_force;
	}
	/**
	 * @param spin2_force the spin2_force to set
	 */
	public final void setSpin2_force(double spin2_force) {
		this.spin2_force = spin2_force;
	}
	/**
	 * @return the spinRadius
	 */
	public final double getSpinRadius() {
		return spinRadius;
	}
	/**
	 * @param d the spinRadius to set
	 */
	public final void setSpinRadius(double d) {
		this.spinRadius = d;
	}
	/**
	 * @return the spinLength
	 */
	public final double getSpinLength() {
		return spinLength;
	}
	/**
	 * @param spinLength the spinLength to set
	 */
	public final void setSpinLength(double spinLength) {
		this.spinLength = spinLength;
	}
	/**
	 * @return the spin1_pos
	 */
	public final double[] getSpin1_pos() {
		return spin1_pos;
	}
	/**
	 * @param spin1_pos the spin1_pos to set
	 */
	public final void setSpin1_pos(double[] spin1_pos) {
		this.spin1_pos = spin1_pos;
	}
	/**
	 * @return the spin2_pos
	 */
	public final double[] getSpin2_pos() {
		return spin2_pos;
	}
	/**
	 * @param spin2_pos the spin2_pos to set
	 */
	public final void setSpin2_pos(double[] spin2_pos) {
		this.spin2_pos = spin2_pos;
	}
	/**
	 * @return the mass
	 */
	public final double getMass() {
		return mass;
	}
	/**
	 * @param mass the mass to set
	 */
	public final void setMass(double mass) {
		this.mass = mass;
	}
	/**
	 * @return the len
	 */
	public final double getLen() {
		return len;
	}
	/**
	 * @param len the len to set
	 */
	public final void setLen(double len) {
		this.len = len;
	}
	/**
	 * @return the ix
	 */

	/**
	 * @return the radius_internal
	 */
	public final double getRadius_internal() {
		return radius_internal;
	}
	/**
	 * @param radius_internal the radius_internal to set
	 */
	private final void setRadius_internal(double radius_internal) {
		this.radius_internal = radius_internal;
	}
	/**
	 * @return the radius_external
	 */
	public final double getRadius_external() {
		return radius_external;
	}
	/**
	 * @param radius_external the radius_external to set
	 */
	private final void setRadius_external(double radius_external) {
		this.radius_external = radius_external;
	}
	/**
	 * @return the position_x
	 */
	public final double getPosition_x() {
		return position_x;
	}
	/**
	 * @param position_x the position_x to set
	 */
	public final void setPosition_x(double position_x) {
		this.position_x = position_x;
	}
	/**
	 * @return the position_y
	 */
	public final double getPosition_y() {
		return position_y;
	}
	/**
	 * @param position_y the position_y to set
	 */
	public final void setPosition_y(double position_y) {
		this.position_y = position_y;
	}
	/**
	 * @return the position_z
	 */
	public final double getPosition_z() {
		return position_z;
	}
	/**
	 * @param position_z the position_z to set
	 */
	public final void setPosition_z(double position_z) {
		this.position_z = position_z;
	}
	
	
	/**
	 * @param x
	 * @param y
	 * @param z
	 */
	public final void setPosition(double x, double y, double z) {
		this.setPosition_x(x);
		this.setPosition_y(y);
		this.setPosition_z(z);
	}
	/**
	 * @return the yaw
	 */
	public final double getYaw() {
		return yaw;
	}
	/**
	 * @param yaw the yaw to set
	 */
	private final void setYaw(double yaw) {
		this.yaw = yaw;
	}
	/**
	 * @return the pitch
	 */
	public final double getPitch() {
		return pitch;
	}
	/**
	 * @param pitch the pitch to set
	 */
	private final void setPitch(double pitch) {
		this.pitch = pitch;
	}
	/**
	 * @return the roll
	 */
	public final double getRoll() {
		return roll;
	}
	/**
	 * @param roll the roll to set
	 */
	private final void setRoll(double roll) {
		this.roll = roll;
	}
	/**
	 * @return the velocity_x
	 */
	public final double getVelocity_x() {
		return velocity_x;
	}
	/**
	 * @param velocity_x the velocity_x to set
	 */
	public final void setVelocity_x(double velocity_x) {
		this.velocity_x = velocity_x;
	}
	/**
	 * @return the velocity_y
	 */
	public final double getVelocity_y() {
		return velocity_y;
	}
	/**
	 * @param velocity_y the velocity_y to set
	 */
	public final void setVelocity_y(double velocity_y) {
		this.velocity_y = velocity_y;
	}
	/**
	 * @return the velocity_z
	 */
	public final double getVelocity_z() {
		return velocity_z;
	}
	/**
	 * @param velocity_z the velocity_z to set
	 */
	public final void setVelocity_z(double velocity_z) {
		this.velocity_z = velocity_z;
	}
	
	public final void computeInertias() {
		
		// Rocket Tube
		double ixx =       this.getMass() * (3 * (Math.pow(this.getRadius_external(),2) + Math.pow(this.getRadius_internal(),2)) + Math.pow(this.getLen(),2))/12;
		double iyy = 0.5 * this.getMass() * (Math.pow(this.getRadius_external(),2) + Math.pow(this.getRadius_internal(),2));
		double izz =       this.getMass() * (3 * (Math.pow(this.getRadius_external(),2) + Math.pow(this.getRadius_internal(),2)) + Math.pow(this.getLen(),2))/12;
		double ixy = 0;
		double ixz = 0;
		double iyx = 0;
		double iyz = 0;
		double izx = 0;
		double izy = 0;
		
		
		// Engine (e = Engine)
		double ixxe = this.getEngine_mass() * (3 * Math.pow(this.getEngine_radius(),  3) + Math.pow(this.getEngine_len(),  2))/12;
		double iyye = 0.5 * this.getEngine_mass() * this.getEngine_radius() * this.getEngine_radius();
		double izze = ixxe; 

		double distance_from_cg = this.getCgy();
		ixxe = ixxe + this.getEngine_mass() * Math.pow((distance_from_cg),2);
		izze = izze + this.getEngine_mass() * Math.pow((distance_from_cg),2);
		
		ixx = ixx + ixxe;
		iyy = iyy + iyye;
		izz = izz + izze;
		
		// Balance Weight
		ixx = ixx + this.getBalance_mass() * Math.pow((this.getBalance_mass_distance() + this.getLen()),2);
		izz = izz + this.getBalance_mass() * Math.pow((this.getBalance_mass_distance() + this.getLen()),2);		
		
		double inertia_data[][] = { {ixx, ixy, ixz},
									{iyx, iyy, iyz},
									{izx, izy, izz}
								  };
				
		RealMatrix inertia = MatrixUtils.createRealMatrix(inertia_data);
		this.setInertia(inertia);
				
	}
	
	public final void setRadii(double ri, double re) {
		this.setRadius_internal(ri);
		this.setRadius_external(re);
	}
	
	public final void setCg(double cgx, double cgy, double cgz) {
		this.setCgx(cgx);
		this.setCgy(cgy);
		this.setCgz(cgz);
	}
	/**
	 * @return the cgx
	 */
	public final double getCgx() {
		return cgx;
	}
	/**
	 * @param cgx the cgx to set
	 */
	private final void setCgx(double cgx) {
		this.cgx = cgx;
	}
	/**
	 * @return the cgy
	 */
	public final double getCgy() {
		return cgy;
	}
	/**
	 * @param cgy the cgy to set
	 */
	private final void setCgy(double cgy) {
		this.cgy = cgy;
	}
	/**
	 * @return the cgz
	 */
	public final double getCgz() {
		return cgz;
	}
	/**
	 * @param cgz the cgz to set
	 */
	private final void setCgz(double cgz) {
		this.cgz = cgz;
	}
	
	public final void setOrientation(double roll, double pitch, double yaw) {
		this.setRoll(roll);    // x
		this.setPitch(pitch);  // y
		this.setYaw(yaw);      // z
	}
	/**
	 * @return the wx
	 */
	public final double getWx() {
		return wx;
	}
	/**
	 * @param wx the wx to set
	 */
	public final void setWx(double wx) {
		this.wx = wx;
	}
	/**
	 * @return the wy
	 */
	public final double getWy() {
		return wy;
	}
	/**
	 * @param wy the wy to set
	 */
	public final void setWy(double wy) {
		this.wy = wy;
	}
	/**
	 * @return the wz
	 */
	public final double getWz() {
		return wz;
	}
	/**
	 * @return the inertia
	 */
	public final RealMatrix getInertia() {
		return inertia;
	}
	/**
	 * @param inertia the inertia to set
	 */
	public final void setInertia(RealMatrix inertia) {
		this.inertia = inertia;
	}
	/**
	 * @param wz the wz to set
	 */
	private final void setWz(double wz) {
		this.wz = wz;
	}
	public void setRotationSpeen(double wx, double wy, double wz) {
		this.setWx(wx);
		this.setWy(wy);
		this.setWz(wz);
		
	}
	public void computeNextPosition(double time_slice,
			RealVector angular_acceleration_vector) {
		
		// Compute Acceleration vector in the global co-ordinates system
		
		
		// Computer the change in angular velocity (around CG, in global Reference frame)
		this.setWx(this.getWx() + time_slice * angular_acceleration_vector.getEntry(0));
		this.setWy(this.getWy() + time_slice * angular_acceleration_vector.getEntry(1));
		this.setWz(this.getWz() + time_slice * angular_acceleration_vector.getEntry(2));
		
		
		// Compute the change in angular position (around CG, in Global Reference frame)
		
		
		// Computer the change in velocity of CG in global Reference frame
		
		
		
		// Compute the change in position of CG in global Reference frame
		
	}
	public RealMatrix getRotationMatrix() {
		RealMatrix rotation_matrix;
		double c1, c2, c3;  // Cosine of roll (1), pitch (2), yaw (3)
		double s1, s2, s3;  // Sine of roll (1), pitch (2), yaw (3)
		
		c1 = Math.cos(this.getRoll());
		c2 = Math.cos(this.getPitch());
		c3 = Math.cos(this.getYaw());
		s1 = Math.sin(this.getRoll());
		s2 = Math.sin(this.getPitch());
		s3 = Math.sin(this.getYaw());
		
		double rotation_matrix_data[][] = { 
				{(c3 * c2), (s1 * c3 * s2 - c1 * s3), (s1 * s3 + c1 * c3 * s2)   },
				{(s3 * c2), (c3 * c1 + s3 * s2 * s1), (s3 * s2 * c1 - c3 * s1)   },
				{(-s2)    , (s1 * c2),                (c2 * c1)}
		};
		
		rotation_matrix = MatrixUtils.createRealMatrix(rotation_matrix_data);
		
		return rotation_matrix;
	}
	
	
	public void updateState(RealVector rot_accel, RealVector trans_accel,
			double time_slice) {
		
		double old_wx = this.getWx();
		double old_wy = this.getWy();
		double old_wz = this.getWz();
		
		double old_vx = this.getVelocity_x();
		double old_vy = this.getVelocity_y();
		double old_vz = this.getVelocity_z();
		
		
		// Calculate the change in Angular velocities
		this.setWx(this.getWx() + time_slice * rot_accel.getEntry(0));
		this.setWy(this.getWy() + time_slice * rot_accel.getEntry(1));
		this.setWz(this.getWz() + time_slice * rot_accel.getEntry(2));
		
		double wx_avg = (old_wx + this.getWx())/2;
		double wy_avg = (old_wy + this.getWy())/2;		
		double wz_avg = (old_wz + this.getWz())/2;
		

		
		// Calculate the change in orientation
		this.setRoll(this.getRoll()   + time_slice * wx_avg);
		this.setPitch(this.getPitch() + time_slice * wy_avg);
		this.setYaw(this.getYaw()     + time_slice * wz_avg);
		
		// Calculate the change in velocity
		this.setVelocity_x(this.getVelocity_x() + time_slice * trans_accel.getEntry(0)/(this.getMass() + this.getBalance_mass()));
		this.setVelocity_y(this.getVelocity_y() + time_slice * trans_accel.getEntry(1)/(this.getMass() + this.getBalance_mass()));
		this.setVelocity_z(this.getVelocity_z() + time_slice * trans_accel.getEntry(2)/(this.getMass() + this.getBalance_mass()));
		
		double vx_avg = (old_vx + this.getVelocity_x())/2;
		double vy_avg = (old_vy + this.getVelocity_y())/2;
		double vz_avg = (old_vz + this.getVelocity_z())/2;
		
		// Calculate new position
		this.setPosition_x(this.getPosition_x() + time_slice * vx_avg);
		this.setPosition_y(this.getPosition_y() + time_slice * vy_avg);
		this.setPosition_z(this.getPosition_z() + time_slice * vz_avg);

		
	}
	
	public void setVelocity(double vx, double vy,double vz) {
		this.setVelocity_x(vx);
		this.setVelocity_x(vy);
		this.setVelocity_z(vz);
	}
	public double getEngine_mass() {
		return engine_mass;
	}
	public void setEngine_mass(double engine_mass) {
		this.engine_mass = engine_mass;
	}
	public double getEngine_len() {
		return engine_len;
	}
	public void setEngine_len(double engine_len) {
		this.engine_len = engine_len;
	}
	public double getEngine_radius() {
		return engine_radius;
	}
	public void setEngine_radius(double engine_radius) {
		this.engine_radius = engine_radius;
	}
	public void computeCg() {
		// TODO Auto-generated method stub
		double cy = 0.5 * (this.getLen() * this.getMass() + this.getEngine_len() * this.getEngine_mass());
		cy = cy/(this.getEngine_mass() + this.getMass());
		
		this.setCgx(0);
		this.setCgy(cy);
		this.setCgz(0);		
	}
	
	

}
