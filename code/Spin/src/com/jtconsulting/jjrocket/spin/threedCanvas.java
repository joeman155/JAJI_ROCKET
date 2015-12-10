package com.jtconsulting.jjrocket.spin;
import com.sun.j3d.utils.universe.SimpleUniverse;
import com.sun.j3d.utils.universe.ViewingPlatform;
import com.sun.j3d.utils.geometry.ColorCube;
import com.sun.j3d.utils.geometry.Cylinder;

import javax.media.j3d.Appearance;
import javax.media.j3d.BoundingSphere;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.Canvas3D;
import javax.media.j3d.DirectionalLight;
import javax.media.j3d.LineArray;
import javax.media.j3d.Material;
import javax.media.j3d.Shape3D;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.commons.math3.linear.RealMatrix;

import java.awt.GraphicsConfiguration;
import java.awt.BorderLayout;
import java.awt.Label;
import java.applet.Applet;
import com.sun.j3d.utils.applet.MainFrame;

public class threedCanvas extends Applet {

	
	
    /**
     * 
     */
    public threedCanvas(TransformGroup tg_parent, Rocket r)  { 
        
        setLayout(new BorderLayout());
        GraphicsConfiguration config = 
            SimpleUniverse.getPreferredConfiguration();
        Canvas3D canvas = new Canvas3D(config);  
        add("North",new Label("This is the top"));
        add("Center", canvas);
        add("South",new Label("This is the bottom"));
        
        /*
        BranchGroup contents = new BranchGroup();
        contents.addChild(new ColorCube(0.3));
        SimpleUniverse universe = new SimpleUniverse(canvas);
        universe.getViewingPlatform().setNominalViewingTransform();
        universe.addBranchGraph(contents);
        */
        
        
 	   // INIT
 	   SimpleUniverse universe = new SimpleUniverse(canvas);
 	   BranchGroup group = new BranchGroup();
 	   TransformGroup axis = new TransformGroup();
 	   TransformGroup tg_top = new TransformGroup();
 	   
 	   
 	   // LIGHT
        DirectionalLight light = new DirectionalLight( true,
                new Color3f(1.0f, 1.0f, 1.0f),
                new Vector3f(-2.0f, 2.0f, -2.0f));
        light.setInfluencingBounds(new BoundingSphere(new Point3d(), 100.0));	          
        group.addChild(light);
        


 	   // MATERIAL - Rocket
       Appearance ap = new Appearance();
       Material ma = new Material();
       ma.setDiffuseColor(0.0f, 0.0f, 1.0f);
       ap.setMaterial(ma);
       
        
 	   // MATERIAL - Spinner rockets
       Appearance ap_sp1 = new Appearance();
       Material ma_sp1 = new Material();
       ma_sp1.setDiffuseColor(1.0f, 0.0f, 0.0f);
       ap_sp1.setMaterial(ma_sp1);     
       
       
 	   // MATERIAL - AXIS
       Appearance ap_axis = new Appearance();
       Material ma_axis = new Material();
       ma_axis.setDiffuseColor(0.0f, 1.0f, 0.0f);
       ap_axis.setMaterial(ma_axis);   
       
        
       // All rockets
 	   Cylinder rocket =  new Cylinder((float) r.getRadius_external(), (float) r.getLen(), ap);
 	   Cylinder rocket_spin1 =  new Cylinder((float) r.getSpinRadius(), (float) r.getSpinLength(), ap_sp1);
 	   Cylinder rocket_spin2 =  new Cylinder((float) r.getSpinRadius(), (float) r.getSpinLength(), ap_sp1);

 	   
 	   
 	   // ENTIRE TRANSFORM GROUP
 	   //TransformGroup tg_parent = new TransformGroup();
 	   tg_parent.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
 	   
 	   
 	   // Move the tg_parent, dependng upon where the rocket is (translation)
 	   Transform3D translation = new Transform3D();
 	   Vector3f translation_vector = new Vector3f((float) r.getPosition_x(), (float) r.getPosition_y(), (float) r.getPosition_z());
 	   translation.setTranslation(translation_vector);
 	   tg_top.setTransform(translation);
 	    	   
 	   
 	   // TRANSFORM SPIN 1
 	   Transform3D transform_spin1 = new Transform3D();
 	   TransformGroup tg_spin1 = new TransformGroup();
 	   Vector3f vector_spin1 = new Vector3f( (float) (r.getSpin1_pos()[0]), (float) (r.getSpin1_pos()[1] - r.getLen()/2), (float) (r.getSpin1_pos()[2]));
 	   transform_spin1.setTranslation(vector_spin1);
 	   Transform3D rotation_spin1 = new Transform3D();
 	   // rotation_spin1.rotZ(Math.PI/2);
 	   
	   Matrix3d matrix3d = new Matrix3d();
	   matrix3d.setColumn(0,  r.getSpin1_rotation_matrix().getColumn(0));
	   matrix3d.setColumn(1,  r.getSpin1_rotation_matrix().getColumn(1));
	   matrix3d.setColumn(2,  r.getSpin1_rotation_matrix().getColumn(2));
 	   rotation_spin1.setRotation(matrix3d);
 	   
	   transform_spin1.mul(rotation_spin1);
	   tg_spin1.setTransform(transform_spin1);

	   
	   tg_spin1.addChild(rocket_spin1);
 	   
 	   // TRANSFORM SPIN 2
 	   Transform3D transform_spin2 = new Transform3D();
 	   TransformGroup tg_spin2 = new TransformGroup();
 	   Vector3f vector_spin2 = new Vector3f((float) (r.getSpin2_pos()[0]), (float) (r.getSpin2_pos()[1] - r.getLen()/2), (float) (r.getSpin2_pos()[2]));
 	   transform_spin2.setTranslation(vector_spin2);
 	   Transform3D rotation_spin2 = new Transform3D();
 	   // rotation_spin2.rotZ(Math.PI/2);
 	   
	   Matrix3d matrix3d2 = new Matrix3d();
	   matrix3d2.setColumn(0,  r.getSpin2_rotation_matrix().getColumn(0));
	   matrix3d2.setColumn(1,  r.getSpin2_rotation_matrix().getColumn(1));
	   matrix3d2.setColumn(2,  r.getSpin2_rotation_matrix().getColumn(2));
 	   rotation_spin2.setRotation(matrix3d2);
 	   
 	   
	   transform_spin2.mul(rotation_spin2);
	   tg_spin2.setTransform(transform_spin2);
 	   
 	   
 	   tg_spin2.addChild(rocket_spin2);
 	    	   
 	   
 	   
 	   // X, Y, Z acis
 	   /*
 	   LineArray axisXLines=new LineArray(2,LineArray.COORDINATES);
 	   group.addChild(new Shape3D(axisXLines));
 	   axisXLines.setCoordinate(0,  new Point3f(-10.0f, 0.0f, 0.0f));
 	   axisXLines.setCoordinate(0,  new Point3f(10.0f, 0.0f, 0.0f));
 	  
 	   
 	   LineArray axisYLines=new LineArray(2,LineArray.COORDINATES);
 	   group.addChild(new Shape3D(axisYLines));
 	   axisYLines.setCoordinate(0,new Point3f(0.0f,-10.0f,0.0f));
 	   axisYLines.setCoordinate(1,new Point3f(0.0f,10.0f,0.0f));
 	   */    
 	  
       // Y-Axis
 	   Cylinder yaxis =  new Cylinder((float) 0.005, (float) 5, ap_axis);
 	   axis.addChild(yaxis);
 	   
 	   // Y-Axis
 	   Cylinder xaxis =  new Cylinder((float) 0.005, (float) 5, ap_axis);
 	   TransformGroup xaxis_tg = new TransformGroup();
 	   Transform3D xaxis_rotation = new Transform3D();
 	   xaxis_rotation.rotZ(Math.PI/2);
 	   xaxis_tg.setTransform(xaxis_rotation);
 	   xaxis_tg.addChild(xaxis);
 	   axis.addChild(xaxis_tg); 	   

 	   // Z-Axis
 	   Cylinder zaxis =  new Cylinder((float) 0.005, (float) 15, ap_axis);
 	   TransformGroup zaxis_tg = new TransformGroup();
 	   Transform3D zaxis_rotation = new Transform3D();
 	   zaxis_rotation.rotX(Math.PI/2);
 	   zaxis_tg.setTransform(zaxis_rotation);
 	   zaxis_tg.addChild(zaxis);
 	   axis.addChild(zaxis_tg); 	
 	   
 	   
 	   
 	   // Combining all Transform Groups
 	   tg_parent.addChild(tg_top);
 	   tg_top.addChild(tg_spin1);
  	   tg_top.addChild(tg_spin2);
 	   tg_top.addChild(rocket);
 	   group.addChild(tg_parent);
 	   group.addChild(axis);
 	   group.compile();
 	   
 	   
       // Transform the View Platform
	   Transform3D viewTransform = new Transform3D();
	   Transform3D viewRotation1 = new Transform3D();
	   Transform3D viewRotation2 = new Transform3D();
	   viewRotation1.rotY(0.5f);
	   viewRotation2.rotX(-0.2f);
	   Vector3f viewTranslation = new Vector3f(0f, (float) (0.1 * r.getLen()), (float) (3 * r.getLen()));
	   viewTransform.setTranslation(viewTranslation);
	   
	   /*
	   viewRotation1.mul(viewRotation2);
	   viewTransform.mul(viewRotation1);
	   universe.getViewingPlatform().getViewPlatformTransform().setTransform(viewTransform);
	   */
	   viewRotation1.mul(viewRotation2);
	   viewRotation1.mul(viewTransform);
	   universe.getViewingPlatform().getViewPlatformTransform().setTransform(viewRotation1);	   

	   // universe.getViewingPlatform().setNominalViewingTransform();

 	   universe.addBranchGraph(group);
 	   
    }
}
