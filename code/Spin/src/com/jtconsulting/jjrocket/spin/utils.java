package com.jtconsulting.jjrocket.spin;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class utils {

	
	public static RealMatrix createRotationMatrix(double roll, double pitch, double yaw) {
		// Note, roll is really rotation around X-Axis
		//       pitch is really rotation around Y-Axis
		//       yaw is really rotation around Z-Axis
		double c1, c2, c3;  // Cosine of roll (1), pitch (2), yaw (3)
		double s1, s2, s3;  // Sine of roll (1), pitch (2), yaw (3)
		
		c1 = Math.cos(roll);
		c2 = Math.cos(pitch);
		c3 = Math.cos(yaw);
		s1 = Math.sin(roll);
		s2 = Math.sin(pitch);
		s3 = Math.sin(yaw);
		

		double[][] matrix_data = {
				{c3 * c2,  (s1 * c3 * s2 - c1 * s3), (s1 * s3 + c1 * c3 * s2)},
		        {s3 * c2,  (c3 * c1 + s3 * s2 * s1), (s3 * s2 * c1 - c3 * s1)},
		        {(-s2)  ,  (s1 * c2),                (c2 * c1)}
		        };
		
		RealMatrix matrix = MatrixUtils.createRealMatrix(matrix_data);
		
		return matrix;
	}
	
	public static RealVector revolveVector(double roll, double pitch, double yaw, RealVector vec1) {
	
		double c1, c2, c3;  // Cosine of roll (1), pitch (2), yaw (3)
		double s1, s2, s3;  // Sine of roll (1), pitch (2), yaw (3)
		
		c1 = Math.cos(roll);
		c2 = Math.cos(pitch);
		c3 = Math.cos(yaw);
		s1 = Math.sin(roll);
		s2 = Math.sin(pitch);
		s3 = Math.sin(yaw);
		
		double vec1_x = vec1.getEntry(0);
		double vec1_y = vec1.getEntry(1);
		double vec1_z = vec1.getEntry(2);

		double[] vec2_data = {c3 * c2 * vec1_x + (s1 * c3 * s2 - c1 * s3) * vec1_y + (s1 * s3 + c1 * c3 * s2) * vec1_z,
		             s3 * c2 * vec1_x + (c3 * c1 + s3 * s2 * s1) * vec1_y + (s3 * s2 * c1 - c3 * s1) * vec1_z,
		             (-s2) * vec1_x + s1 * c2 * vec1_y + c2 * c1 * vec1_z};
		
		RealVector vec2 = MatrixUtils.createRealVector(vec2_data);
		
		return vec2;
	}
	
	public static RealVector crossProduct(RealVector vec1, RealVector vec2) {
		
        double[] result_vector_data = {vec1.getEntry(1) * vec2.getEntry(2) - vec1.getEntry(2) * vec2.getEntry(1),
        		vec1.getEntry(0) * vec2.getEntry(2) - vec1.getEntry(2) * vec2.getEntry(0),
        		vec1.getEntry(0) * vec2.getEntry(1) - vec1.getEntry(1) * vec2.getEntry(0)
        		};

        	// Moment due to Spin2
        	RealVector result_vector = MatrixUtils.createRealVector(result_vector_data);
        	
        	return result_vector;

	}

	public static RealVector matrixVectorMultiply(
			RealMatrix m, RealVector v) {
		double[][] md = {
				{m.getEntry(0, 0), m.getEntry(0, 1), m.getEntry(0, 2)  },
				{m.getEntry(1, 0), m.getEntry(1, 1), m.getEntry(1, 2)  },
				{m.getEntry(2, 0), m.getEntry(2, 1), m.getEntry(2, 2)  }
		};
		
		
		double[] vd = {v.getEntry(0), v.getEntry(1), v.getEntry(2)};
		
		
		double[] result_data = {
				md[0][0] * vd[0] + md[0][1] * vd[1] + md[0][2] * vd[2],
				md[1][0] * vd[0] + md[1][1] * vd[1] + md[1][2] * vd[2],
				md[2][0] * vd[0] + md[2][1] * vd[1] + md[2][2] * vd[2]};
		
		RealVector result = MatrixUtils.createRealVector(result_data);
		
		return result;
	}
	
	
	public static void debug(String str) {
		// System.out.println(str);
	}
	
}
