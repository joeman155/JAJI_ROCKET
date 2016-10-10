


#ifdef STABILITY_CORRECTION
// Detect instability of the rocket
boolean check_system_stability(double vx, double vy, double vz, double ax, double ay, double az)
{
  // Check if our velocity measurements exceed upper threshold
  if (abs(vx) > upper_velocity_threshold || abs(vz) > upper_velocity_threshold) {

    // Peform additional check to double check we need to make adjustments to CG
    if (
      (sgn(ax) * sgn(vx) == -1 || abs(vx) <  upper_velocity_threshold)
      &&
      (sgn(az) * sgn(vz) == -1 || abs(vz) < upper_velocity_threshold)
    ) {
      return false;
    }

    return true;
  }

  return false;
}



void calculate_smoother_location(double vx, double vy, double vz)
{
  // Deduce where the smoother should be
  // theta = Math.atan(r.getAng_vz()/r.getAng_vx());
  // We Already have the Rotational velocity co-ordinate in the local system
  // RealVector corrective_torque_direction = utils.revolveVector(0, Math.PI,  0, rotation_velocity_local);
  double corrective_torque_direction[] = { -1 * vx , -1 * vy, -1 * vz};


  // Create Unit Correction'Vector'
  double corrective_rotation[] = {corrective_torque_direction[0], corrective_torque_direction[1], corrective_torque_direction[2]};

  // Generate the thrust vector ... not caring about magnitude...only direction...in local coordinate system
  // DECLARED AT BEGINNING

  // Determine the direction of the CG vector...needed to produce torque to oppose the current motion
  crossproduct(thrust_vector, corrective_rotation);
  double corrective_cg_vector[] = { vec3[0], vec3[1], vec3[2] };

  // Determine angle vector in X-direction in local reference frame... Use this later to find angle the CG vector makes with X-axis
  // DECLARED AT BEGINNING

  // Determine the angle this CG makes
  double corrective_cg_vector_size = pow(pow(corrective_cg_vector[0], 2) + pow(corrective_cg_vector[1], 2) + pow(corrective_cg_vector[2], 2), 0.5 );

  // Calculate dot product
  double dot_product = (x_vector[0] * corrective_cg_vector[0] + x_vector[1] * corrective_cg_vector[1] + x_vector[2] * corrective_cg_vector[2]) / corrective_cg_vector_size;

  // Correct for any rounding issues (that make the abs(dot_product) > 1); Otherwise we get NAN errors!
  if (dot_product > 1) {
    dot_product = 1;
  } else if (dot_product < -1) {
    dot_product = -1;
  }

  // Deduce the angle!
  corrective_angle = acos(dot_product);

  // Figure out if    0 < angle 180  OR   180 < angle < 360
  double zcross = x_vector[0] * corrective_cg_vector[2] - corrective_cg_vector[0] * x_vector[2];

  // Need to get corrective_angle beteween 0 and 2 x PI. We use cross product above to sort this out.
  if (zcross > 0) {
    corrective_angle = 2 * PI - corrective_angle;
  }

  // Smoothers 180 degrees out of phase from direction of 'corrective CG vector'
  corrective_angle = corrective_angle + PI;

  // Make sure angle is between 0 and 6.28
  corrective_angle = angle_reorg(corrective_angle);

}


void smoother_step_processing()
{
  if (smoother_step == 1) {
    smoother_step_1();
  } else if (smoother_step == 2) {
    smoother_step_2();
  } else if (smoother_step == 3) {
    smoother_step_3();
  } else if (smoother_step == 4) {
    smoother_step_4();
  } else if (smoother_step == 5) {
    smoother_step_5();
  } else if (smoother_step == 6) {
    smoother_step_6();
  } else if (smoother_step == 7) {
    smoother_step_7();
  }

}


void smoother_step_1()
{

  //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
  mid_point_angle = angle_between(s1_angle, s2_angle);

  // We know that mid_point_angle MUST be less then OR Equal to 180 degrees BECAUSE this angle is got from dot-product
  move_to_neutral_distance = (PI - mid_point_angle) / 2;

  // **** CALCULATE WHERE MID POINT OF SMOOTHERS IS AND HOW FAR TO MOVE TO CORRECTIVE ANGLE ****
  //      WE DO THIS BECAUSE IT MIGHT BE QUICKER TO DO THIS INSTEAD OF GOING TO NEUTRAL POSITION
  mid_point_distance  = (s1_angle + s2_angle) / 2;                                           // Angular distance mid-way between s1 and s2

  // We want the mid-point being the direction in which the acute angle (< 180) is directed
  // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
  if (abs(s2_angle - s1_angle) > PI) {
    mid_point_distance = mid_point_distance + PI;
  }
  mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI


  intermediate_move = angle_between(corrective_angle, mid_point_distance);
  if (intermediate_move > PI / 2) {                                                           // If Greater than PI/2, then we are being in-efficient
    mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase)
    mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
    intermediate_move = angle_between(corrective_angle, mid_point_distance);
  }

  smoother_step = 2;

  /*
          Serial.println("INTERMEDIATE:    " + String(intermediate_move));
    Serial.println("S1/S2 Angle:       " + String(mid_point_angle));
    Serial.println("NEUTRAL MOV REQ'D: " + String(move_to_neutral_distance));
    Serial.println("s1_direction: "    + String(s1_direction));
  */

}


// Move smoothers so they are 180 degrees out of phase if deemed necessary
void smoother_step_2()
{

  if (move_to_neutral_distance <= 0) {
    // Already 180 degrees out of phase, so no need to move
    smoother_step = 4;
    // No need to move to neutral position, already in neutral position
  } else if (intermediate_move < PI / 4) {
    // Only a small movement required, so we will move straight to that position...this is because the increased speed in getting to the final
    // position outweighs the imbalances that might be caused.
    smoother_step = 4;
    // Only a small movement required, so we will move straight to that position
  } else {
    // OK...so we have a large movement, and we can't afford to destabilise system, so we need to move to neutral position
    derive_direction();

    print_time();

#ifdef DEBUG
    Serial.println("Neutral Move");
#endif

    move_servos(s1_direction, s2_direction, move_to_neutral_distance, 0);
    smoother_step = 3;
  }
}





// This is run if smoothers moved to get into neutral position. We will need to re-calc some values
void smoother_step_3()
{
  // Find angle between the two smoothers...then halve...this is the mid-point
  // (We need to re-calculate because we may have moved in smoothers in previous step)
  // mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));
  mid_point_angle =  angle_between(s1_angle, s2_angle);

  mid_point_distance  = (s1_angle + s2_angle) / 2;                                           // Angular distance mid-way between s1 and s2

  // We want the mid-point being the direction in which the acute angle (< 180) is directed
  // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
  if (abs(s2_angle - s1_angle) > PI) {
    mid_point_distance = mid_point_distance + PI;
  }
  mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI

  // Deduce the distance we need to move
  intermediate_move = angle_between(corrective_angle, mid_point_distance);
  if (intermediate_move > PI / 2) {                                                           // If Greater than PI/2, then we are being in-efficient
    mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase)
    mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
    intermediate_move = angle_between(corrective_angle, mid_point_distance);
  }



  // Serial.println("step3: S1/S2 Angle:                  " + String(mid_point_angle));
  // Serial.println("step3: mid_point_distance Angle:     " + String(mid_point_distance));
  // Serial.println("step3:  IM: " + String(intermediate_move));

  // Signal to code to go on to 'Intermediate' move
  smoother_step = 4;
}


// Perform immediate move (if required) and then calculate the final move.
void smoother_step_4()
{
  // Intermediate move - moving both weights together


  if (intermediate_move < 0) {
    smoother_step = 5;
  } else {

    // s1_direction = 1;  // FALSE = CCW, TRUE = CW
    // s2_direction = 1;  // FALSE = CCW, TRUE = CW


    // DIRECTION TO GET SMOOTHERS TO INTERMEDIATE POSITION - in FASTEST POSSIBLE WAY!
    // To assist us in finding directions to move the smoothers we need to get Cross product of the midpoint vector and the
    // Corrective direction vectore.
    // The direction this resultant vector...up (+ve y) or down (-ve y) tells us which way to rotate the smoothers
    // NOTE: The Smoothers ALWAYS move in opposite directions with respect to each other
    // If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx


    double i_vector[] = {cos(mid_point_distance), 0, -sin(mid_point_distance)};
    double c_vector[]  = {cos(corrective_angle), 0, -sin(corrective_angle)};

    crossproduct(i_vector, c_vector);

    double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];

    // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
    if (zcross > 0 ) {
      s1_direction = cw;
      s2_direction = cw;
    } else if (zcross < 0 ) {
      s1_direction = ccw;
      s2_direction = ccw;
    }

    print_time();

#ifdef DEBUG
    Serial.print("IM: ");
    Serial.println(intermediate_move);
#endif

    move_servos(s1_direction, s2_direction, intermediate_move, 0);
    smoother_step = 5;
  }


  // CALCULATE THE FINAL MOVE
  mid_point_angle =  angle_between(s1_angle, s2_angle);

  mid_point_distance  = (s1_angle + s2_angle) / 2;                                           // Angular distance mid-way between s1 and s2

  final_angle_move = angle_between(corrective_angle, s1_angle);
}



void smoother_step_5()
{

  if (final_angle_move < 0) {
    smoother_step = 6;

  } else {

    // DIRECTION TO GET SMOOTHERS TO FINAL POSITION - in FASTEST POSSIBLE WAY!
    // To assist us in finding directions to move the smoothers we need to get Cross product of the s1 smoother vector and the
    // Corrective direction vectore.
    // The direction this resultant vector...up (+ve y) or down (-ve y) tells us which way to rotate the smoothers
    // NOTE: The Smoothers ALWAYS move in opposite directions with respect to each other
    // If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx


    double s1_vector[] = {cos(s1_angle), 0, -sin(s1_angle)};
    double c_vector[]  = {cos(corrective_angle), 0, -sin(corrective_angle)};

    crossproduct(s1_vector, c_vector);

    double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];

    // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
    if (zcross > 0 ) {
      s1_direction = cw;
      s2_direction = ccw;
    } else if (zcross < 0 ) {
      s1_direction = ccw;
      s2_direction = cw;
    }

    print_time();

#ifdef DEBUG
    Serial.print("FM: ");
    Serial.println(final_angle_move);
#endif

    move_servos(s1_direction, s2_direction, final_angle_move, lower_velocity_threshold);
    smoother_step = 6;
  }
}


void smoother_step_6()
{

  // First make sure that acceleration is in opposite direction of velocity (i.e. it is slowing down)
  if ((sgn(rotation_ax) * sgn(rotation_vx) != -1 && abs(rotation_vx) > upper_velocity_threshold)
      ||
      (sgn(rotation_az) * sgn(rotation_vz) != -1 && abs(rotation_vz) > upper_velocity_threshold)) {


#ifdef DEBUG
    Serial.println("Malfu or over correcting.!");
#endif

    // COMMENT this code in this IF blovk FOR TESTING WHEN NOT NOT EXCEPTING ACTUAL CORRECTION OF SYSTEM (because we 'simulate' a movement

    // So. let's assume we are over-correcting
    smoother_step = 7;


    //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
    mid_point_angle = angle_between(s1_angle, s2_angle);
    resting_angle_move = (PI - mid_point_angle) / 2;

    derive_direction();

  }

  // If velocity < lower_velocity_threshold, then start to reduce acceleration
  if (abs(rotation_vx) <  lower_velocity_threshold && abs(rotation_vz) <  lower_velocity_threshold) {

#ifdef DEBUG
    Serial.println("Success. Easing back back");
#endif

    smoother_step = 7;

    //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
    mid_point_angle = angle_between(s1_angle, s2_angle);

    // We know that mid_point_angle MUST be less then OR Equal to 180 degrees BECAUSE this angle is got from dot-product
    resting_angle_move = (PI - mid_point_angle) / 2;

    derive_direction();
  }
}


void smoother_step_7()
{
  print_time();

#ifdef DEBUG
  Serial.print("RM: ");
  Serial.println(resting_angle_move);
#endif

  move_servos(s1_direction, s2_direction, resting_angle_move, lower_velocity_threshold);
  smoother_step = 0;
  digitalWrite(LED_INDICATOR_PIN, HIGH);
  end_time1 = micros();

}


void move_servos(boolean s1_direction, boolean s2_direction, double angle, double threshold)
{
  // TODO

#ifdef DEBUG
  Serial.print("S1 ANG: "); Serial.println(s1_angle);
  Serial.print("S2 ANG: "); Serial.println(s2_angle);
#endif
}





static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

static double angle_reorg(double angle) {
  double new_angle;

  if (abs(angle) >= 2 * PI) {
    new_angle = 2 * PI * ((angle / (2 * PI)) - round(angle / (2 * PI)));
  } else {
    new_angle = angle;
  }

  if (new_angle < 0) {
    new_angle = new_angle + PI * 2;
  }
  return new_angle;

}

void crossproduct(double vec1[], double vec2[])
{
  vec3[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec3[1] = vec1[0] * vec2[2] - vec1[2] * vec2[0];
  vec3[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}


// DIRECTION TO GET SMOOTHERS TO REST/NEUTRAL POSITION - in FASTEST POSSIBLE WAY!
// To assist us in finding directions to move the smoothers we need to get Cross product of the two smoother angles
// and see which direction this vector is pointing...up (+ve y) or down (-ve y)
// NOTE: The Smoothers ALWAYS move in opposite directions
// If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx
void derive_direction()
{
  s1_angle = angle_reorg(s1_angle);
  s2_angle = angle_reorg(s2_angle);

  // Get vector equivalents that the s1/s2 smoothers make. We have negate the Z direction, because the angle goes in opposite direction
  // Remember the smoothers lie in the X-Z plane
  double s1_vector[] = {cos(s1_angle), 0, -sin(s1_angle)};
  double s2_vector[] = {cos(s2_angle), 0, -sin(s2_angle)};

  crossproduct(s1_vector, s2_vector);

  double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];

  // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
  if (zcross > 0) {
    s1_direction = ccw;
    s2_direction = cw;
  } else if (zcross < 0) {
    s1_direction = cw;
    s2_direction = ccw;
  }
}



// The routine Assumes that angle1, angle2 are between 0 and 2PI
double angle_between(double angle1, double angle2)
{
  double angle;

  angle = abs(angle1 - angle2);
  angle = angle_reorg(angle);

  if (angle > PI) {
    angle = 2 * PI -  angle;
  }

  return angle;
}

#endif
