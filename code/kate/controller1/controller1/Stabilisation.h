// BALANCING VARIABLES
double    upper_velocity_threshold;
double    lower_velocity_threshold;
unsigned int smoother_step = 0;
double    corrective_angle = 0;
double    mid_point_angle = 0;
double    mid_point_distance = 0;
double    move_to_neutral_distance = 0;
boolean   s1_direction, s2_direction;
double    intermediate_move = 0;
double    final_angle_move = 0;
double    resting_angle_move = 0;

// Vectors
double thrust_vector[] = {0, 1, 0};
double x_vector[]      = {1, 0, 0};
double y_vector[]      = {0, 1, 0};     // Vector pointing in direction of rocket motion (up)
double vec3[]          = {0, 0, 0};     // Result of cross product