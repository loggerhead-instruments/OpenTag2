

// simple calculation of pitch, roll, yaw
void euler(){
  
  int adj_mag_x = mag_x - magXoffset;
  int adj_mag_y = mag_y - magYoffset;
  int adj_mag_z = mag_z - magZoffset;
  
  // roll
  float phi = atan2(accel_y, accel_z); // roll in radians

  // de-rotate by roll angle
  float sinAngle = sin(phi);
  float cosAngle = cos(phi);
  float Bfy = (adj_mag_y * cosAngle) - (adj_mag_z * sinAngle);
  float Bz = (adj_mag_y * sinAngle) + (adj_mag_z * cosAngle);
  float Gz = (accel_y * sinAngle) + (accel_z * cosAngle);

  // theta = pitch angle
  float theta = atan(- accel_x / Gz);
  sinAngle = sin(theta);
  cosAngle = cos(theta);

  // de-rotate by pitch angle theta
  float Bfx = (adj_mag_x * cosAngle) + (Bz * sinAngle);
  float Bfz = (-adj_mag_x * sinAngle) + (Bz * cosAngle);

  // Psi = yaw = heading
  float psi = atan2(-Bfy, Bfx);

  pitch = degrees(theta);
  roll = degrees(phi);
  yaw = degrees(psi);
}

