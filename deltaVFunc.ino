void deltaVFunc() {
  // controller

  // check encoder position FEET
  currPosLeftAng = (-1 * myEncLeft.read() * (PI * diameter)) / (3200);
  currPosLeftAng = currPosLeftAng / 12; // overflow

  // check encoder position FEET
  currPosRightAng = (myEncRight.read() * (PI * diameter)) / (3200);
  currPosRightAng = currPosRightAng / 12; // overflow

  // calculate error from current position to desired position
  errorR = angleDist - ((currPosLeftAng - currPosRightAng) / 2);
  cumErrorRight += (errorR * elapsedTime) / 1000;

  // for PI (V)
  deltaV = ((Kp2 * errorR) + (Ki2 * cumErrorRight)) * (255 / 8);
}
