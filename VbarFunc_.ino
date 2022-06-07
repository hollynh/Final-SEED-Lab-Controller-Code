void vBarFuncPos() {
  // position controller for going straight to specified distance
  
  // check encoder position FEET
  currPosLeftPos = (-1 * myEncLeft.read() * (PI * diameter)) / (3200);
  currPosLeftPos = currPosLeftPos / 12; // overflow

  // check encoder position FEET
  currPosRightPos = (myEncRight.read() * (PI * diameter)) / (3200);
  currPosRightPos = currPosRightPos / 12; // overflow

  // calculate error from current position to desired position
  errorL = setStraight - ((currPosLeftPos + currPosRightPos) / 2);
  cumErrorLeft += (errorL * elapsedTime) / 1000;

  // for PI (V)
  vBar = ((Kp1 * errorL) + (Ki1 * cumErrorLeft)) * (255 / 8);  
}
