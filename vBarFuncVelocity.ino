void vBarFuncVel() {
  // controller for velocity
  
  // check encoder position FEET
  currPosLeftVel = (-1 * myEncLeft.read() * (PI * diameter)) / (3200);
  currPosLeftVel = currPosLeftVel / 12; // overflow

  // check encoder position FEET
  currPosRightVel = (myEncRight.read() * (PI * diameter)) / (3200);
  currPosRightVel = currPosRightVel / 12; // overflow

  // average current positions
  currPosAve = (currPosLeftVel + currPosRightVel) / 2;

  // find angular velocity using timing from main loop
  angVel = ((currPosAve - oldPosAve) * (1000 / elapsedTime));

  // calculate error from current position to desired position
  errorL = setVel - angVel;
  cumErrorLeft += (errorL * elapsedTime) / 1000;

  // for PI (V)
  vBar = ((Kp1 * errorL) + (Ki1 * cumErrorLeft)) * (255 / 8);

  oldPosAve = currPosAve;  
}
