void motSat() {
  if (commandLeft > vmax) {
    commandLeft = vmax;
  }
  else if (commandLeft < -vmax) {
    commandLeft = -vmax;
  }
  if (commandRight > vmax) {
    commandRight = vmax;
  }
  else if (commandRight < -vmax) {
    commandRight = -vmax;
  }

  // adjust voltage pin signs (direction) to turn angle
  if (commandLeft <= 0) {
    // set motor to negative direction
    digitalWrite(motDirLeft, HIGH);
  }
  else {
    // set motor to positive direction
    digitalWrite(motDirLeft, LOW);
  }

  // adjust voltage pin signs (direction) to turn angle
  if (commandRight >= 0) {
    // set motor to negative direction
    digitalWrite(motDirRight, LOW);
  }
  else {
    // set motor to positive direction
    digitalWrite(motDirRight, HIGH);
  }
  commandLeftPrev = commandLeft;
  commandRightPrev = commandRight;

  // Can only send positive PWM
    commandLeft = abs(commandLeft);
    commandRight = abs(commandRight);

    // write voltage to the motor
    analogWrite(motorVLeft, commandLeft); 
    analogWrite(motorVRight, commandRight);
}
