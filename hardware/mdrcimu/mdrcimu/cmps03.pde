int readCompass(int addr) {
  int heading = -1;
  Wire.beginTransmission(addr);
  Wire.send(2);
  Wire.endTransmission();
  
  Wire.requestFrom(addr, 2);
  
  if (Wire.available() > 1) {
    heading = Wire.receive() << 8;
    heading |= Wire.receive();
  }
  
  return heading;
}

void yawDelta(int addr, float& delta, float& r, float& p, float& y, float ang_limit2) {
  float ang2 = p*p + r*r;
  if (ang2 < ang_limit2) {
    float cYaw = readCompass(addr) / 10.0;
  
    Serial.print("Heading read: ");
    Serial.println(cYaw);
    
    delta = cYaw - ToDeg(y);
    
    Serial.print("New delta: ");
    Serial.println(delta);
  }
}
