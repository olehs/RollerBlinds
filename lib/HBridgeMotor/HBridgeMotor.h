#ifndef HBRIDGEMOTOR_H
#define HBRIDGEMOTOR_H

#include "Arduino.h"

class HBridgeMotor
{
protected:
  byte m_dirPin, m_pwmPin, m_brakePin, m_sensorPin;
  byte m_speed;
  bool m_invertDir, m_stopped;
  int m_sensorThreshold;
  unsigned long moveTime;
  bool moveState;

public:
  HBridgeMotor(byte dirPin, byte pwmPin, byte brakePin = 0);
  ~HBridgeMotor();

  void setInvertDir(bool invertDir);
  void setSensorPin(byte sensorPin, int threshold);

  void setSpeed(byte speed);
  byte getSpeed();

  void goForward();
  void goBackward();
  virtual void goForward(byte speed);
  virtual void goBackward(byte speed);

  virtual void stop();
  virtual void brake();
  virtual void resume();

  bool moving();
  bool stopped();
  bool braked();
  virtual bool forwarding();
  virtual bool backwarding();
};

#endif // HBRIDGEMOTOR_H