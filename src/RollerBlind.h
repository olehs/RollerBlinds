#ifndef ROLLER_BLIND_H
#define ROLLER_BLIND_H

#include "HBridgeMotor.h"

#define BLIND_MAX 100.

class RollerBlind : public HBridgeMotor
{
public:
  enum BlindState
  {
    Unknown,
    Stopped,
    GoingUp,
    GoingDown,
    Brake,
    Jammed,
    EndstopUp,
    EndstopDown
  };

  RollerBlind(byte dirPin, byte pwmPin, byte brakePin = 0);
  ~RollerBlind();

  void setPosition(byte pos);
  byte position();
  void process();

  void up();
  void down();
  void up(byte speed);
  void down(byte speed);
  void stop();
  void brake();
  bool goingUp();
  bool goingDown();
  BlindState state();

  unsigned long downTime, upTime;
  byte currentPosition;

private:
  unsigned long downStartTime, upStartTime, startTime;
  byte targetPosition, startPosition;
  bool targeting;
  BlindState currentState;

  void updatePosition();
};

#endif // ROLLER_BLIND_H