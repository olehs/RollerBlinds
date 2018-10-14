#include "RollerBlind.h"

#define BLIND_MAX 100.

RollerBlind::RollerBlind(byte dirPin, byte pwmPin, byte brakePin)
    : HBridgeMotor(dirPin, pwmPin, brakePin),
      downTime(0), upTime(0), currentPosition(0),
      downStartTime(0), upStartTime(0), startTime(0),
      targetPosition(0), startPosition(0),
      targeting(false), currentState(Stopped)
{
}

RollerBlind::~RollerBlind()
{
}

void RollerBlind::setPosition(byte pos)
{
    targetPosition = constrain(pos, 0, BLIND_MAX);
    if (targetPosition > position())
    {
        if (downTime)
        {
            down();
            targeting = true;
        }
    }
    else if (targetPosition < position())
    {
        if (upTime)
        {
            up();
            targeting = true;
        }
    }
    else
    {
        stop();
    }
}

void RollerBlind::updatePosition()
{
    if (startTime && downTime && goingDown())
    {
        float tm = BLIND_MAX * (millis() - startTime) / downTime;
        currentPosition = min(1.0 * startPosition + tm, BLIND_MAX);
    }
    else if (startTime && upTime && goingUp())
    {
        float tm = BLIND_MAX * (millis() - startTime) / upTime;
        currentPosition = max(1.0 * startPosition - tm, 0);
    }
}

byte RollerBlind::position()
{
    updatePosition();
    return currentPosition;
}

void RollerBlind::process()
{
    if (stopped())
        return;

    if (!startTime && moving()) // started moving
    {
        startTime = millis();
        if (downStartTime)
            downStartTime = startTime;
        else if (upStartTime)
            upStartTime = startTime;
    }

    if (targeting)
    {
        if ((forwarding() && (position() >= targetPosition)) || (backwarding() && (position() <= targetPosition)))
        {
            stop();
            return;
        }
    }

    if (startTime && !moving()) // endstop reached
    {
        if (downStartTime)
        {
            downTime = millis() - downStartTime;
            downStartTime = 0;
        }
        else if (upStartTime)
        {
            upTime = millis() - upStartTime;
            upStartTime = 0;
        }

        stop();
        if (forwarding())
        {
            currentPosition = BLIND_MAX;
            currentState = EndstopDown;
        }
        else if (backwarding())
        {
            currentPosition = 0;
            currentState = EndstopUp;
        }
    }

    if (startTime && moving() && (downTime > 0 || upTime > 0) && (millis() - startTime > max(downTime, upTime) * 1.5)) // stuck?
    {
        stop();
        currentState = Jammed;
    }
}

void RollerBlind::stop()
{
    targeting = false;
    downStartTime = 0;
    upStartTime = 0;
    updatePosition();
    HBridgeMotor::stop();
    currentState = Stopped;
}

void RollerBlind::brake()
{
    downStartTime = 0;
    upStartTime = 0;
    updatePosition();
    HBridgeMotor::brake();
    currentState = Brake;
}

void RollerBlind::up()
{
    up(m_speed);
}

void RollerBlind::down()
{
    down(m_speed);
}

void RollerBlind::up(byte speed)
{
    if (state() == EndstopUp)
        return;

    if (goingDown())
    {
        stop();
        while (moving())
            delay(10);
    }

    if (!goingUp())
    {
        startPosition = position();
        startTime = m_sensorPin ? 0 : millis();
        if (state() == EndstopDown)
        {
            upStartTime = millis();
            downStartTime = 0;
        }
    }

    goBackward(speed);
    currentState = GoingUp;
}

void RollerBlind::down(byte speed)
{
    if (state() == EndstopDown)
        return;

    if (goingUp())
    {
        stop();
        while (moving())
            delay(10);
    }

    if (!goingDown())
    {
        startPosition = position();
        startTime = m_sensorPin ? 0 : millis();
        if (state() == EndstopUp)
        {
            downStartTime = millis();
            upStartTime = 0;
        }
    }

    goForward(speed);
    currentState = GoingDown;
}

bool RollerBlind::goingUp()
{
    return moving() && backwarding();
}

bool RollerBlind::goingDown()
{
    return moving() && forwarding();
}

RollerBlind::BlindState RollerBlind::state()
{
    return currentState;
}