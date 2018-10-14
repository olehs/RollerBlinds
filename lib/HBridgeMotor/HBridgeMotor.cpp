#include "HBridgeMotor.h"

#define MOVE_DEBOUNCE 100

HBridgeMotor::HBridgeMotor(byte dirPin, byte pwmPin, byte brakePin)
    : m_dirPin(dirPin), m_pwmPin(pwmPin), m_brakePin(brakePin), m_sensorPin(0), m_speed(0),
      m_invertDir(false), m_stopped(true), m_sensorThreshold(0),
      moveTime(0)
{
    pinMode(dirPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    if (brakePin)
        pinMode(brakePin, OUTPUT);
}

HBridgeMotor::~HBridgeMotor()
{
}

void HBridgeMotor::setInvertDir(bool invertDir)
{
    if (m_invertDir != invertDir)
    {
        m_invertDir = invertDir;
        digitalWrite(m_dirPin, digitalRead(m_dirPin) == HIGH ? LOW : HIGH);
    }
}

void HBridgeMotor::setSensorPin(byte sensorPin, int threshold)
{
    if (sensorPin)
    {
        m_sensorPin = sensorPin;
        m_sensorThreshold = threshold;
    }
}

void HBridgeMotor::setSpeed(byte speed)
{
    analogWrite(m_pwmPin, speed);
    m_speed = speed;
    m_stopped = speed == 0;
}

byte HBridgeMotor::getSpeed()
{
    return m_speed;
}

void HBridgeMotor::goForward()
{
    goForward(m_speed);
}

void HBridgeMotor::goBackward()
{
    goBackward(m_speed);
}

void HBridgeMotor::goForward(byte speed)
{
    analogWrite(m_pwmPin, speed);
    digitalWrite(m_dirPin, m_invertDir ? LOW : HIGH);
    resume();
    m_stopped = speed == 0;
}

void HBridgeMotor::goBackward(byte speed)
{
    analogWrite(m_pwmPin, speed);
    digitalWrite(m_dirPin, m_invertDir ? HIGH : LOW);
    resume();
    m_stopped = speed == 0;
}

void HBridgeMotor::stop()
{
    analogWrite(m_pwmPin, 0);
    m_stopped = true;
}

void HBridgeMotor::brake()
{
    if (m_brakePin)
    {
        digitalWrite(m_brakePin, HIGH);
    }
}

void HBridgeMotor::resume()
{
    if (m_brakePin)
    {
        digitalWrite(m_brakePin, LOW);
    }
}

bool HBridgeMotor::moving()
{
    if (m_sensorPin && (m_sensorThreshold > 0))
    {
        bool state = analogRead(m_sensorPin) >= m_sensorThreshold;
        unsigned long tm = moveTime;
        if (state != moveState)
            moveTime = millis();
        if (millis() - tm > MOVE_DEBOUNCE)
            moveState = state;

        return moveState;
    }
    else
        return !stopped() && !braked() && (m_speed > 0);
}

bool HBridgeMotor::stopped()
{
    return m_stopped;
}

bool HBridgeMotor::braked()
{
    if (!m_brakePin)
        return false;

    return digitalRead(m_brakePin) == HIGH;
}

bool HBridgeMotor::forwarding()
{
    return (m_speed > 0) && (digitalRead(m_dirPin) == m_invertDir ? LOW : HIGH);
}

bool HBridgeMotor::backwarding()
{
    return (m_speed > 0) && (digitalRead(m_dirPin) == m_invertDir ? HIGH : LOW);
}
