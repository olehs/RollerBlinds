#include "RollerBlind.h"
#include "RCSwitch.h"

/*
 * If your Arduino board has additional serial ports
 * you can use to connect the RS485 module.
 * Otherwise, the transport uses AltSoftSerial to handle two serial
 * links on one Arduino. Use the following pins for RS485 link
 *
 *  Board          Transmit  Receive   PWM Unusable
 * -----          --------  -------   ------------
 * Arduino Uno        9         8         10
 * Arduino Mega      46        48       44, 45
 *
 */

#define MY_NODE_ID 30

// Enable debug prints to serial monitor
// #define MY_DEBUG

// #define MY_RS485_SOH_COUNT 3

// Set how long to wait for transport ready in milliseconds
#define MY_TRANSPORT_WAIT_READY_MS 10000

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 4

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

// Enable this if RS485 is connected to a hardware serial port
//#define MY_RS485_HWSERIAL Serial1

#include <MySensors.h>

#define BLIND_DIR 12
#define BLIND_PWM 3

RollerBlind blind(BLIND_DIR, BLIND_PWM);

#define CHILD_ID 1 // Id of the sensor child

RCSwitch rcSwitch = RCSwitch();

#define LEARN_BUTTON_PIN 6
#define LEARN_ADDR 16
#define TIME_ADDR 48
#define POS_ADDR 56
#define LED_LEARNING 7

#define HOLD_TIME 200

byte learning = 0;
unsigned long codes[4];

unsigned long downTime = 0, upTime = 0;
byte last_position = 0;
RollerBlind::BlindState last_state = RollerBlind::Unknown;

MyMessage msg(CHILD_ID, V_STATUS);

byte last_button = 0;
unsigned long button_time = 0;

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Roller Blinds", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID, S_COVER);
}

void receive(const MyMessage &message)
{
  byte val;
  switch (message.type)
  {
  case V_VAR1:
    val = message.getByte();
    blind.setSpeed(val);
    saveState(message.sensor, val);

    Serial.print("SET SPEED: ");
    Serial.println(val);
    break;

  case V_PERCENTAGE:
    val = constrain(message.getByte(), 0, BLIND_MAX);
    if (val == 0)
      blind.up();
    else if (val == BLIND_MAX)
      blind.down();
    else
      blind.setPosition(val);

    Serial.print("SET POSITION: ");
    Serial.println(val);
    break;

  case V_UP:
    Serial.println("UP command");
    blind.up();
    break;

  case V_DOWN:
    Serial.println("DOWN command");
    blind.down();
    break;

  case V_STOP:
    Serial.println("STOP command");
    blind.stop();
    break;

  default:
    break;
  }
}

void setup()
{
  wdt_disable(); // maybe redundant

  blind.setSensorPin(A0, 20);

  rcSwitch.enableReceive(0); // UNO pin #2

  pinMode(LEARN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_LEARNING, OUTPUT);
  digitalWrite(LED_LEARNING, LOW);

  hwReadConfigBlock(&codes, EEPROM_LOCAL_CONFIG_ADDRESS + LEARN_ADDR, sizeof(codes));
  hwReadConfigBlock(&downTime, EEPROM_LOCAL_CONFIG_ADDRESS + TIME_ADDR, sizeof(downTime));
  hwReadConfigBlock(&upTime, EEPROM_LOCAL_CONFIG_ADDRESS + TIME_ADDR + sizeof(upTime), sizeof(upTime));
  blind.downTime = downTime;
  blind.upTime = upTime;
  Serial.print("Down time: ");
  Serial.println(downTime / 1000);
  Serial.print("Up time: ");
  Serial.println(upTime / 1000);

  byte speed = loadState(CHILD_ID);
  blind.currentPosition = loadState(POS_ADDR);

  blind.setSpeed(speed);
  blind.stop();

  send(msg.setType(V_VAR1).set(speed));

  wdt_enable(WDTO_4S);

  Serial.println("Started");
}

void processRC()
{
  if (((last_button == 3) || (last_button == 4)) && (millis() - button_time > HOLD_TIME))
  {
    if (blind.moving())
      blind.stop();
    last_button = 0;
  }

  if (!learning && digitalRead(LEARN_BUTTON_PIN) == LOW)
  {
    learning = 1;
    digitalWrite(LED_LEARNING, HIGH);
    Serial.println("Entering learning mode");
  }

  if (!rcSwitch.available())
    return;

  unsigned long value = rcSwitch.getReceivedValue();
  Serial.print("Received ");
  Serial.println(value);

  if (learning)
  {
    byte idx = learning - 1;
    codes[idx] = value;
    hwWriteConfigBlock(&codes[idx], EEPROM_LOCAL_CONFIG_ADDRESS + LEARN_ADDR + (idx) * sizeof(unsigned long), sizeof(unsigned long));
    digitalWrite(LED_LEARNING, LOW);
    Serial.print("Learned button #");
    Serial.println(learning);
    delay(500);
    digitalWrite(LED_LEARNING, HIGH);

    if (++learning > 4)
    {
      learning = 0;
      digitalWrite(LED_LEARNING, LOW);
    }
  }
  else
  {
    unsigned long tm = millis();
    byte button = 0;
    for (byte i = 0; i < 4; i++)
    {
      if (value == codes[i])
      {
        button = i + 1;
        break;
      }
    }

    if ((button != last_button) || ((tm - button_time) > HOLD_TIME))
    {
      switch (button)
      {
      case 1:
        if (blind.goingUp())
        {
          blind.stop();
          Serial.println("STOP pressed");
        }
        else
        {
          blind.up(255);
          Serial.println("UP pressed");
        }
        break;

      case 2:
        if (blind.goingDown())
        {
          blind.stop();
          Serial.println("STOP pressed");
        }
        else
        {
          blind.down(255);
          Serial.println("DOWN pressed");
        }
        break;

      case 3:
        blind.up(255);
        Serial.println("Step Up");
        break;

      case 4:
        blind.down(255);
        Serial.println("Step Down");
        break;

      default:
        break;
      }
    }

    if (button)
    {
      last_button = button;
      button_time = tm;
    }
  }

  rcSwitch.resetAvailable();
}

void updateTimes()
{
  if (blind.downTime && (downTime != blind.downTime))
  {
    downTime = blind.downTime;
    hwWriteConfigBlock(&downTime, EEPROM_LOCAL_CONFIG_ADDRESS + TIME_ADDR, sizeof(downTime));
    Serial.print("DOWN time: ");
    Serial.println(downTime / 1000);
  }

  if (blind.upTime && (upTime != blind.upTime))
  {
    upTime = blind.upTime;
    hwWriteConfigBlock(&upTime, EEPROM_LOCAL_CONFIG_ADDRESS + TIME_ADDR + sizeof(downTime), sizeof(upTime));
    Serial.print("UP time: ");
    Serial.println(upTime / 1000);
  }
}

bool sendMsg(MyMessage &m, byte retries = 4)
{
  while (retries--)
  {
    if (send(m))
      return true;
    wait(1);
  }
  return false;
}

void reportState()
{
  RollerBlind::BlindState state = blind.state();

  if (state != last_state)
  {
    switch (state)
    {
    case RollerBlind::Stopped:
      msg.set("STOPPED");
      break;
    case RollerBlind::GoingUp:
      msg.set("UP");
      break;
    case RollerBlind::GoingDown:
      msg.set("DOWN");
      break;
    case RollerBlind::Brake:
      msg.set("BRAKE");
      break;
    case RollerBlind::Jammed:
      msg.set("JAMMED");
      break;
    case RollerBlind::EndstopUp:
      msg.set("TOP");
      break;
    case RollerBlind::EndstopDown:
      msg.set("BOTTOM");
      break;
    default:
      msg.set("UNKNOWN");
      break;
    }

    if (sendMsg(msg.setType(V_STATUS)))
      last_state = state;
  }

  byte pos = blind.position();
  if (last_position != pos)
  {
    if ((state == RollerBlind::Stopped) || (state == RollerBlind::EndstopDown) || (state == RollerBlind::EndstopUp) || ((pos % 5) == 0))
    {
      // Serial.print("Current position: ");
      // Serial.println(pos);
      if (sendMsg(msg.setType(V_PERCENTAGE).set(pos)))
      {
        last_position = pos;
        saveState(POS_ADDR, pos);
      }
    }
  }
}

void loop()
{
  wdt_reset();
  processRC();
  blind.process();
  updateTimes();
  reportState();
}