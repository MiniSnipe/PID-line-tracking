#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST        375
#define ZUMO_SLOW        250
#define ZUMO_TURN        320
#define X_CENTER         (pixy.frameWidth/2)

Pixy2 pixy;
ZumoMotors motors;
ZumoBuzzer buzzer;

PIDLoop headingLoop(9000, 4750, 5000, false);
unsigned long lastDetectionTime = 0;
const int detectionDelay = 10;  // 1 second delay
unsigned long startTurningTime = millis();

void setup() 
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  pixy.init();
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  // look straight and down
  pixy.setServos(350, 1000);
}

void loop()
{
  int8_t res;
  int32_t error; 
  int left, right;
  char buf[96];

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();

  // If error or nothing detected, turn 360 degrees after a delay
  while (res <= 0) 
  {
    if (millis() - lastDetectionTime > detectionDelay)
    {
      Serial.println("No vector detected. Turning 360 degrees.");

      // Turn the robot 180 degrees
      motors.setLeftSpeed(-ZUMO_TURN);
      motors.setRightSpeed(ZUMO_TURN);
      
      if (millis() - startTurningTime > 1000) 
      {
      // Stop turning after 3 seconds
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      break; // Exit the loop
      }

      // Update the last detection time
      lastDetectionTime = millis();

      if (res & LINE_VECTOR)
      {
        break;
      }
    }
 
    return;
  }

  // We found the vector...
  if (res & LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    pixy.line.vectors->print();

    // Reverse the vector if it points towards us
    if (pixy.line.vectors->m_y0 < pixy.line.vectors->m_y1)
    {
      // Swap the endpoints of the vector
      int temp = pixy.line.vectors->m_x0;
      pixy.line.vectors->m_x0 = pixy.line.vectors->m_x1;
      pixy.line.vectors->m_x1 = temp;
      
      temp = pixy.line.vectors->m_y0;
      pixy.line.vectors->m_y0 = pixy.line.vectors->m_y1;
      pixy.line.vectors->m_y1 = temp;
    }

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    // Separate heading into left and right wheel velocities.
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags & LINE_FLAG_INTERSECTION_PRESENT)
      {
        left += ZUMO_SLOW;
        right += ZUMO_SLOW;
      }
 
      else // otherwise, pedal to the metal!
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }    
    }
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);

    // Update the last detection time
    lastDetectionTime = millis();
    startTurningTime = millis();
  }
}


