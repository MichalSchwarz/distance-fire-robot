/**
 * Ovladani robota, ktery najde nejblizsi objekt v rozsahu -90 az +90 stupnu
 * od primeho smeru robota a dojede k nemu.
 *
 * @author Michal Schwarz <bc.michal.schwarz@gmail.com>
 * @licence MIT
 */

#include "main.h"

void setup()
{
  init_stepper(MAX_SPEED, ACCELERATION);
  init_echo();
  servo.attach(SERVO_PIN);
  pinMode(IGNITER_PIN, OUTPUT);
  actual_step = S_SCANNING;
}

void loop()
{
  if(actual_step == S_SCANNING)
  {
    run_scanning();
  }
  else if(actual_step == S_SET_DIRECTION)
  {
    run_set_direction();
  }
  else if(actual_step == S_MOVE)
  {
    run_final_move();
  }
  else if(actual_step == S_MAKE_FIRE)
  {
    run_fire();
  }
  stepper.run();
  servo_run(desired_servo_angle, servo_speed);
}

void run_fire()
{
  for(unsigned int i = 0; i < SPARKS_COUNT; i++)
  {
    digitalWrite(IGNITER_PIN, HIGH);
    delay(DELAY_IGNITE_ON);
    digitalWrite(IGNITER_PIN, LOW);
    delay(DELAY_IGNITE_OFF);
  }
  actual_step = S_DONE;
}

void run_final_move()
{
  float distance_cm = get_distance_cm();
  if(distance_cm > TOUCH_DISTANCE_CM)
  {
    move_stepper_cm(distance_cm - TOUCH_DISTANCE_CM);
  }
  if(!stepper.distanceToGo())
  {
    actual_step = S_MAKE_FIRE;
  }

}

float get_steering_step(int angle, float actual_distance)
{
  float step = ZERO_ROTATIONS;
  bool tolerance = angle < DIRECT_ANGLE + DIRECT_ANGLE_TOLERANCE &&
                   angle > DIRECT_ANGLE - DIRECT_ANGLE_TOLERANCE;
  if(actual_distance < TOUCH_DISTANCE_CM || tolerance)
  {
    step = ZERO_ROTATIONS;
  }
  else
  {
    if(actual_distance < ROTATION_CM + TOUCH_DISTANCE_CM)
    {
      step = (actual_distance - TOUCH_DISTANCE_CM) / ROTATION_CM;
    }
    else
    {
      step = DEFAULT_STEERING_ROTATIONS;
    }
  }
  return step;
}

unsigned int add_angle(unsigned int angle, int add)
{
  int result = angle + add;
  if(result > MAX_ANGLE)
  {
    result = MAX_ANGLE;
  }
  else if (result < MIN_ANGLE)
  {
    result = MIN_ANGLE;
  }
  return (unsigned int) result;
}

void set_scaning_range(int angle)
{
  if(angle > DIRECT_ANGLE)
  {
    scan_range_stop = add_angle(angle, STEERING_SCAN_ANGLE_LOW);
    scan_range_start = add_angle(angle, -STEERING_SCAN_ANGLE_HIGH);
  }
  else
  {
    scan_range_start = add_angle(angle, -STEERING_SCAN_ANGLE_LOW);
    scan_range_stop = add_angle(angle, STEERING_SCAN_ANGLE_HIGH);
  }
}

void run_set_direction()
{
  if(actual_direction_step == S_DIRECTION_MOVING)
  {
    float step = get_steering_step(closest_angle, closest_distance);
    if(step == ZERO_ROTATIONS)
    {
      set_servo_angle(closest_angle);
      actual_step = S_MOVE;
    }
    else
    {
      move_stepper_rotations(step);
      actual_direction_step = S_DIRECTION_STEERING;
    }
  }
  if(actual_direction_step == S_DIRECTION_STEERING)
  {
    if(!stepper.distanceToGo())
    {
      set_scaning_range(closest_angle);
      actual_scanning_step = S_SCAN_START_POSITION;
      actual_step = S_SCANNING;
    }
  }
}

void run_scanning()
{
  if(actual_scanning_step == S_SCAN_START_POSITION)
  {
    servo_speed = SERVO_SPEED_MOVE;
    if(get_desired_servo_angle() != scan_range_start)
    {
      set_servo_angle(scan_range_start);
    }
    if(get_servo_angle() == scan_range_start)
    {
      actual_scanning_step = S_SCAN_SCANNING;
      closest_distance = UNREACHABLE_DISTANCE_CM;
    }
  }

  else if (actual_scanning_step == S_SCAN_SCANNING)
  {
    servo_speed = SERVO_SPEED_SCAN;
    if(get_desired_servo_angle() == get_servo_angle())
    {
      distances[get_servo_angle()] = get_distance_cm();
      unsigned int new_angle = get_servo_angle() + 1;
      set_servo_angle(new_angle);
    }
    if(get_servo_angle() >= scan_range_stop)
    {
      for(unsigned int i = MIN_ANGLE; i < MAX_ANGLE + 1; i++)
      {
        if(distances[i] > 0 && distances[i] < closest_distance)
        {
          closest_distance = distances[i];
          closest_angle = i;
        }
      }
      for(unsigned int i = MIN_ANGLE; i < MAX_ANGLE + 1; i++)
      {
        distances[i] = UNREACHABLE_DISTANCE_CM;
      }
      actual_scanning_step = S_SCAN_FINAL_POSITION;
    }
  }
  else if (actual_scanning_step == S_SCAN_FINAL_POSITION)
  {
    servo_speed = SERVO_SPEED_MOVE;
    if(get_servo_angle() != closest_angle)
    {
      set_servo_angle(closest_angle);
    }
    else
    {
      actual_direction_step = S_DIRECTION_MOVING;
      actual_step = S_SET_DIRECTION;
    }
  }
}

float get_distance_cm()
{
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(ECHO_DELAY_MEAS);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(ECHO_DELAY_RESPONSE);
  digitalWrite(TRIGPIN, LOW);
  float distance = pulseIn(ECHOPIN, HIGH) * ECHO_DISTANCE_RATIO;
  delay(ECHO_DELAY);
  return distance;
}

void init_stepper(unsigned int speed, unsigned int acceleration)
{
  stepper.setSpeed(speed);
  stepper.setAcceleration(acceleration);
  stepper.setMaxSpeed(MAX_SPEED);
}

void init_echo()
{
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHO_ENABLE_PIN, OUTPUT);
  digitalWrite(ECHO_ENABLE_PIN, HIGH);
}

void move_stepper_rotations(float rotations)
{
  int stepper_steps = round(rotations * ROTATION_STEPS);
  stepper.move(stepper_steps);
}

void move_stepper_cm(float cm)
{
  float rotations = cm / ROTATION_CM;
  move_stepper_rotations(rotations);
}

void set_servo_angle(unsigned int angle)
{
  desired_servo_angle = angle;
}

unsigned int get_servo_angle()
{
  return servo_angle;
}

unsigned int get_desired_servo_angle()
{
  return desired_servo_angle;
}

void servo_run(unsigned int angle, unsigned int speed)
{
  unsigned long actual_time = millis();
  bool run_allowed = actual_time - servo_timer >= speed;
  if (run_allowed)
  {
    if(angle > servo_angle)
    {
      servo.write(++servo_angle);
    }
    else if(angle < servo_angle)
    {
      servo.write(--servo_angle);
    }
    servo_timer = actual_time;
  }
}
