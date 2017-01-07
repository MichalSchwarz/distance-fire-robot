#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

#define HALFSTEP 8
#define FULLSTEP 4
#define blue 2
#define pink 3
#define yellow 4
#define orange 5
#define MAX_SPEED 200
#define ACCELERATION 100
AccelStepper stepper1(HALFSTEP, blue, yellow, pink, orange);

#define SERVO_PIN 10
Servo servo;
unsigned long servo_timer = 0;
unsigned int servo_angle = 90;
unsigned int desired_servo_angle = 90;
unsigned int servo_speed = 100;

#define S_DONE 0
#define S_SCANNING 1
#define S_SET_DIRECTION 2
#define S_MOVE 3
unsigned int actual_step = S_SCANNING;

bool is_scan_running = false;
float distances[181];
#define S_SCAN_START_POSITION 1
#define S_SCAN_SCANNING 2
#define S_SCAN_FINAL_POSITION 3
unsigned int actual_scanning_step = S_SCAN_START_POSITION;

bool is_direction_running = false;
#define S_DIRECTION_STEERING 1
#define S_DIRECTION_MOVING 2
#define S_DIRECTION_FINAL_POSITION 3
unsigned int actual_direction_step = S_DIRECTION_STEERING;

#define ECHOPIN 7
#define TRIGPIN 8
#define ECHO_ENABLE_PIN 9
float closest_distance = 5000;
float closest_distance_base = 0;
unsigned int closest_angle = 90;


/**
* Nastavi vychozi hodnoty rychlosti a akcelerace stepperu
*/
void init_stepper(unsigned int speed, unsigned int acceleration);

/**
* Pootoci stepperem o pocet otacek po smeru hod. rucicek.
* Zaporne cislo = opacny smer
*/
void move_stepper_rotations(float rotations);

void init_echo();

void move_stepper_cm(float cm);

void servo_run(unsigned int angle, unsigned int speed);

void set_servo_angle(unsigned int angle);

unsigned int get_servo_angle();
unsigned int get_desired_servo_angle();

void run_scanning();

void run_set_direction();

bool is_scanning();

float get_distance_cm();

float get_rotations_by_angle(unsigned int angle);

bool prase = true;
bool koza = false;

void setup() {
  init_stepper(MAX_SPEED, ACCELERATION);
  init_echo();
  servo.attach(SERVO_PIN);
  Serial.begin(9600);
  //move_stepper_cm(20);
}

void loop() {

  if(actual_step == S_SCANNING)
  {
    run_scanning();
    actual_step = is_scanning() ? S_SCANNING : S_SET_DIRECTION;
  }
  else if(actual_step == S_SET_DIRECTION)
  {
    run_set_direction();
    actual_step = is_direction_running ? S_SET_DIRECTION : S_MOVE;
  }
  else if(actual_step == S_MOVE)
  {


  }


  stepper1.run();
  servo_run(desired_servo_angle, servo_speed);
}

void run_set_direction()
{
  bool stop_direction = false;
  if(actual_direction_step == S_DIRECTION_STEERING)
  {
    if(get_desired_servo_angle() != 0)
    {
      set_servo_angle(0);
    }
    if(get_desired_servo_angle() == get_servo_angle())
    {
      actual_direction_step = S_DIRECTION_MOVING;
    }
  }
  if(actual_direction_step == S_DIRECTION_MOVING)
  {
    Serial.print("prase\n");
    actual_direction_step = S_DIRECTION_FINAL_POSITION;
    move_stepper_cm(get_rotations_by_angle(closest_angle));

  }
  if(actual_direction_step == S_DIRECTION_FINAL_POSITION && !stepper1.isRunning())
  {
    servo_speed = 20;
    set_servo_angle(90);
    if(get_desired_servo_angle() == get_servo_angle())
    {
      move_stepper_cm(closest_distance_base-5);
      stop_direction = true;
    }

  }
  is_direction_running = !stop_direction;
}

float get_rotations_by_angle(unsigned int angle)
{

  double sensor_distance = 7.5;
  double rotation_diameter = 14.2;
  double diagonal_distance = sensor_distance + closest_distance;
  double axial_distance = cos(angle / (180 / M_PI)) * diagonal_distance;
  double radial_distance = sin(angle / (180 / M_PI)) * diagonal_distance;

  double rotation_angle = 180 / M_PI * atan(axial_distance / (rotation_diameter + radial_distance));
  float rotations_to_move = rotation_angle / 90 * 22.3;
  //rotations_to_move = 26;
  Serial.print(cos(angle / (180 / M_PI)));
  Serial.print("cos\n");
  Serial.print(rotation_angle);
  Serial.print("x\n");
  Serial.print(rotations_to_move);
  Serial.print("y\n");
  closest_distance_base = sqrt((rotation_diameter + radial_distance)*(rotation_diameter + radial_distance) + axial_distance*axial_distance)-rotation_diameter - sensor_distance;
  Serial.print(closest_distance_base);
  Serial.print("closest_distance_base\n");
  /*
  if(angle < 90)
  {
    rotations_to_move = 90 - angle;
  }
  else if(angle > 90)
  {
    rotations_to_move = -(angle - 90);
  }
  */
  return rotations_to_move;
}

void run_scanning()
{
  bool stop_scan = false;
  if(actual_scanning_step == S_SCAN_START_POSITION)
  {
    servo_speed = 20;
    if(get_desired_servo_angle() != 0)
    {
      set_servo_angle(0);
    }
    if(get_servo_angle() == 0)
    {
      actual_scanning_step = S_SCAN_SCANNING;
    }
  }

  else if (actual_scanning_step == S_SCAN_SCANNING)
  {
    servo_speed = 130;
    if(get_desired_servo_angle() == get_servo_angle())
    {
      distances[get_servo_angle()] = get_distance_cm();
      unsigned int new_angle = get_servo_angle()+1;
      set_servo_angle(new_angle);
    }
    if(get_servo_angle() >= 180)
    {
      for(int i = 0; i < 181; i++)
      {
        if(distances[i] < closest_distance)
        {
          closest_distance = distances[i];
          closest_angle = i;
        }
      }
      Serial.print(closest_distance);
      Serial.print("a\n");
      Serial.print(closest_angle);
      Serial.print("b\n");
      actual_scanning_step = S_SCAN_FINAL_POSITION;
    }
  }
  else if (actual_scanning_step == S_SCAN_FINAL_POSITION)
  {
    servo_speed = 20;
    if(get_servo_angle() != 90)
    {
      set_servo_angle(90);
    }
    else
    {
      stop_scan = true;
    }
  }
  is_scan_running = !stop_scan;
}

float get_distance_cm()
{

    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
    float distance = pulseIn(ECHOPIN, HIGH) * 0.017315f;
    delay(50);

  Serial.print(distance);
  Serial.print(" distance\n");

  return distance;
}

bool is_scanning()
{
  return is_scan_running;
}

void init_stepper(unsigned int speed, unsigned int acceleration)
{
  stepper1.setSpeed(speed);
  stepper1.setAcceleration(acceleration);
  stepper1.setMaxSpeed(2000.0);
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
  int stepper_steps = round(rotations * 4096);
  stepper1.move(stepper_steps);
}

void move_stepper_cm(float cm)
{
  float rotations = cm / 16.5;
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
