/**
 * Ovladani robota, ktery najde nejblizsi objekt v rozsahu -90 az +90 stupnu
 * od primeho smeru robota a dojede k nemu.
 *
 * @author Michal Schwarz <bc.michal.schwarz@gmail.com>
 * @licence MIT
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

#define HALFSTEP 8
#define blue 2
#define pink 3
#define yellow 4
#define orange 5
#define MAX_SPEED 200
#define ACCELERATION 100
#define ROTATION_CM 16.5
#define ROTATION_STEPS 4096
#define TOUCH_DISTANCE_CM 10
#define UNREACHABLE_DISTANCE_CM 1000000
#define DEFAULT_STEERING_ROTATIONS 1
#define ZERO_ROTATIONS 0
AccelStepper stepper(HALFSTEP, blue, yellow, pink, orange);

#define SERVO_PIN 10
#define SERVO_SPEED_SCAN 130
#define SERVO_SPEED_MOVE 20
/**
 * Timer kvuli ovladani rychlosti otaceni serva
 */
unsigned long servo_timer = 0;
/**
 * Cim mensi hodnota, tim rychlejsi pohyb. Jde o reset timeru
 */
unsigned int servo_speed = 100;
unsigned int servo_angle = 90;
unsigned int desired_servo_angle = 90;
Servo servo;

#define S_DONE 0
#define S_SCANNING 1
#define S_SET_DIRECTION 2
#define S_MOVE 3
/**
 * Aktualne bezici smycka
 */
unsigned int actual_step = S_SCANNING;

#define S_SCAN_START_POSITION 1
#define S_SCAN_SCANNING 2
#define S_SCAN_FINAL_POSITION 3
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define DIRECT_ANGLE 90
#define DIRECT_ANGLE_TOLERANCE 2
#define STEERING_SCAN_ANGLE_LOW 10
#define STEERING_SCAN_ANGLE_HIGH 80
/**
 * [uhel] => vzdalenost
 */
float distances[181];
/**
 * Aktualne bezici smycka mereni vzdalenosti
 */
unsigned int actual_scanning_step = S_SCAN_START_POSITION;
unsigned int scan_range_start = MIN_ANGLE;
unsigned int scan_range_stop = MAX_ANGLE;

#define S_DIRECTION_STEERING 1
#define S_DIRECTION_MOVING 2
/**
 * Aktualne bezici smycka zataceni
 */
unsigned int actual_direction_step = S_DIRECTION_MOVING;

#define ECHOPIN 7
#define TRIGPIN 8
#define ECHO_ENABLE_PIN 9
#define ECHO_DISTANCE_RATIO 0.017315f
#define ECHO_DELAY_MEAS 2
#define ECHO_DELAY_RESPONSE 10
#define ECHO_DELAY 50
float closest_distance = UNREACHABLE_DISTANCE_CM;
unsigned int closest_angle = DIRECT_ANGLE;

#define IGNITER_PIN 11

// *** Smycky
// pro PHPkare je to docela orisek. Vsude globalni promenne a uplne
// se nedari SRP. Bude treba refactoring

/**
 * Meri vzdalenost okolnich objektu
 * @todo refactoring needed
 * @global closest_distance
 * @global closest_angle
 * @global actual_scanning_step
 * @global actual_direction_step
 * @global scan_range_stop
 * @global scan_range_start
 * @global actual_step
 * @global servo_speed
 * @global distances
 */
void run_scanning();

/**
 * Pootoci robotem blize k neblizsimu objektu
 * @todo refactoring needed
 * @global closest_angle
 * @global closest_distance
 * @global actual_scanning_step
 * @global actual_direction_step
 * @global actual_step
 */
void run_set_direction();

/**
 * Robot je uz nasmerovan k neblizsimu objektu a dojede k nemu
 */
void run_final_move();

/**
 * Nastavi uhel serva danou rychlosti
 * @global servo_timer
 * @global servo_angle
 */
void servo_run(unsigned int angle, unsigned int speed);


// *** Inicializace

/**
 * Nastavi vychozi hodnoty rychlosti a akcelerace stepperu
 */
void init_stepper(unsigned int speed, unsigned int acceleration);

/**
 * Inicializace mereni vzdalenosti
 */
void init_echo();

// *** Funkce

/**
 * Pootoci stepperem o pocet otacek po smeru hod. rucicek.
 * Zaporne cislo = opacny smer
 */
void move_stepper_rotations(float rotations);

/**
 * Pootoci krokovym motorem, aby robot ujel vzdalenost v cm
 */
void move_stepper_cm(float cm);

/**
 * Nastavi uhel serva
 * @global desired_servo_angle
 */
void set_servo_angle(unsigned int angle);

/**
 * Vrati aktualni uhel serva
 * @global servo_angle
 */
unsigned int get_servo_angle();

/**
 * Vrati pozadovany cilovy uhel serva
 * @global desired_servo_angle
 */
unsigned int get_desired_servo_angle();

/**
 * Vrati vzdalenost objektu od senzoru v cm
 */
float get_distance_cm();

/**
 * Vrati o kolik otacek se ma pootocit stepper pro zatoceni
 */
float get_steering_step(int angle, float actual_distance);

/**
 * Nastavi rozpeti kde se bude merit vzdalenost
 * @global scan_range_stop
 * @global scan_range_start
 */
void set_scaning_range(int angle);

/**
 * Logika pro pricitani a odecitani uhlu
 */
unsigned int add_angle(unsigned int angle, int add);
