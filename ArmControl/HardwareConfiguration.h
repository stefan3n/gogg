#include <Servo.h>

//Define pin values

// Actuators:

// (1) First actuator: Input1, Input2, Enable
#define IN1_A1 5
#define IN2_A1 4
#define ENA_A1 3 

// (2) Second actuator: Input1, Input2, Enable
#define IN1_A2 8
#define IN2_A2 7
#define ENA_A2 6 

// (3) Third actuator: Input1, Input2, Enable
#define IN1_A3 12
#define IN2_A3 10
#define ENA_A3 9 

// Actuator potentiometer feedback:
#define FEEDBACK_A1 A0 // (1) First actuator
#define FEEDBACK_A2 A1 // (2) Second actuator
#define FEEDBACK_A3 A2 // (3) Third actuator


// Stepper:
#define DIRECTION_STEPPER 2 // Direction
#define PULSE_STEPPER 3 // Pulse
#define ENA_STEPPER 4 // Enable


// Claw:   ====== NOT SET ======
#define ROTATE_CLAW  
#define CLAW  


// Others:
#define ERROR_MARGIN 5

// Id to use instead of name
#define ACTUATOR1 1
#define ACTUATOR2 2
#define ACTUATOR3 3

// Delay for stepper
#define stepDelay 500

// Arm blocked after CMD_FINAL_ARM
#define BlockingTime 5

/* === OLD KINEMATICS ===

 extern double d1; // Distance from intersection point between first actuator and bottom horizontal link and first join
extern double d2; // Distance form intersection point between first actuator and vertical link and first join
extern double d3; // Distance from intersection point between second actuator and vertical link and second join
extern double d4; // Distance from intersection point between second actuator and upper horizontal link and second join  
extern double Al1; // the lenght of the first actuator
extern double Al2; // the lenght of the second actuator

extern double l1; // the length of the first link
extern double l2; // the lenght of the second link
*/

// === NEW KINEMATICS === 

extern double l1; // Length of the (1) first link - closest to base
extern double l2; // Length of the (2) second link - middle
extern double l3; // length of the (3) third link - closest to claw



// New position
extern int target_position1; // Target - Used for (1) first actuator
extern int target_position2; // Target - Used for (2) second actuator
extern int target_position3; // Target - Used for (3) third actuator

// Default actuators position
extern int default_position1; // Default - Used for (1) first actuator
extern int default_position2; // Default - Used for (2) second actuator
extern int default_position3; // Default - Used for (3) third actuator


// Current actuator position
extern int feedback_value1;
extern int feedback_value2;
extern int feedback_value3

extern int stepTarget; // Position
extern int stepCount; // Steps after command is sent
extern int startPosition; // Initial position of the stepper
extern int direction; // Number of steps relative at the initial position
extern int pos1; // Initial position of the (1) first actuator
extern int pos2; // Initial position of the (2) second actuator
extern int pos3; // Initial position of the (3) third actuator 
extern Servo angle; // Command the claw
extern Servo angle_claw; // Rrotate the claw
extern bool no_calibrate; // Calibrate arm manually

extern double heigh_down_limit; // Prevent the claw to hit the ground
extern bool command; //  Keep last command
extern long time; // Arm will not respond
extern int time_claw; // Close the claw after a specific time
extern bool close;
extern bool emergency; // 1 = Emergency state | 0 = Not emergency state
