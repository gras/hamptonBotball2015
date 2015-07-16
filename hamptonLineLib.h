/*
Brandon Duderstadt
brandonduderstadt@gmail.com
Hampton Robotics 2015
*/

///NOTES:
///All library notes are denoted via triple slash, these are direct statements to the software/hardware team
///This library uses functions from the hamptonMovementLib.h... be sure to download this lib and put it in the same folder
///The variables at the beginnning of the forward and turning funcitons are hardware based as well as TICKS_PER_CM. They must be changed to fit your robot.
///All terms such as "right" and "left" are from the perspective of the robot facing forward
///All distances are in cm, all speeds are in % motor power, and all times are in ms
///All distances are approximate, as line following is curvy. Accordingly, sensor placement, correction speed, and initial error are all contributing to error in the final distance
///The global definition of black must be changed to fit each robot based on sensor placement and lighting conditions

#include "hamptonMovementLib.h"

//DEFINITIONS//

	//SENSORS
	#define RIGHT_LINE_SENSOR 0 //small hat
	#define LEFT_LINE_SENSOR 1 //small hat
	#define REAR_LINE_SENSOR 2 //top hat
	#define FAR_LEFT_LINE_SENSOR 3 //top hat
	
	//GLOBALS
	const int black = 700;
	

//PROTOTYPES//
void backward_follow_left(int dis, int spd, int comp);
void backward_follow_right(int dis, int spd, int comp);
void backward_follow_left_for(int time, int spd, int comp);
void backward_follow_right_for(int time, int spd, int comp);
void forward_follow_left(int dis, int spd, int comp);
void forward_follow_right(int dis, int spd, int comp);
void forward_follow_left_for(int dis, int spd, int comp);
void forward_follow_right_for(int dis, int spd, int comp);
void forward_follow_left_until_line(int spd, int comp);
void left_turn_until_line(int spd);
void right_turn_until_line(int spd);

//FUNCTIONS//

void backward_follow_left(int dis, int spd, int comp){
	
	//follows the left side of a line backwards for a set istance
	
	dis *= TICKS_PER_CM;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(abs(get_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR)) < dis){
		if(analog(REAR_LINE_SENSOR) < black){ //when the sensor is on white, arc towards the black on the robot's right
			motor(LEFT_MOTOR, -(spd));
			motor(RIGHT_MOTOR, -(spd-comp));
			}
		if(analog(REAR_LINE_SENSOR) > black){ //when the sensor is on black, arc towards the white on the robot's left
			motor(LEFT_MOTOR, -(spd-comp));
			motor(RIGHT_MOTOR, -(spd));
			}
			msleep(50); //change this to control how many cycles per second the loop runs
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}


void backward_follow_right(int dis, int spd, int comp){
	
	//follows the right side of a line backwards for a set distance
	
	dis *= TICKS_PER_CM;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(abs(get_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR)) < dis){
		if(analog(REAR_LINE_SENSOR) > black){ //when the sensor is on black, arc to the white on the robots' right
			motor(LEFT_MOTOR, -spd);
			motor(RIGHT_MOTOR, -(spd-comp));
			}
		if(analog(REAR_LINE_SENSOR) < black){ //when the sensor is on white, arc to the black on the robot's left
			motor(LEFT_MOTOR, -(spd-comp));
			motor(RIGHT_MOTOR, -(spd));
			}
			msleep(50); //change this to control how many cycles per second the loop runs
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}


void backward_follow_left_for(int time, int spd, int comp){
	
	//line follows backwards on the left side of the line for an amount of time
	
	int n = 0;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(50*n < time){ //the 50 here is associated with the cycles per second below, change if the cps is changed
		if(analog(REAR_LINE_SENSOR) < black){ //when the sensor is on white, arc towards the black on the robot's right
			motor(LEFT_MOTOR, -(spd));
			motor(RIGHT_MOTOR, -(spd-comp));
			}
		if(analog(REAR_LINE_SENSOR) > black){ //when the sensor is on black, arc towards the white on the robot's left
			motor(LEFT_MOTOR, -(spd-comp));
			motor(RIGHT_MOTOR, -(spd));
			}
			msleep(50); //change this to control how many cycles per second the loop runs
			n++;
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}

	
void backward_follow_right_for(int time, int spd, int comp){
	
	//follows the right side of a line backwards for a set amount of time
	
	int n = 0;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(50*n < time){ //the 50 here is associated with the cycles per second below, change if the cps is changed
		if(analog(REAR_LINE_SENSOR) > black){ //when the sensor is on black, arc to the white on the robots' right
			motor(LEFT_MOTOR, -spd);
			motor(RIGHT_MOTOR, -(spd-comp));
			}
		if(analog(REAR_LINE_SENSOR) < black){ //when the sensor is on white, arc to the black on the robot's left
			motor(LEFT_MOTOR, -(spd-comp));
			motor(RIGHT_MOTOR, -(spd));
			}
			msleep(50); //change this to control how many cycles per second the loop runs
			n++;
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}

void forward_follow_left(int dis, int spd, int comp){
	
	//follows the left side of a line for a set distance
	
	dis *= TICKS_PER_CM;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(get_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR) < dis){
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) > black){ //when the left sensor is on white and the right is on black
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) > black && analog(RIGHT_LINE_SENSOR) > black){ //when both sensors are on black, arc to the white on the robot's left
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			
			}
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //when both sensors are on white, arc to the black on the robot's right
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		if(analog(LEFT_LINE_SENSOR)>black && analog(RIGHT_LINE_SENSOR) < black){ //if the compensation does not fix the follow fast enough and the robot is shearing across the line
			printf("switched side of line");
			anticlockwise_turn(30,75,25,10);
			forward(7,75,25,10);//arguments on the commands in this case will change based on hardware and line thickness
			right_turn_until_line(900);
			}
			msleep(50);
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}
	
void forward_follow_right(int dis, int spd, int comp){
	
	//follows the right side of a line for a set distance
	
	dis *= TICKS_PER_CM;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(get_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR) < dis){
		if(analog(LEFT_LINE_SENSOR) > black && analog(RIGHT_LINE_SENSOR) < black){ //when the left sensor is on black and the right is on white
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			
			}
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //when both sensors are on white, arc to the black on the robot's left
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			
			}
		if(analog(LEFT_LINE_SENSOR) >black && analog(RIGHT_LINE_SENSOR) > black){ //when both sensors are on black, arc to the white on the robot's right
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		if(analog(LEFT_LINE_SENSOR)<black && analog(RIGHT_LINE_SENSOR) > black){ //if the compensation does not fix the follow fast enough and the robot is shearing across the line
			printf("switched side of line");
			clockwise_turn(30, 75, 25, 10);
			forward(7,75, 25, 10);//arguments on the commands in this case will change based on hardware and line thickness
			left_turn_until_line(900);
			}
			msleep(50); //change this to control how many cycles per second the loop runs
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}

void forward_follow_left_for(int time, int spd, int comp){
	
	//follows the left side of a line for an amount of time
	
	int n = 0;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(50 * n < time){ //the 50 here is associated with the cycles per second below, change if the cps is changed
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) > black){ //when the left sensor is on white and the right is on black
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) > black && analog(RIGHT_LINE_SENSOR) > black){ //when both sensors are on black, arc to the white on the robot's left
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //when both sensors are on white, arc to the black on the robot's right
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		if(analog(LEFT_LINE_SENSOR)>black && analog(RIGHT_LINE_SENSOR) < black){ //if the compensation does not fix the follow fast enough and the robot is shearing across the line
			printf("switched side of line");
			anticlockwise_turn(30, 75, 25, 10);
			forward(7,75, 25, 10);//arguments on the commands in this case will change based on hardware and line thickness
			right_turn_until_line(90);
			}
			msleep(50); //change this to control how many cycles per second the loop runs
			n++;
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);	
	}

	
void forward_follow_right_for(int time, int spd, int comp){
	
	//follows the right side of a line for an amount of time
	
	int n = 0;
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(50 * n < time){ //the 50 here is associated with the cycles per second below, change if the cps is changed
		if(analog(LEFT_LINE_SENSOR) > black && analog(RIGHT_LINE_SENSOR) < black){ //when the left sensor is on black and the right is on white
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //when both sensors are on white, arc to the black on the robot's left
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) >black && analog(RIGHT_LINE_SENSOR) > black){ //when both sensors are on black, arc to the white on the robot's right
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		if(analog(LEFT_LINE_SENSOR)<black && analog(RIGHT_LINE_SENSOR) > black){ //if the compensation does not fix the follow fast enough and the robot is shearing across the line
			printf("switched side of line");
			clockwise_turn(30, 75, 25, 10);
			forward(7,75, 25, 10);//arguments on the commands in this case will change based on hardware and line thickness
			left_turn_until_line(90);
			}
			msleep(50); //change this to control how many cycles per second the loop runs
			n++;
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}


void forward_follow_left_until_line(int spd, int comp){
	
	//follows the left side of a line until there is an intersection
	///NOTE: the sensor to detect intersections was on the left of my robot, making the forward_follow_left_until_line(int dis, int spd, int comp)
	///the more effective function for moving until an intersection. If you understand this library, you'll have no trouble moving on the right of
	///a line until an intersection
	
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	while(analog(FAR_LEFT_LINE_SENSOR) < black){
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) > black){ //when the left sensor is on white and the right is on black
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			}
		if(analog(LEFT_LINE_SENSOR) > black && analog(RIGHT_LINE_SENSOR) > black){ //when both sensors are on black, arc to the white on the robot's left
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			
			}
		if(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //when both sensors are on white, arc to the black on the robot's right
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		if(analog(LEFT_LINE_SENSOR)>black && analog(RIGHT_LINE_SENSOR) < black){ //if the compensation does not fix the follow fast enough and the robot is shearing across the line
			printf("switched side of line");
			anticlockwise_turn(30,75,25,10);
			forward(7,75, 25, 10);//arguments on the commands in this case will change based on hardware and line thickness
			right_turn_until_line(900);
			}
			msleep(50);
		}
		freeze(LEFT_MOTOR);
		freeze(RIGHT_MOTOR);
	}

void left_turn_until_line(int spd){
	
	//turns the robot left until it detets a line
	
	motor(RIGHT_MOTOR, spd);
	motor(LEFT_MOTOR, -spd);
	while(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //while neither sensor sees black
		msleep(20); //change this to control how many cycles per second the loop runs
		}
	freeze(RIGHT_MOTOR);
	freeze(LEFT_MOTOR);
	}
	
void right_turn_until_line(int spd){
	
	//turns the robot right until it detets a line
	
	motor(RIGHT_MOTOR, -spd);
	motor(LEFT_MOTOR, spd);
	while(analog(LEFT_LINE_SENSOR) < black && analog(RIGHT_LINE_SENSOR) < black){ //while neither sensor sees black
		msleep(20); //change this to control how many cycles per second the loop runs
		}
	freeze(RIGHT_MOTOR);
	freeze(LEFT_MOTOR);
	}
