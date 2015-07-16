/*
	Brandon Duderstadt
	brandonduderstadt@gmail.com
	Hampton Robotics 2105
*/

///NOTES:
///All library notes are denoted via triple slash, these are direct statements to the software/hardware team
///LET THE FINCTIONS DO THEIR JOBS! The larger the comp, the harder the correcton, and the stronger the follow
///For the forward and turning functions, getting accurate encoder measurements of ticks per rotation is VITAL!!!!!!!
///If you do not do this, the forward will overcorrect, and cause the robot to arc dramatically due to built up error

//DEFINITIONS
	//MOTORS
	#define RIGHT_MOTOR 0 //+ is forward
	#define LEFT_MOTOR 1 //+ is forward
	
	//CONSTANTS
	#define TICKS_PER_CM 50
	#define PI 3.14159

//PROTOTYPES
int abs(int n);
void anticlockwise_turn(int dis, int spd, int comp, int thresh);
void clockwise_turn(int dis, int spd, int comp, int thresh);
void forward(int dis, int spd, int comp, int thresh);
int get_absolute_average_motor_position_counter(int motor1, int motor2);
int get_average_motor_position_counter(int motor1, int motor2);

//FUNCTIONS

int abs(int n){
	if(n>0){
		return n;
		}
	else{
		return -n;
		}
	}
	
	
void anticlockwise_turn(int deg, int spd, int comp, int thresh){
	
	//This is an attempt to use motor control for turning
	float fDeg = (float) deg;
	float fThresh=(float)thresh * .01; //Thresh is in 100ts of a rotation
	float wheelBase = 18.0; ///this is the distance between the wheels in cm
	int error = 0;
	int lCount = 0;
	int rCount = 0;
	int lTicks = 409; ///Use the motor position gui to find the ticks per rotation
	int rTicks = 496; ///Use the motor position gui to find the ticks per rotation
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	int dis = (int)((PI* wheelBase) * (abs(fDeg)/360.0) * TICKS_PER_CM);
	printf("%d\n");
	while(get_absolute_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR) < dis){ //the robot has not reached its destination
		lCount =(int) (get_motor_position_counter(LEFT_MOTOR) * (1.0/lTicks)); //convert to rotations
		rCount =(int) (get_motor_position_counter(RIGHT_MOTOR) * (1.01/rTicks)); //convert to rotations
		error += (rCount - lCount); //calculate the cumulative error, the "I" constant
		if(error>fThresh){//if the right motor has gone farther, causing a leftward arc
			printf("arcing left, correcting right\n");
			motor(LEFT_MOTOR, -(spd));
			motor(RIGHT_MOTOR, (spd-comp));
			}
		else if(error < -fThresh){//if the left motor has gone farther, causing a rightward arc
			printf("arcing right, correcting left\n");
			motor(LEFT_MOTOR, -(spd-comp));
			motor(RIGHT_MOTOR, (spd));
			}
		else{
			printf("on track, no correction\n");
			motor(LEFT_MOTOR, -(spd));
			motor(RIGHT_MOTOR, (spd));
			}
			msleep(50); //cycles 20 times per second
		}
		freeze(RIGHT_MOTOR);
		freeze(LEFT_MOTOR);
	}

void clockwise_turn(int deg, int spd, int comp, int thresh){
	
	//This is an attempt to use motor control for turning
	float fDeg = (float) deg;
	float fThresh= (float) thresh *.01; //Thresh is in 100ts of a rotation
	float wheelBase = 18.0; ///this is the distance between the wheels in cm
	int error = 0;
	int lCount = 0;
	int rCount = 0;
	int lTicks = 409; ///Use the motor position gui to find the ticks per rotation
	int rTicks = 496; ///Use the motor position gui to find the ticks per rotation
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	int dis = (int)((PI* wheelBase) * (abs(fDeg)/360.0) * TICKS_PER_CM);
	while(get_absolute_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR) < dis){ //the robot has not reached its destination
		lCount = (int) (get_motor_position_counter(LEFT_MOTOR) * (1.0/lTicks)); //convert to rotations
		rCount = (int) (get_motor_position_counter(RIGHT_MOTOR) * (1.0/rTicks)); //convert to rotations
		error += (rCount - lCount); //calculate the cumulative error, the "I" constant
		if(error>fThresh){//if the right motor has gone farther, causing a leftward arc
			printf("arcing left, correcting right\n");
			motor(LEFT_MOTOR, (spd));
			motor(RIGHT_MOTOR, -(spd-comp));
			}
		else if(error < -fThresh){//if the left motor has gone farther, causing a rightward arc
			printf("arcing right, correcting left\n");
			motor(LEFT_MOTOR, (spd-comp));
			motor(RIGHT_MOTOR, -(spd));
			}
		else{
			printf("on track, no correction\n");
			motor(LEFT_MOTOR, (spd));
			motor(RIGHT_MOTOR, -(spd));
			}
			msleep(50); //cycles 20 times per second
		}
		freeze(RIGHT_MOTOR);
		freeze(LEFT_MOTOR);
	}

	
void forward(int dis, int spd, int comp, int thresh){
	
	//This is the motor control function that keeps our robot driving mostly straight. Enjoy.
	
	float fThresh = (float) .001* thresh; //thresh is in 1000ths of a rotation
	int error = 0;
	int lCount = 0;
	int rCount = 0;
	int lTicks = 409; ///Use the motor position gui to find the ticks per rotation
	int rTicks = 496; ///Use the motor position gui to find the ticks per rotation
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	dis*= TICKS_PER_CM;
	while(get_average_motor_position_counter(LEFT_MOTOR, RIGHT_MOTOR) < dis){ //the robot has not reached its destination
		lCount =(int) (get_motor_position_counter(LEFT_MOTOR) * (1.0/lTicks)); //convert to rotations
		rCount =(int) (get_motor_position_counter(RIGHT_MOTOR) * (1.0/rTicks)); //convert to rotations
		error += (rCount - lCount); //calculate the cumulative error, the "I" constant
		if(error>fThresh){//if the right motor has gone farther, causing a leftward arc
			printf("arcing left, correcting right\n");
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd-comp);
			}
		else if(error < -fThresh){//if the left motor has gone farther, causing a rightward arc
			printf("arcing right, correcting left\n");
			motor(LEFT_MOTOR, spd-comp);
			motor(RIGHT_MOTOR, spd);
			}
		else{
			printf("on track, no correction\n");
			motor(LEFT_MOTOR, spd);
			motor(RIGHT_MOTOR, spd);
			}
			msleep(50); //cycles 20 times per second
		}
		freeze(RIGHT_MOTOR);
		freeze(LEFT_MOTOR);
	}
	

int get_absolute_average_motor_position_counter(int motor1, int motor2){
	
	//returns the scalar of average motor movement
	
	return ((abs(get_motor_position_counter(motor1)) + abs(get_motor_position_counter(motor2)))/2);
	}

int get_average_motor_position_counter(int motor1, int motor2){
	
	//this is a simple calculation to estimate line follow distances
	
	return ((get_motor_position_counter(motor1) + get_motor_position_counter(motor2))/2);
	}
