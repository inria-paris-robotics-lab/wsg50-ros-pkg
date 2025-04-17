//======================================================================
/**
 *  @file
 *  functions.h
 *
 *  @section testing.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	30.04.2012
 *  
 *  
 *  @section testing.h_copyright Copyright
 *  
 *  Copyright 2012 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

 #include <string>

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------



/*#ifdef __cplusplus
extern "C" {
#endif */

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------
typedef struct {
	unsigned int state;
	bool ismoving;
	float position, speed;
	float f_motor, f_finger0, f_finger1;
	std::string state_text;
} gripper_response;

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------



//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

float convert(unsigned char *b);
int homing( bool ignore_response = false );
int move(float width, float speed, bool stop_on_block, bool ignore_response = false);
int stop( bool ignore_response = false );
int grasp( float objWidth, float speed, bool ignore_response = false);
int release( float width, float speed, bool ignore_response = false);
int ack_fault(bool ignore_response = false);

int setAcceleration( float acc, bool ignore_response = false );
int setGraspingForceLimit( float force, bool ignore_response = false );
int doTare( bool ignore_response = false );

const char * systemState( void );
int graspingState( void );
float getOpening(int auto_update = 0);
float getForce(int auto_update = 0);
float getSpeed(int auto_update = 0);
int getAcceleration( void );
int getGraspingForceLimit( void );

int script_measure_move (unsigned char cmd_type, float cmd_width, float cmd_speed, gripper_response & info);

//void getStateValues(); //(unsigned char *);

//void test( void );


/*#ifdef __cplusplus
}
#endif */

#endif /* FUNCTIONS_H_ */
