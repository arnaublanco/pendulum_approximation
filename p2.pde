
// SET INITIAL VARIABLES. Question 2.1.1
// INSERT YOUR CODE HERE.
float Theta=PI/10;
float DThetaDt=0.0;


//////////////////////////////////////////////////////////////////////////////

// STATE DEFINITION. Question 2.1.2
// INSERT YOUR CODE HERE

int nvariables=3;//number of variables
float[] vector_state=new float[nvariables];//this is the state vector that will update every time the time changes
float[] initState=new float [nvariables]; //the vector with the initial value of the variables
float[] vector_state1=new float[nvariables];//euler-cromer
float[] vector_state2=new float[nvariables];//rk2
float[] vector_state3=new float[nvariables];//rk4
//the index for every variable
int stateTheta=1;
int stateTime=0;
int stateDThetaDt=2;

//////////////////////////////////////////////////////////////////////////////

//Window set up variables
int WindowWidthHeight = 350;
float WorldSize = 2.0;
float PixelsPerMeter;
float OriginPixelsX;
float OriginPixelsY;

//Global constants
float g = 9.8;// gravity = 9.8 m/s^2
float PendulumLength = 1.0;// Length of the pendulum arm, in meters. This is constant, and therefore not really part of the simulation state


void setup()
{
    // CREATE INITIAL STATES. Question 2.1.3
    // INSERT YOUR CODE HERE
    
    initState[stateTime]=0.0;
    initState[stateTheta]=Theta;
    initState[stateDThetaDt]=DThetaDt;
    for ( int i = 0; i < nvariables; ++i )
    {
     vector_state[i] = initState[i];
     vector_state1[i] = initState[i];
     vector_state2[i] = initState[i];
     vector_state3[i] = initState[i];
    }

   
       
/////////////////////////////////////////////////////////////////////////////    
   
    // Set up normalized colors.
    colorMode( RGB, 1.0 );
    
    // Set up the stroke color and width.
    stroke( 0.0 );
    //strokeWeight( 0.01 );
    
    // Create the window size, set up the transformation variables.
    size( 720, 720 );
    PixelsPerMeter = (( float )WindowWidthHeight ) / WorldSize;
    OriginPixelsX = 0.25 * ( float )WindowWidthHeight;
    OriginPixelsY = 0.25 * ( float )WindowWidthHeight;
  
    
}
float A_from_X( float i_x )
{
return -( g / PendulumLength ) * i_x;
}


void timeStep(float delta_t){
  
  // EULER METHOD. Question 2.2.2
  // INSERT YOUR CODE HERE
  //first we compute acceleration.
  float A;
  A=-(g/PendulumLength)*vector_state[stateTheta];
  //now we update the position from the current velocity
  vector_state[stateTheta] += delta_t * vector_state[stateDThetaDt];
  //update state velocity
  vector_state[stateDThetaDt] += delta_t * A;
//update time
 vector_state[stateTime] +=  delta_t;

  

  
/////////////////////////////////////////////////////////////////////////////
  
  // EULER-CROMER METHOD. Question 2.2.2
  // INSERT YOUR CODE HERE
  float A1;
  A1=-(g/PendulumLength)*vector_state1[stateTheta];
  //update state velocity first
  vector_state1[stateDThetaDt] += delta_t * A1;
 // now we update the position from the current velocity
  vector_state1[stateTheta] += delta_t * vector_state1[stateDThetaDt];
//update time
 vector_state1[stateTime] +=  delta_t;


  
/////////////////////////////////////////////////////////////////////////////
  
  // RK_2. Question 2.2.2
  // INSERT YOUR CODE HERE
  float vStar1 = vector_state2[stateDThetaDt];
  float aStar1 = A_from_X( vector_state2[stateTheta] );
  float vStar2 = vector_state2[stateDThetaDt] + ( delta_t * aStar1 );
  float xTmp = vector_state2[stateTheta] + ( delta_t * vStar1 );
  float aStar2 = A_from_X( xTmp );
  vector_state2[stateTheta] += ( delta_t / 2.0 ) * ( vStar1 + vStar2 );
 vector_state2[stateDThetaDt] += ( delta_t / 2.0 ) * ( aStar1 + aStar2 );
//update time
 vector_state2[stateTime] +=  delta_t;



  
/////////////////////////////////////////////////////////////////////////////
  
  // RK_4. Question 2.2.2
  // INSERT YOUR CODE HERE
  float vStar10 =vector_state3[stateDThetaDt];
  float aStar10 = A_from_X( vector_state3[stateTheta] );
  float vStar20 = vector_state3[stateDThetaDt] + ( ( delta_t / 2.0 ) * aStar10 );
  float xTmp2 = vector_state3[stateTheta] + ( ( delta_t / 2.0 ) * vStar10 );
  float aStar20 = A_from_X( xTmp2 );
  float vStar3 = vector_state3[stateDThetaDt] + ( ( delta_t / 2.0 ) * aStar20 );
  float xTmp3 = vector_state3[stateTheta] + ( ( delta_t / 2.0 ) * vStar20 );
  float aStar3 = A_from_X( xTmp3 );
  float vStar4 = vector_state3[stateDThetaDt]+ ( delta_t * aStar3 );
  float xTmp4 = vector_state3[stateTheta] + ( delta_t * vStar3 );
  float aStar4 = A_from_X( xTmp4 );
 vector_state3[stateTheta] += ( delta_t / 6.0 ) *( vStar10 + (2.0*vStar20) + (2.0*vStar3) + vStar4 );
vector_state3[stateDThetaDt] += ( delta_t / 6.0 ) *( aStar10 + (2.0*aStar20) + (2.0*aStar3) + aStar4 );
  //update time
 vector_state3[stateTime] +=  delta_t;

  
/////////////////////////////////////////////////////////////////////////////

}


// The DrawState function assumes that the coordinate space is that of the
// simulation - namely, meters, with the pendulum pivot placed at the origin.
// Draw the arm, pivot, and bob!
// There is currently a bug in processing.js which requires us to do the
// pixels-per-meter scaling ourselves.
void DrawState()
{

   float sqrtGoverL = sqrt( g / PendulumLength );
   float x0 = initState[stateTheta];
   float v0 = initState[stateDThetaDt];
   float t = vector_state[stateTime];
   float CorrectTheta = ( x0 * cos( sqrtGoverL * t ) ) + ( ( v0 / sqrtGoverL ) * sin( sqrtGoverL + t ) );
    // DRAW THE EULER SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE
    float ArmEndX = PixelsPerMeter * PendulumLength * sin(vector_state[stateTheta]);//Replace the zero inside the parenthesis with your code
    float ArmEndY = PixelsPerMeter * PendulumLength * cos(vector_state[stateTheta]);//Replace the zero inside the parenthesis with your code

    // Draw the pendulum arm.
    strokeWeight( 1.0 );
    line( 0.0, 0.0, ArmEndX, ArmEndY );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 0.0, 0.0, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the pendulum bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( ArmEndX, ArmEndY, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );

  
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
   
    

    // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE EULER SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE
     float ArmEndX0 = PixelsPerMeter * PendulumLength * sin(CorrectTheta);//Replace the zero inside the parenthesis with your code
    float ArmEndY0 = PixelsPerMeter * PendulumLength * cos(CorrectTheta)+20;//Replace the zero inside the parenthesis with your code

     strokeWeight( 1.0 );
    line( 0.0, 20, ArmEndX0, ArmEndY0 );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 0.0, 20, PixelsPerMeter * 0.03, PixelsPerMeter * 0.03 );
    // Draw the pendulum bob
    fill( 1.0, 0.0, 220.0 );
    ellipse( ArmEndX0, ArmEndY0,  PixelsPerMeter * 0.1, PixelsPerMeter * 0.1 );
    

    
/////////////////////////////////////////////////////////////////////////////
    
    // DRAW THE EULER-CROMER SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE
     float ArmEndX1 =PixelsPerMeter * PendulumLength * sin(vector_state1[stateTheta])+500;//Replace the zero inside the parenthesis with your code
    float ArmEndY1= PixelsPerMeter * PendulumLength * cos(vector_state1[stateTheta]);//Replace the zero inside the parenthesis with your code

    // Draw the pendulum arm.
    strokeWeight( 1.0 );
    line( 500, 0.0, ArmEndX1, ArmEndY1 );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 500, 0.0, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the pendulum bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( ArmEndX1, ArmEndY1, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );
    

      
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           

    // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE EULER-CROMER SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE   
  
     strokeWeight( 1.0 );
    line( 500, 20, ArmEndX0+500, ArmEndY0 );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 500, 20, PixelsPerMeter * 0.03, PixelsPerMeter * 0.03 );
    // Draw the pendulum bob
    fill( 1.0, 0.0, 220.0 );
    ellipse( ArmEndX0+500, ArmEndY0,  PixelsPerMeter * 0.1, PixelsPerMeter * 0.1 );
    

             
/////////////////////////////////////////////////////////////////////////////
    
    // DRAW THE RK_2. Question 2.2.3
    // INSERT YOUR CODE HERE   
      float ArmEndX2 =PixelsPerMeter * PendulumLength * sin(vector_state2[stateTheta]);//Replace the zero inside the parenthesis with your code
    float ArmEndY2= PixelsPerMeter * PendulumLength * cos(vector_state2[stateTheta])+300;//Replace the zero inside the parenthesis with your code

    // Draw the pendulum arm.
    strokeWeight( 1.0 );
    line( 0.0, 300, ArmEndX2, ArmEndY2 );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 0.0, 300, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the pendulum bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( ArmEndX2, ArmEndY2, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );
    

      
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     

    // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE RK-2 SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE    
     strokeWeight( 1.0 );
    line( 0.0, 320, ArmEndX0, ArmEndY0+300);
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 0.0, 320, PixelsPerMeter * 0.03, PixelsPerMeter * 0.03 );
    // Draw the pendulum bob
    fill( 1.0, 0.0, 220.0 );
    ellipse( ArmEndX0, ArmEndY0+300,  PixelsPerMeter * 0.1, PixelsPerMeter * 0.1 );
    
    
    
/////////////////////////////////////////////////////////////////////////////
    
    // DRAW THE RK_4. Question 2.2.3
    // INSERT YOUR CODE HERE   
      float ArmEndX3 =PixelsPerMeter * PendulumLength * sin(vector_state3[stateTheta])+500;//Replace the zero inside the parenthesis with your code
    float ArmEndY3= PixelsPerMeter * PendulumLength * cos(vector_state3[stateTheta])+300;//Replace the zero inside the parenthesis with your code

    // Draw the pendulum arm.
    strokeWeight( 1.0 );
    line( 500, 300, ArmEndX3, ArmEndY3 );
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 500, 300, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the pendulum bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( ArmEndX3, ArmEndY3, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );
    

      
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      

    // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE RK-4 SOLUTION. Question 2.2.3
    // INSERT YOUR CODE HERE    
      strokeWeight( 1.0 );
    line( 500, 320, ArmEndX0+500, ArmEndY0+300);
          
    // Draw the pendulum pivot
    fill( 0.0 );
    ellipse( 500, 320, PixelsPerMeter * 0.03, PixelsPerMeter * 0.03 );
    // Draw the pendulum bob
    fill( 1.0, 0.0, 220.0 );
    ellipse( ArmEndX0+500, ArmEndY0+300,  PixelsPerMeter * 0.1, PixelsPerMeter * 0.1 );
    
    
   
    
/////////////////////////////////////////////////////////////////////////////    
             
}
    
  


// The draw function creates a transformation matrix between pixel space
// and simulation space, in meters, and then calls the DrawState function.
// Unfortunately, there is currently a bug in processing.js with concatenated
// transformation matrices, so we have to do the coordinate scaling ourselves
// in the draw function.
void draw()
{
    //Time Step
    timeStep(1.0/24.0);
  
    //Clear the display to a constant color
    background( 0.75 );
   line(WindowWidthHeight,0.0,WindowWidthHeight,800);
   line(0.0,WindowWidthHeight,800,WindowWidthHeight);
    
    // DRAW THE QUADRANTS AND NAMES. Question 2.3.1
    // INSERT YOUR CODE HERE
    // Add additional code here (if not done in the DrawState() function) to draw the quadrants and names.        
    

    
    /////////////////////////////////////////////////////////////////////////////////////

    // Translate to the origin.
    translate( OriginPixelsX, OriginPixelsY );

    // Draw the simulation
    DrawState();
}