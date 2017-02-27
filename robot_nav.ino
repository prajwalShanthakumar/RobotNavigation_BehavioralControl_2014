// test 5

//#define Behaviour 13                  // led on pin 13; indicates the mode the robot is in.

// Obstacle sensing parameters
#define maxDist 35                   // cm 
#define sensingThreshold 34
#define wallDistance 25
#define distanceScaleFactor 58.2     // for converting duration to cm
#define echoTimeout 2200             // 3.0 ms
#define triggerPin 14
#define lostwallDriveRight 55
#define lostwallDriveLeft 20


// Obstacle sensors
#define left_90_sensor 11
#define left_60_sensor 8
#define left_30_sensor 7
#define middle_sensor 6
#define right_30_sensor 5                                                    
#define right_60_sensor 4
#define right_90_sensor 3

// Wheel encoder sensors
#define leftsensor 12
#define rightsensor 2

// Motor pins
#define LPWM 10
#define RPWM 9
//#define Lgnd 5 
//#define Rgnd 6

// Robot physical parameters
#define R 3.5
#define L 20
#define ticksperev 48



//Misc parameters
#define pWeight 2                       
#define tWeight 1

#define w1 1                           
#define w2 1
#define w3 1                                        
#define w4 0
#define w5 1
#define w6 1
#define w7 1

#define errpos 15
#define exitFWthreshold PI/2 
//#define entryFWthreshold 
#define AOcorrection PI/12.0
#define velocity 100
#define vMax 1.3*velocity
#define vMin 0.3*velocity
#define ticksduration 20000                

//PID parameters                      
#define Kp (15.0 * velocity / 100.0)
#define Ki (4.0 * velocity / 100.0)
#define Kd (35.0 * velocity / 100.0)
#define Kpao (30.0 * velocity / 100.0)
#define Kiao 0
#define Kdao 0
//#define Kpfw (15.0 * velocity / 100.0)
#define Kifw 0
#define Kdfw 0

// Loop counting variables
int r,c; 
float Kpfw;

// Mode indication variables
int current_mode, FWdirection;
char previously, prev_middle;
// Transformation matrix variables
float tm1_1[3][3];                             // Sensor frame -> Robot frame transfirmation matrix
float tm1_2[3][3];
float tm1_3[3][3];
float tm1_4[3][3];
float tm1_5[3][3];
float tm1_6[3][3];
float tm1_7[3][3];
float tm2[3][3];                               // Robot frame -> World frame transfirmation matrix


// Obstacle sensing matrices


float left_60_sf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}};   
float left_30_sf[3][3] = {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_60_sf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_30_sf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float left_90_sf[3][3] = {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_90_sf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float middle_sf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 

float left_60_rf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}};    
float left_30_rf[3][3] = {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_60_rf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_30_rf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float left_90_rf[3][3] = {{0,0,0},{0,0,0},{1,0,0,}}; 
float right_90_rf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 
float middle_rf[3][3]= {{0,0,0},{0,0,0},{1,0,0,}}; 


float obstacle_rf[3][3];

// Follow wall variables
float p1[2][1]  = {{0.0},{0.0}};
float p2[2][1] = {{0.0},{0.0}};
float ua[2][1] = {{0.0},{0.0}};
float up[2][1] = {{0.0},{0.0}};
float u_fw[2][1]= {{0.0},{0.0}};
float u_fw_tp[2][1] = {{0.0},{0.0}};
float u_fw_t[2][1]  = {{0.0},{0.0}};
float u_fw_pp[2][1]= {{0.0},{0.0}};
float u_fw_p[2][1]= {{0.0},{0.0}};
float u_fw_pu[2][1]= {{0.0},{0.0}};


// Sensor positions withrespect to the robot frame (Can make them #define)
float xs[8];    
float ys[8];
float phis[8];

// Misc robot parameters and control variables
int  vr, vl, comeback, H[10], K[10], numcordinates, n, current_coordinate;
int destFound, PositionNotImproving ;
float v, phidest, oldphibot, phibot, x, y, Dl, Dr, Dc, Nl, Nr, orientation_error[6], P, I, D, omega, avoidAngle, followWallDecider;
float displacement, prevDisplacement;

//Wheel encoder variables
int flagL, flagR;
float count[2];

//Variables for timing
unsigned long starttime, timetaken;
int temp, z;







void setup()
{ 
   Serial.begin(115200);
  
  pinMode(leftsensor, INPUT);
  pinMode(rightsensor, INPUT);
  
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);

  
  pinMode(triggerPin, OUTPUT);
  pinMode(left_90_sensor, INPUT);
  pinMode(left_60_sensor, INPUT);
  pinMode(left_30_sensor, INPUT);
  pinMode(right_30_sensor, INPUT);
  pinMode(right_60_sensor, INPUT);
  pinMode(right_90_sensor, INPUT);
  pinMode(middle_sensor, INPUT);
  
//  pinMode(Behaviour, OUTPUT);
  
  prev_middle = 0;
  current_coordinate = 1;
  temp = 0;
  H[1] = 100;
  H[2] = 0;
  H[3] = 0;
  H[4] = 0;
  K[1] = 0;
  K[2] = 0;
  K[3] = 0;
  K[4] = 0;
  numcordinates = 1;
  
//  Serial.println("enter number of coordinates to traverse");
//  while(temp == 0)
//  {
//  if (Serial.available() > 0)
//  {
//  temp = 1;
//  numcordinates = Serial.read();
//  }
//  }
//  temp == 0;
//  
// 
//  
//  for (z = 0; z < numcordinates; z++)
//  {
//  Serial.println("enter next desired X coordinate");
//    while(temp == 0)
//  {
//  if (Serial.available() > 0)
//  {
//  temp = 1;
//  H[z+1] = Serial.read();
//  }
//  }
//  temp == 0;
//  
//    Serial.println("enter next desired Y coordinate");
//    while(temp == 0)
//  {
//  if (Serial.available() > 0)
//  {
//  temp = 1;
//  K[z+1] = Serial.read();
//  }
//  }
//  temp == 0;
//  }


//
  x = 0;
  y = 0;
  oldphibot = 0;
  phibot = 0;
  resetParameters();
//  
  phidest = atan2(K[current_coordinate],H[current_coordinate]);
  orientation_error[0] = phidest - phibot;
//  
  xs[1] = (-7.5 );                  
  xs[2] = (0.0 ); 
  xs[3] = (3.5 );
  xs[4] = (5.0 );
  xs[5] = (3.5 );    
  xs[6] = (0.0 );
  xs[7] = (-7.5 );
//                                                              
  ys[1] = 7.5;
  ys[2] = 8.5;
  ys[3] = 6.0;
  ys[4] = 0.0;
  ys[5] = -6.0;
  ys[6] = -8.5;
  ys[7] = -7.5;
//  
  phis[1] = PI/2.0;
  phis[2] = PI/3.0;
  phis[3] = PI/6.0;
  phis[4] = 0.0;
  phis[5] = -PI/6.0; //Left
  phis[6] = -PI/3.0;
  phis[7] = -PI/2.0;
  
  current_mode = -1;
  FWdirection = -1;
  v = velocity * R;  
// 
   Transformation_Matrix(tm1_1, xs[1], ys[1], phis[1]); 
   Transformation_Matrix(tm1_2, xs[2], ys[2], phis[2]);
   Transformation_Matrix(tm1_3, xs[3], ys[3], phis[3]);
   Transformation_Matrix(tm1_4, xs[4], ys[4], phis[4]);
   Transformation_Matrix(tm1_5, xs[5], ys[5], phis[5]);
   Transformation_Matrix(tm1_6, xs[6], ys[6], phis[6]);
   Transformation_Matrix(tm1_7, xs[7], ys[7], phis[7]);
 
 
   destFound = 0;
   PositionNotImproving = 0;
   delay(3000);

//  digitalWrite(Lgnd, LOW);
//  digitalWrite(Rgnd, LOW);

}






void loop()

{
        starttime = micros();
      //  Serial.flush();
     //    Serial.println(freeRam());
         //starttime = micros();

         if(destFound == 1)
         prevDisplacement = displacement;
         
         displacement = sqrt( pow( (x-H[current_coordinate]), 2) + pow( (y-K[current_coordinate]), 2) );
         
         if( (destFound == 1) && (displacement - prevDisplacement) > 0 )
         PositionNotImproving = 1;
         
         if(displacement < errpos )
         destFound = 1;
          

                if(PositionNotImproving == 1)
                {      
                                   
                    if( current_coordinate < numcordinates )        // Fetch next co-ordinate
                    {
                      destFound = 0;
                      PositionNotImproving = 0;
 
                      current_coordinate++;
                      
                      analogWrite(LPWM, 0); 
                      analogWrite(RPWM, 0);
//                      trackRobot();
//                      trackRobot();
//                      trackRobot();
//                      trackRobot();
//                      trackRobot();
                      Serial.print(x);
                    Serial.print(" ");
                    Serial.print(y);
                    Serial.print(" ");
                    Serial.println(phibot * 180/PI);
                      Serial.print(current_coordinate - 1);
                      Serial.println(" coordinate(s) visited ");
                      delay(2000);
                      current_mode = -1;  
                      

                    }
                  
                    else
                    {
                    analogWrite(LPWM, 0); // Stop
                    analogWrite(RPWM, 0);
                    trackRobot();
                      trackRobot();
                      trackRobot();
                      trackRobot();
                      trackRobot();
                    Serial.print(x);
                    Serial.print(" ");
                    Serial.print(y);
                    Serial.print(" ");
                    Serial.println(phibot * 180/PI);
                    Serial.println("All coordinates visited!");
                    while(1);
                    
                    }
                }  
//       // Serial.println("hi");
        obstacle_distances();                           // 3.18 ms
//       // Serial.println("bye");
//        
          phidest = atan2(K[current_coordinate]-y, H[current_coordinate]-x);
        if( (left_90_sf[0][0] < sensingThreshold) || (left_60_sf[0][0] < sensingThreshold) || ( left_30_sf[0][0] < sensingThreshold) || ( middle_sf[0][0] < (sensingThreshold - 10)) ||
        (right_90_sf[0][0] < sensingThreshold) || (right_60_sf[0][0] < sensingThreshold) || ( right_30_sf[0][0] < sensingThreshold) || (current_mode == 2)  )
        {
                    
			MMul(left_90_rf,tm1_1,left_90_sf,3,3,3,1);
			MMul(left_60_rf,tm1_2,left_60_sf,3,3,3,1);               
			MMul(left_30_rf,tm1_3,left_30_sf,3,3,3,1);
                        MMul(middle_rf,tm1_4,middle_sf,3,3,3,1);
			MMul(right_30_rf,tm1_5,right_30_sf,3,3,3,1); 
			MMul(right_60_rf,tm1_6,right_60_sf,3,3,3,1);
                        MMul(right_90_rf,tm1_7,right_90_sf,3,3,3,1);

                        MAdd(obstacle_rf, left_90_rf, left_60_rf, left_30_rf, middle_rf, right_30_rf, right_60_rf, right_90_rf);
			

               avoidAngle = atan2(obstacle_rf[1][0], obstacle_rf[0][0]);

              ComputeFWD();
//              
               if( abs(followWallDecider) > exitFWthreshold && (current_mode == 2 || current_mode == 1)  )    // check
               {
                    if(current_mode != 2)
                    {
                      current_mode = 2;
                      resetParameters();
                      
                              if(avoidAngle > 0)
                                FWdirection = 1;
                              
                              else
                                FWdirection = 0;
                              
                    }
                     followWall();
              } 
//               
               else
                 {
                     if(current_mode != 1)
                          {
                            current_mode = 1;
                            resetParameters();
                          }
                          
                     AO_GTG();
                 }
        }
//
                 else
                 {
                   if(current_mode != 0)
                      {
                        current_mode = 0;
                        resetParameters();
                      }     
                   goToGoal();
                 }      
//        
          //Serial.print("One iteration    ");
          Serial.println(micros() - starttime);
}







void goToGoal()
{
    
              shiftErrors();
              
              orientation_error[0] = (phidest - phibot) ;
              limitOrientationError();
              
              P = Kpao*orientation_error[0];  
              I = Kiao*(orientation_error[1] + orientation_error[2] + orientation_error[3] + orientation_error[4] );
              D = Kdao *(orientation_error[0] - orientation_error[1]);

            vr =  (2.0*v + (P+I+D)*L)/(2*R); 
            vl =  (2.0*v - (P+I+D)*L)/(2*R);  
            
            vr =  (2.0*v + (P+I+D)*L)/(2*R); 
            vl =  (2.0*v - (P+I+D)*L)/(2*R);
            
            printRobotParameters();
            driveMotors();
            trackRobot();			
}



void AO_GTG()
{
              
              
              if( middle_sf[0][0] < (sensingThreshold - 10))
              {
                  if (prev_middle == 0)
                  {
                    if (avoidAngle >= 0)
                    {
                      avoidAngle = PI/2;
                      previously = 1;
                    }
                    else
                    {
                      previously = 0;
                      avoidAngle = -PI/2;
                    }
                  }
                  
                  else 
                  {
                    if (previously == 1)
                    avoidAngle = PI/2;
                    else
                    avoidAngle = -PI/2;
                  }
                 
                  
                  prev_middle = 1;

              }
              else
              prev_middle = 0;
              //if( avoidAngle != 0)
              shiftErrors();
              orientation_error[0] = 1*avoidAngle + 0*(phidest - phibot); 
//              else
//              orientation_error[0] = (phidest - phibot) ;
              limitOrientationError();
              
              P = Kpao*orientation_error[0];  
              I = Kiao*(orientation_error[1] + orientation_error[2] + orientation_error[3] + orientation_error[4] );
              D = Kdao *(orientation_error[0] - orientation_error[1]);

            vr =  (2.0*v + (P+I+D)*L)/(2*R); 
            vl =  (2.0*v - (P+I+D)*L)/(2*R);  
            
            vr =  (2.0*v + (P+I+D)*L)/(2*R); 
            vl =  (2.0*v - (P+I+D)*L)/(2*R);
            
            printRobotParameters();
            driveMotors();
            trackRobot();			
}
 	
void followWall()
 {
   float norm, dotsum, FWangle, wallError;
   dotsum = 0.0;
   
    if(FWdirection == 1)
    {
      Kpfw = (15.0 * velocity / 100.0);
    ObtainRight();
    }
    else
    {
    Kpfw = (20.0 * velocity / 100.0);
    ObtainLeft();
    }
    
    // ua will be used to find the perpendicular distance to the wall
    ua[0][0] = p1[0][0];
    ua[1][0] = p1[1][0];
   
   // Find the tangent to the wall and the unit vector along the tangent direction
    u_fw_t[0][0] = p2[0][0] - p1[0][0]; 
    u_fw_t[1][0] = p2[1][0] - p1[1][0];
    norm = sqrt( pow(u_fw_t[0][0],2.0) + pow(u_fw_t[1][0],2.0) );
    u_fw_tp[0][0] = u_fw_t[0][0]/norm;
    u_fw_tp[1][0] = u_fw_t[1][0]/norm;  
    
    // Find the perpendicular to the wall and the unit vector along the perpendicular direction using 1) ua and 2) tangential component of ua
    dotsum = ua[0][0]*u_fw_tp[0][0] + ua[1][0]*u_fw_tp[1][0];            // component of ua along tangent
    u_fw_p[0][0] = (1.0*ua[0][0] - up[0][0]) - (dotsum)*u_fw_tp[0][0];   // as we know 1)ua and 2)tangential component of ua, we can compute the perpendicular component
    u_fw_p[1][0] = (ua[1][0] - up[1][0]) - (dotsum)*u_fw_tp[1][0];
    norm = sqrt( pow(u_fw_p[0][0],2.0) + pow(u_fw_p[1][0],2.0));
    u_fw_pu[0][0] = u_fw_p[0][0]/norm;
    u_fw_pu[1][0] = u_fw_p[1][0]/norm;
  

     
    // Compute the difference between         
    u_fw_pp[0][0] = u_fw_p[0][0] - wallDistance*u_fw_pu[0][0];        
    u_fw_pp[1][0] = u_fw_p[1][0] - wallDistance*u_fw_pu[1][0];
    
    u_fw[0][0] =  u_fw_pp[0][0]*pWeight + u_fw_t[0][0]*tWeight;
    u_fw[1][0] =  u_fw_pp[1][0]*pWeight + u_fw_t[1][0]*tWeight;
    
              
              
           
              FWangle = atan2(u_fw[1][0], u_fw[0][0]);
              shiftErrors();
              orientation_error[0] = FWangle;
              limitOrientationError(); 
 
                if ( (FWdirection == 1) && (right_90_sf[0][0] == maxDist ) && (right_60_sf[0][0] == maxDist ) && (right_30_sf[0][0] == maxDist )  && (middle_sf[0][0] == maxDist ) )
                 orientation_error[0] = - lostwallDriveRight / (Kpfw * 3.0);
                if ( (FWdirection == 0) && (left_90_sf[0][0] == maxDist ) && (left_60_sf[0][0] == maxDist ) && (left_30_sf[0][0] == maxDist ) && (middle_sf[0][0] == maxDist ) ) 
                 orientation_error[0] = + lostwallDriveLeft / (Kpfw * 3.0);

              P = Kpfw*orientation_error[0];  
              I = Kifw*(orientation_error[1] + orientation_error[2] + orientation_error[3] + orientation_error[4]);
              D = Kdfw *(orientation_error[0] - orientation_error[1]);

            vr =  (2.0*v + (P+I+D)*L)/(2*R); 
            vl =  (2.0*v - (P+I+D)*L)/(2*R);  
              
               printRobotParameters();
              driveMotors();
                     
              trackRobot();   
}
  
  
  
void ObtainRight()
{
  
    int flag = 0;
  
//     onlyone_right();                    
     if ((right_90_sf[0][0] != maxDist ) && (right_60_sf[0][0] == maxDist ) && (right_30_sf[0][0] == maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = right_60_rf[0][0]; 
         p2[1][0] = right_60_rf[1][0];
         
         p1[0][0] = right_90_rf[0][0]; 
         p1[1][0] = right_90_rf[1][0];
         flag = 1;
    }
    
    if ((right_90_sf[0][0] == maxDist ) && (right_60_sf[0][0] != maxDist ) && (right_30_sf[0][0] == maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = right_60_rf[0][0]; 
         p2[1][0] = right_60_rf[1][0];
         
         p1[0][0] = right_90_rf[0][0]; 
         p1[1][0] = right_90_rf[1][0];
         flag = 1;
    }
    
    if ((right_90_sf[0][0] == maxDist ) && (right_60_sf[0][0] == maxDist ) && (right_30_sf[0][0] != maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = right_30_rf[0][0]; 
         p2[1][0] = right_30_rf[1][0];
         
         p1[0][0] = right_60_rf[0][0]; 
         p1[1][0] = right_60_rf[1][0];
         
         flag = 1;
    }
    //The condition for only middle sensor sensing is not included here. It doesnt look like in the FW behaviour, this condition will ever arise
    
    
    if (flag == 0)    // may have to add more stuff
    {
      twoormore_right();
      
    }
    flag = 0;
               
}



void ObtainLeft()
{
  
    int flag = 0;
  
//     onlyone_left();                    
     if ((left_90_sf[0][0] != maxDist ) && (left_60_sf[0][0] == maxDist ) && (left_30_sf[0][0] == maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = left_60_rf[0][0]; 
         p2[1][0] = left_60_rf[1][0];
         
         p1[0][0] = left_90_rf[0][0]; 
         p1[1][0] = left_90_rf[1][0];
         flag = 1;
    }
    
    if ((left_90_sf[0][0] == maxDist ) && (left_60_sf[0][0] != maxDist ) && (left_30_sf[0][0] == maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = left_60_rf[0][0]; 
         p2[1][0] = left_60_rf[1][0];
         
         p1[0][0] = left_90_rf[0][0]; 
         p1[1][0] = left_90_rf[1][0];
         flag = 1;
    }
    
    if ((left_90_sf[0][0] == maxDist ) && (left_60_sf[0][0] == maxDist ) && (left_30_sf[0][0] != maxDist ) && (middle_sf[0][0] == maxDist))
    {
         p2[0][0] = left_30_rf[0][0]; 
         p2[1][0] = left_30_rf[1][0];
         
         p1[0][0] = left_60_rf[0][0]; 
         p1[1][0] = left_60_rf[1][0];
         
         flag = 1;
    }
    //The condition for only middle sensor sensing is not included here. It doesnt look like in the FW behaviour, this condition will ever arise
    
    
    if (flag == 0)
    {
      twoormore_left(); 
    }
    flag = 0;


}



void resetParameters()
{
      orientation_error[0] = 0;
      orientation_error[1] = 0;
      orientation_error[2] = 0;
      orientation_error[3] = 0;
      orientation_error[4] = 0;
      orientation_error[5] = 0;
}

void shiftErrors()
{
      orientation_error[5] = orientation_error[4];
      orientation_error[4] = orientation_error[3]; 
      orientation_error[3] = orientation_error[2];
      orientation_error[2] = orientation_error[1]; 
      orientation_error[1] = orientation_error[0];
}

void limitOrientationError()
{
  if (orientation_error[0] > PI)
  orientation_error[0] = orientation_error[0] - 2*PI;
  
  else if (orientation_error[0] <= -PI)
  orientation_error[0] = 2*PI + orientation_error[0];
}



void driveMotors()
  {
             if (vr > vMax)
                 vr = vMax;
             if (vr < vMin)
                 vr = vMin;
 
  
              if (vl > vMax)
                 vl = vMax;
              if (vl < vMin)
                 vl = vMin; 
            
            analogWrite(LPWM, vl);
            analogWrite(RPWM, vr);
  }

void trackRobot()
{
       count[0] = 0;
       count[1] = 0;
       timetaken = micros();
       ticks(count);
       Serial.print((count[0]));
       Serial.print(" ");
       Serial.println((count[1]));
      // Serial.print(count[0]);
      //  Serial.print(" ");
      //  Serial.println(count[1]);
      // Serial.println(" ");
      
        Nl = 1.0*count[0]/ticksperev;
        Nr = 1.0*count[1]/ticksperev;
        
        Dl = 2.0*PI*R*Nl;
        Dr = 2.0*PI*R*Nr;
        Dc = ( Dl + Dr )/2.0;  
         
      
        oldphibot = phibot;
        phibot = phibot + (Dr-Dl)/L;
      
        if(phibot > PI)
        phibot = phibot - 2*PI;
        
        else if(phibot <= -PI)
        phibot = phibot + 2*PI;
       
        x = x + Dc*cos((oldphibot));
        y = y + Dc*sin((oldphibot));
}



 void ticks(float count[])
 { 
                int newinputL, newinputR, previnputR, previnputL;
                unsigned int i;
                i = 0; 
                flagL = 0; 
                flagR = 0;
                newinputL = digitalRead(leftsensor) ;
                newinputR = digitalRead(rightsensor) ;
              
                while (i < ticksduration)
                { 
              
                  i++;
                  previnputL = newinputL;
                  newinputL = digitalRead(leftsensor);
                  previnputR = newinputR;
                  newinputR = digitalRead(rightsensor);

                  if (newinputL == previnputL && flagL == 0)
                  flagL = 1;
                  if (newinputR == previnputR && flagR == 0)
                  flagR = 1;
                  
                  if ((newinputL != previnputL))
                  {
                      if (flagL == 1)
                      flagL = 2;
                      else
                      flagL = 0; 
                  } 
                  
                  if ((newinputR != previnputR))
                  {
                      if (flagR == 1)
                      flagR = 2;
                      else
                      flagR = 0; 
                  }
                  
                  
                  if ((newinputL == previnputL) && (flagL == 2))
                  {
                      count[0] = count[0] + 1 ;
                      flagL = 0;
                  }
                  
                  if ((newinputR == previnputR) && (flagR == 2))
                  {
                      count[1] = count[1] + 1 ;
                      flagR = 0;
                  }
                  
                }
 }



void obstacle_distances()                                                // 3.18 ms
{
  unsigned long int duration[8], beginTime;
  unsigned long int sonarFlag[8] = {0,0,0,0,0,0,0,0};
  
  digitalWrite(triggerPin,HIGH);                                             // trigger the ultrasonic sensors to Tx ultrasonic signals
  delayMicroseconds(15);
  digitalWrite(triggerPin,LOW);
  
      beginTime = micros();
      left_90_sf[0][0] = maxDist ;
      left_60_sf[0][0] = maxDist ;
      left_30_sf[0][0] = maxDist ;
      middle_sf[0][0] = maxDist ;
      right_30_sf[0][0] = maxDist ;
      right_60_sf[0][0] = maxDist ;
      right_90_sf[0][0] = maxDist ;
   
  while( (micros() - beginTime) < echoTimeout )                             // possibility of bug; variable may be needed type returned by micros may not match;  
  {   
    
      if( (digitalRead(right_90_sensor) == 1) && (sonarFlag[7] == 0) )                                                                                  
      {
          sonarFlag[7] = 1;                                                  
          duration[7] = micros();                                           
      }
    
      if( (digitalRead(left_90_sensor) == 1) && (sonarFlag[1] == 0) )       // wait for echoPin (here, right_45_sensor) to go HIGH to start timer; 
                                                                             // second test condition is used to ensure timer is not restarted every iteration
      {
          sonarFlag[1] = 1;                                                  // sonarFlag = 1 to indicates timer has started
          duration[1] = micros();                                           
      }
      
      if( (digitalRead(left_60_sensor) == 1) && (sonarFlag[2] == 0) )                                                                                
      {
          sonarFlag[2] = 1;                                                  
          duration[2] = micros();                                           
      }
      
      if( (digitalRead(left_30_sensor) == 1) && (sonarFlag[3] == 0) )                                                                                      
      {
          sonarFlag[3] = 1;                                                  
          duration[3] = micros();                                           
      }
      
      if( (digitalRead(middle_sensor) == 1) && (sonarFlag[4] == 0) )                                                                                 
      {
          sonarFlag[4] = 1;                                                  
          duration[4] = micros();                                           
      }
      
      if( (digitalRead(right_30_sensor) == 1) && (sonarFlag[5] == 0) )                                                                                 
      {
          sonarFlag[5] = 1;                                                  
          duration[5] = micros();                                           
      }
      
      if( (digitalRead(right_60_sensor) == 1) && (sonarFlag[6] == 0) )                                                                                  
      {
          sonarFlag[6] = 1;                                                  
          duration[6] = micros();                                           
      }
      
     


   if( (digitalRead(right_90_sensor) == 0) && (sonarFlag[7] == 1) )      
      {                                                 
          duration[7] = micros() - duration[7];   
          right_90_sf[0][0] = duration[7] / distanceScaleFactor; 
          sonarFlag[7] = 2;         
      }
                                                                     // it takes a finite time for ultrasonic sensor to Tx ultrasonic signal and raise the echoPin (here, right_90_sensor)
      if( (digitalRead(left_90_sensor) == 0) && (sonarFlag[1] == 1) )      // 2nd condition ensures a LOW on the echoPin is not interpreted as echo reception before ultrasonic signal is sent.
      {
          duration[1] = micros() - duration[1];                              // duration is the time interval between Tx and Rx of the ultrasonic signal 
          left_90_sf[0][0] = duration[1] / distanceScaleFactor;                       // convert duration to distance in cm
          sonarFlag[1] = 2;
      }    
    
    
      if( (digitalRead(left_60_sensor) == 0) && (sonarFlag[2] == 1) )      
      {                                                
          duration[2] = micros() - duration[2];
          left_60_sf[0][0] = duration[2] / distanceScaleFactor;   
          sonarFlag[2] = 2;       
      }
      
      if( (digitalRead(left_30_sensor) == 0) && (sonarFlag[3] == 1) )      
      {                                               
          duration[3] = micros() - duration[3]; 
          left_30_sf[0][0] = duration[3] / distanceScaleFactor;
          sonarFlag[3] = 2;
      }
      
      if( (digitalRead(middle_sensor) == 0) && (sonarFlag[4] == 1) )      
      {                                                 
          duration[4] = micros() - duration[4];  
          middle_sf[0][0] = duration[4] / distanceScaleFactor; 
          sonarFlag[4] = 2;         
      }
      
      if( (digitalRead(right_30_sensor) == 0) && (sonarFlag[5] == 1) )      
      {                                                 
          duration[5] = micros() - duration[5];  
          right_30_sf[0][0] = duration[5] / distanceScaleFactor; 
          sonarFlag[5] = 2;         
      }
      
      if( (digitalRead(right_60_sensor) == 0) && (sonarFlag[6] == 1) )      
      {                                                 
          duration[6] = micros() - duration[6];   
          right_60_sf[0][0] = duration[6] / distanceScaleFactor; 
          sonarFlag[6] = 2;         
      }
      
   
      
  }
          //  Serial.println(freeRam());
            Serial.print((left_90_sf[0][0])); 
            Serial.print(" ");
            Serial.print((left_60_sf[0][0]));
            Serial.print(" ");
            Serial.print((left_30_sf[0][0]));
            Serial.print(" ");
            Serial.print((middle_sf[0][0]));
            Serial.print(" ");
            Serial.print((right_30_sf[0][0]));
            Serial.print(" ");
            Serial.print((right_60_sf[0][0]));
            Serial.print(" ");
            Serial.println((right_90_sf[0][0]));  
            Serial.println(" "); 
}




void printRobotParameters()          // 1.35 ms
{
          //  Serial.print(" ");
            
            Serial.print(int(x));
            Serial.print(" ");
            Serial.println(int(y));
            Serial.print(" ");
            Serial.print(int(phibot*180/PI));
            Serial.print(" ");
            Serial.print(int(phidest * 180/PI));
            Serial.print(" ");
            Serial.print(int(avoidAngle * 180/PI));
            Serial.print(" ");
            
            
            Serial.println(int(orientation_error[0] * 180/PI) );
           
           
           if(current_mode == 0)
            Serial.println("GTG");
            
            
             if(current_mode == 1)
            Serial.println("AO_GTG");
            
            if(current_mode == 2)
            {
                if(FWdirection == 1)
                Serial.println("FWR ");

                
                else
                Serial.println("FWL");
                
            }
            
            Serial.print(int(vl));
            Serial.print(" ");
            Serial.println(int(vr));
            Serial.println("");
            Serial.println("");
}






 void MAdd(float sum[3][3], float a[3][3],float b[3][3],float c[3][3], float d[3][3], float e[3][3], float f[3][3], float g[3][3])
{
		int i;
		for (i=0;i<3;i++)
		sum[i][0] = a[i][0]*w1 + b[i][0]*w2 + c[i][0]*w3 + d[i][0]*w4 + e[i][0]*w5 + f[i][0]*w6 + g[i][0]*w7;
}
			
			

	
void Transformation_Matrix(float RM[3][3],float x,float y,float theta)
{			
		RM[0][0] = cos(theta);
		RM[0][1] = -1.0*sin(theta);
		RM[0][2] = x;

		RM[1][0] = sin(theta);
		RM[1][1] = cos(theta);
		RM[1][2] = y;

		RM[2][0] = 0;
		RM[2][1] = 0;
		RM[2][2] = 1;
}
			
			
	
void MMul( float multiply[3][3], float first[3][3], float second[3][3], int fr, int fc,int sr,int sc)
{
		int m, n, p, q, c, d, k;
                float sum;
                sum = 0.0;
				  
  		 for ( c = 0 ; c < fr; c++ )
  		{
  			for ( d = 0 ; d < sc ; d++ )
  			{
  				for ( k = 0 ; k < sr ; k++ )
  				{
  				          sum = sum + first[c][k]*second[k][d];
  				}
  				 
  				multiply[c][d] = sum;
  				sum = 0.0;
                        }
                }  				        				   
}
 
 
 int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



 void twoormore_right()
{
  float arr[4];
arr[0] = right_90_sf[0][0];
arr[1] =  right_60_sf[0][0];
arr[2] = right_30_sf[0][0];
arr[3] = middle_sf[0][0];
  
  int i, posfirst, possecond;
 
   float first, second;

 
    first = second = maxDist;
    possecond = posfirst = -1;
    
    for (i = 0; i < 4 ; i ++)
    {
        /* If current element is smaller than first then update both
          first and second */
        if ( (arr[i] < first ) ) //Not checking for the equality case; might lead to an error, if bot doesnt turn enough in the right direction
        {
			
				second = first;
				possecond = posfirst;
			
				first = arr[i];
				posfirst = i;
				
	}

        
 
        /* If arr[i] is in between first and second then update second  */
        else if (arr[i] < second )
				{
					second = arr[i];
					possecond = i;
				}
    }

    
    // Any of them could be greater than the other
    //if possecond is p2
    if ( possecond > posfirst)
    {
      if (possecond == 1)
      {
         p2[0][0] = right_60_rf[0][0]; 
         p2[1][0] = right_60_rf[1][0];
      }
      
      if (possecond == 2)
      {
         p2[0][0] = right_30_rf[0][0]; 
         p2[1][0] = right_30_rf[1][0];
      }
      
      if (possecond == 3)
      {
         p2[0][0] = middle_rf[0][0]; 
         p2[1][0] = middle_rf[1][0];
      }      
      
      // posfirst is p1
      
      if (posfirst == 0)
      {
         p1[0][0] = right_90_rf[0][0]; 
         p1[1][0] = right_90_rf[1][0];
      }      
      
     if (posfirst == 1)
      {
         p1[0][0] = right_60_rf[0][0]; 
         p1[1][0] = right_60_rf[1][0];
      }
      
      if (posfirst == 2)
      {
         p1[0][0] = right_30_rf[0][0]; 
         p1[1][0] = right_30_rf[1][0];
      }
      
   }
  
    //Now the other way around, posfirst is p2
    
    if( posfirst > possecond)
    {
      if (posfirst == 1)
      {
         p2[0][0] = right_60_rf[0][0]; 
         p2[1][0] = right_60_rf[1][0];
      }
      
      if (posfirst == 2)
      {
         p2[0][0] = right_30_rf[0][0]; 
         p2[1][0] = right_30_rf[1][0];
      }
      
      if (posfirst == 3)
      {
         p2[0][0] = middle_rf[0][0]; 
         p2[1][0] = middle_rf[1][0];
      }
      
      
      //Now possecond as p1
      
       if (possecond == 0)
      {
         p1[0][0] = right_90_rf[0][0]; 
         p1[1][0] = right_90_rf[1][0];
      }  
      
      if (possecond == 1)
      {
         p1[0][0] = right_60_rf[0][0]; 
         p1[1][0] = right_60_rf[1][0];
      }
      
      if (possecond == 2)
      {
         p1[0][0] = right_30_rf[0][0]; 
         p1[1][0] = right_30_rf[1][0];
      }
      
      
    }
    Serial.print(posfirst);
              Serial.print(" ");
              Serial.println(possecond);
}
 
 
 
void twoormore_left()
{
  float arr[4] = {left_90_sf[0][0], left_60_sf[0][0], left_30_sf[0][0], middle_sf[0][0]};
  
  int i,  posfirst, possecond;
  float first, second;
  
 
   
 
    first = second = maxDist;
    possecond = posfirst = -1;
    for (i = 0; i < 4 ; i ++)
    {
        /* If current element is smaller than first then update both
          first and second */
        if ( (arr[i] < first ) ) //Not checking for the equality case; might lead to an error, if bot doesnt turn enough in the left direction
        {
			
				second = first;
				possecond = posfirst;
			
				first = arr[i];
				posfirst = i;
				
	}

        
 
        /* If arr[i] is in between first and second then update second  */
        else if (arr[i] < second )
				{
					second = arr[i];
					possecond = i;
				}
    }

    
    // Any of them could be greater than the other
    //if possecond is p2
    if ( possecond > posfirst)
    {
      if (possecond == 1)
      {
         p2[0][0] = left_60_rf[0][0]; 
         p2[1][0] = left_60_rf[1][0];
      }
      
      if (possecond == 2)
      {
         p2[0][0] = left_30_rf[0][0]; 
         p2[1][0] = left_30_rf[1][0];
      }
      
      if (possecond == 3)
      {
         p2[0][0] = middle_rf[0][0]; 
         p2[1][0] = middle_rf[1][0];
      }      
      
      // posfirst is p1
      
      if (posfirst == 0)
      {
         p1[0][0] = left_90_rf[0][0]; 
         p1[1][0] = left_90_rf[1][0];
      }      
      
      if (posfirst == 1)
      {
         p1[0][0] = left_60_rf[0][0]; 
         p1[1][0] = left_60_rf[1][0];
      }
      
      if (posfirst == 2)
      {
         p1[0][0] = left_30_rf[0][0]; 
         p1[1][0] = left_30_rf[1][0];
      }
      
   }
  
    //Now the other way around, posfirst is p2
    
    if( posfirst > possecond)
    {
      if (posfirst == 1)
      {
         p2[0][0] = left_60_rf[0][0]; 
         p2[1][0] = left_60_rf[1][0];
      }
      
      if (posfirst == 2)
      {
         p2[0][0] = left_30_rf[0][0]; 
         p2[1][0] = left_30_rf[1][0];
      }
      
      if (posfirst == 3)
      {
         p2[0][0] = middle_rf[0][0]; 
         p2[1][0] = middle_rf[1][0];
      }
      
      
      //Now possecond as p1
      
       if (possecond == 0)
      {
         p1[0][0] = left_90_rf[0][0]; 
         p1[1][0] = left_90_rf[1][0];
      }  
      
      if (possecond == 1)
      {
         p1[0][0] = left_60_rf[0][0]; 
         p1[1][0] = left_60_rf[1][0];
      }
      
      if (possecond == 2)
      {
         p1[0][0] = left_30_rf[0][0]; 
         p1[1][0] = left_30_rf[1][0];
      }
      
      
    }
    Serial.print(posfirst);
              Serial.print(" ");
              Serial.println(possecond);
}

            
            
void ComputeFWD()
{
               if (avoidAngle > 0)
                 followWallDecider = avoidAngle - (phidest - phibot) + AOcorrection;
                 
               else if (avoidAngle < 0)
                 followWallDecider = avoidAngle - (phidest - phibot) - AOcorrection;  
              
                else 
                 followWallDecider = avoidAngle - (phidest - phibot);           
                 
               if(followWallDecider > PI)
               followWallDecider = followWallDecider - 2*PI;
               
               else if(followWallDecider <= -PI)
               followWallDecider = followWallDecider + 2*PI;
               
               Serial.print("FWD ");
               Serial.println(followWallDecider * 180/PI);
}
            
            
            
            
