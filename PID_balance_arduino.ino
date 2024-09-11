#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;
#define motorRight 9
#define motorLeft 10
#define greenled 8
#define redled 7


/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

 int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float currSum;


float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0 ;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=1.5;//3.55
double ki=0.05;//0.003
double kd=3;//2.05
///////////////////////////////////////////////

double throttle=1100; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

// float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
// float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;

// float Kalman1DOutput[]={0,0};

// void kalman_1d(float KalmanState , float KalmanUncertainty, float KalmanInput, float KalmanMeasurment){
//   KalmanState = KalmanState + 0.004*KalmanInput;
//   KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
//   float KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+3*3);
//   KalmanState = KalmanState+KalmanGain*(KalmanMeasurment-KalmanState);
//   KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;

//   Kalman1DOutput[0]=KalmanState;
//   Kalman1DOutput[1]=KalmanUncertainty;

// }

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(greenled , OUTPUT);
  pinMode(redled, OUTPUT);
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  right_prop.attach(motorRight); //attatch the right motor to pin 3
  left_prop.attach(motorLeft);  //attatch the left motor to pin 5

  calibrateESC(right_prop);
  calibrateESC(left_prop);



  time = millis(); //Start counting time in milliseconds


void loop() {

    while (Serial.available() > 0) {
        desired_angle = Serial.read();
    }
    Serial.print("Desired Angle: ");
    Serial.print(desired_angle);
    Serial.print(" ");



/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
     Serial.print("Angle = ");
    Serial.print(Total_angle[1]);

    if(Total_angle[1]>(-2) && Total_angle[1]<(2)){
        digitalWrite(LED_BUILTIN, LOW);
    }else{
        digitalWrite(LED_BUILTIN, HIGH);
    }
    



   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
error = Total_angle[1] - desired_angle;

if(error > -3  &&  error < 3){
  digitalWrite(greenled , HIGH);
  digitalWrite(redled , LOW);
}else{
  digitalWrite(greenled , LOW);
  digitalWrite(redled , HIGH);
}
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_d = kd*((error - previous_error)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d;

Serial.print(" PID Value= ");
Serial.print(PID);

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeft = throttle + PID;
pwmRight = throttle - PID;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRight < 1000)
{
  pwmRight= 1010;
}
if(pwmRight > 1300)
{
  pwmRight=1300;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1010;
}
if(pwmLeft > 1300)
{
  pwmLeft=1300;
}


/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
Serial.print(" Left Prop:");
Serial.print(pwmLeft);
Serial.print(" Right Prop:");
Serial.println(pwmRight);
//left_prop.writeMicroseconds(1500);
//right_prop.writeMicroseconds(1500);
previous_error = error; //Remember to store the previous error.

}//end of loop void

void calibrateESC(Servo& esc) {
  // Full throttle calibration signal
  esc.writeMicroseconds(2000);
  delay(2000); // Wait for 2 seconds

  // No throttle calibration signal
  esc.writeMicroseconds(1000);
  delay(2000); // Wait for 2 seconds

  // Set to neutral signal (often necessary for some ESCs to recognize zero throttle after calibration)
  esc.writeMicroseconds(1500);
  delay(1000); // Wait for 1 second
}
