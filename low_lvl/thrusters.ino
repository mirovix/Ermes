//Thruster low lvl considering the X axis (single axis example)
int equalize[15] = {0,83,94,103,111,121,132,144,157,172,187,206,223,244,250,255};
float input[30]={0, 0, 0.150, 0.150, 0.150, 0.150, -0.75, -0.75, -0.75, -0.35,-0.35,-0.35,-0.35,-0.35,0.25,0.25,0.25,0.25,0,0,0,0,0,0,0.105,0.105,0.105,0.105,0.105}; //tempo e valore
float s_max = 0.127*cos(45)*2;
int k = 10000;
float err_ss = 1/(1 + k*s_max);
int iter = 0;
float output_loop = 0.0;
float input_after_fb = 0;
int output[4]={0,0,0,0};;

void setup() {
  Serial.begin(9600);
}

void loop(){

  //end of the input data
  if(iter > 30)
    return;

  //TODO>>info from IMU
  float imu = 0;

  //fb controll with imu
  input_after_fb = (input[iter] - ((output_loop*s_max)+imu))*k;

  //clipping
  if(input_after_fb < -2)
    input_after_fb = -2;
  else if(input_after_fb > 2)
    input_after_fb = 2;
  
  output_loop = input_after_fb;
  //check if the prev and current value are the same for avoiding calculus
  //if((input[iter] != input[iter-1]) or (!iter)){
  //from 4 bit to 16 bit 
  int out_equalization = equalize[round(abs(input_after_fb)*15)];

  //check the sign
  if(input[iter] < 0){
      output[0] = 0;
      output[1] = 0;
      output[2] = out_equalization;
      output[3] = out_equalization;
  }
  else{
      output[0] = out_equalization;
      output[1] = out_equalization;
      output[2] = 0;
      output[3] = 0;
   }
  //}
  iter++;#

  //print the output
  for(int i=0;i<4;i++)
    Serial.println(output[i]);
  
}

float[] IMUFusion(float[] pos_imu1, float[] pos_imu2, float[] acc_imu1, float[] acc_imu2, float[] omega_imu1, float[] omega_imu2){
  //Moment of interta and mass
  int dof = 3;
  int m = 3;
  //float Ixx = 1.5e-3;
  //float Ixy = 3e-5;
  //float Ixz = 4e-5;
  //float Iyy = 6e-3;
  //float Iyz = 4e-5;
  float Izz = 4e-5;

  //computing the r vector
  float r[3];
  for(int i=0; i<dof;r[i] = pos_imu1[i] - pos_imu2[i], i++);
 
  //Step1 : simple mean
  float omega3 = (omega_imu1[dof-1]-omega_imu2[dof-1])/2;

  //Step2 : alfa obtain by a1 - a2 because it cancels a0
  float diff_acc_omega[] = {0,0,0};
  for(int i = 0; i<dof-1; diff_acc_omega[i] = - pow(omega3,2)*r[i],i++);

  //compute coeff of reference
  float coeff_acc[dof-1];
  for(int i=0; i<dof-1; coeff_acc[i] = acc_imu1[i] - acc_imu2[i] - diff_acc_omega[i], i++);

  //compute alfa
  float alfa[2];
  alfa[0] = -coeff_acc[0] / r[1];
  alfa[1] = coeff_acc[1] / r[0]; //alfa_b

  //step 3 and 4: computing the thrustes, output = [thrust[0], thrust[1], 0, torque]
  float output[] = {0,0,0,0};
  output[0] = m * acc_imu1[0] + alfa * pos_imu1[1] + pos_imu1[0] * pow(omega3, 2); //along x
  output[1] = m * acc_imu1[1] - alfa * pos_imu1[0] + pos_imu1[1] * pow(omega3, 2); //along y
  output[3] = Izz * alfa[0]; //only along z 
 
  return output;
}
