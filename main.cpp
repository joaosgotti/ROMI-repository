// Coded by Luís Afonso 11-04-2019
#include "mbed.h"
#include "BufferedSerial.h"
#include "Robot.h"
#include "Communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstdio>
//joao **


BufferedSerial pc(USBTX, USBRX);



#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define pi 3.141592654
#define PPR 1440


float *velocity1(float pose_objetivo[3],float pose[3],float v_ant, float w_ant);
float *velRobot2velWheels(float vRobot, float wRobot, float wheelRadius ,float wheelsDistance);
float *nextPose(float T, float w[2] , float wRobot, float poseRobot[3], float wheelsRadius, float wheelsDistance);
float *pursuit(float pose[3], float pose_final[3], float map[40][40], float Map_logs[40][40], float old_erro, float obj_pose[2]);
float VFF(float pose[3], float pose_final[3], float Map[40][40], float Map_logs[40][40]);
float *LRDistance(int countsLeft, int countsRight,float wheelRadius);

int main() {
   
    float odomX, odomY, odomTheta;
    pc.set_baud(115200);
    init_communication(&pc);


  //////////////////////////////////////////////////////////////////////////////// 
  //POSES DO ROBOT:
  float *pose_atual = (float*)calloc(3,sizeof(float));
  float *pose_o = (float*)calloc(3,sizeof(float));
  float *pose_final = (float*)calloc(3,sizeof(float));
  float *obj_pose = (float*)calloc(2,sizeof(float));
  pose_atual[0] =  10; 
  pose_atual[1] = 10;
  pose_atual[2] =  0;
  pose_o[0] = 40.5765;
  pose_o[1] = 39.4678;
  pose_o[2] = 0;
  pose_final[0] = 40;
  pose_final[1] = 0;
  pose_final[2] = 0;
 //Para o método pursuit ---> inicializar obj_pose igual à POSE.
  obj_pose[0] = pose_atual[0];
  obj_pose[1] = pose_atual[1];
  
  /////////////////////////////////////////////////////////////////////////////
  //PARAMETROS DO ROBOT:
  float wheelsRadius=3.5;
  float wheelsDistance=13;
  float T = 0.1;

  ////////////////////////////////////////////////////////////////////////////
  float vRobot = 0;
  float wRobot = 0;
  float flag = 0;
  float *velocidades = (float*)calloc(3,sizeof(float));
  float *parametros = (float*)calloc(5,sizeof(float));  // para o pursuit apenas
  float *w = (float*)calloc(2,sizeof(float));
  float *pose = (float*)calloc(3,sizeof(float));
  float old_erro = 0;

  /////////////////////////////////////////////////////////////////////////////////////
  //Inicializar o mapa e o mapa das log_odds a 0.
  float Map[40][40];
  float Map_logs[40][40]; 

  for(int i = 0; i < 40; i++){
      for(int j = 0; j < 40; j++){
              Map[i][j] = 0;  
      }
  }        
  for(int i = 0; i < 40; i++){
      for(int j = 0; j < 40; j++){
              Map_logs[i][j] = 0;  
      }
  }   
  /////////////////////////////////////////////////////////////////////////////////////////
  //DISTANCIAS:
    float *Dist = (float*)calloc(2,sizeof(float));

    
  //////////////////////////////////////////////////////////////////////

  /*for( int i = 1;i<=5; i++){

    //velocidades = velocity1(pose_o,pose_atual,vRobot,wRobot);
    //vRobot = velocidades[0];
    //wRobot = velocidades[1];
    //flag = velocidades[2];
    
    parametros = pursuit(pose_atual,pose_final,Map,Map_logs,old_erro, obj_pose);
    vRobot = parametros[0];
    wRobot = parametros[1];
    flag = parametros[2];
    obj_pose[0] = parametros[3];
    obj_pose[1] = parametros[4];
    old_erro = parametros[5];
    //printf(" ciclo: %d obj(1):%f  obj(2):%f   erro: %f\n",i,obj_pose[0],obj_pose[1],old_erro);
  
    w = velRobot2velWheels(vRobot,wRobot, wheelsRadius, wheelsDistance);
    pose = nextPose(T,w,wRobot, pose_atual, wheelsRadius, wheelsDistance);
    pose_atual[0] = pose[0];
    pose_atual[1] = pose[1];
    pose_atual[2] = pose[2];

    //setspeed to que?

    printf("Ciclo: %d: vRobot:%f  wRobot:%f\n",i,vRobot,wRobot);


 }*/
    setSpeeds(100,0);
   /*while(1) {
        // poll for measurements. Returns -1 if no new measurements are available. returns 0 if found one.

        getCountsAndReset();
        //setSpeeds(80,80);
        wait_us(10000); 
        //printf("Counts Left: %d  CountsRight: %d \n",countsLeft,countsRight);
        
        Dist = LRDistance(countsLeft, countsRight, wheelsRadius);
        //printf("Distancia Left: %.6f,  Distancia Right: %.6f \n", Dist[0], Dist[1]);
        
        parametros = pursuit(pose_atual,pose_final,Map,Map_logs,old_erro, obj_pose);
        vRobot = parametros[0];
        wRobot = parametros[1];
        flag = parametros[2];
        obj_pose[0] = parametros[3];
        obj_pose[1] = parametros[4];
        old_erro = parametros[5];
        //printf(" ciclo: %d obj(1):%f  obj(2):%f   erro: %f\n",i,obj_pose[0],obj_pose[1],old_erro);

        w = velRobot2velWheels(vRobot,wRobot, wheelsRadius, wheelsDistance);
        pose = nextPose(T,w,wRobot, pose_atual, wheelsRadius, wheelsDistance);
        pose_atual[0] = pose[0];
        pose_atual[1] = pose[1];
        pose_atual[2] = pose[2];
        setSpeeds(100,0);



    }

  //printf("wrapToPi teste %f\n",atan2(sin(-9.4),cos(-9.4)));
  free(pose_atual);
  free(pose_o);
  free(velocidades);
  free(w);
  free(pose);
  return 0;*/
}

float *LRDistance(int countsLeft, int countsRight,float wheelRadius){
   float *Distances = (float*)calloc(2,sizeof(float)); 
   
   float DTL = (countsLeft * 2*pi*wheelRadius)/ PPR;
   float DTR= (countsRight * 2*pi*wheelRadius)/ PPR;
  
   Distances[0] = DTL;
   Distances[1] = DTR;
   return Distances;
   
} 


float *velocity1(float pose_objetivo[3],float pose[3],float v_ant, float w_ant){
  float *output = (float*)calloc(3,sizeof(float));   // ouput = [v,w,flag].
  
  float errox = (abs(pose_objetivo[0]-  pose[0])/pose_objetivo[0])*100;
  float erroy = (abs(pose_objetivo[0]-  pose[0])/pose_objetivo[0])*100;
  float erro = sqrt(pow(errox,2) + pow(erroy,2));
  float threshold = 1.1;

  if (errox < threshold){
    output[0] = v_ant;
    output[1] = w_ant;
    output[2] = 1;
  }
  else{
      float kv = 0.3;
      float kw = 2;
      float v = kv * sqrt(pow(pose_objetivo[0] - pose[0],2) + pow(pose_objetivo[1] - pose[1],2));
      float phi = atan2(pose_objetivo[1] -pose[1],pose_objetivo[0]- pose[0]);
      float aux = phi -  pose[2];
      float w = kw * atan2(sin(aux),cos(aux));

      output[0] = v;
      output[1] = w;
      output[2] = 0;
  }
  return output;
}



float *velRobot2velWheels(float vRobot, float wRobot, float wheelRadius ,float wheelsDistance){
  float *w = (float*)calloc(2,sizeof(float)); 
  w[0] = (vRobot - (wheelsDistance/2)*wRobot) / wheelRadius;
  w[1] = (vRobot + (wheelsDistance/2)*wRobot) / wheelRadius;
  return w;
}


float *nextPose(float T, float w[2], float wRobot, float poseRobot[3], float wheelsRadius, float wheelsDistance){
  
  float *pose = (float*)calloc(3,sizeof(float));  
  pose[0] = poseRobot[0]+(T*wheelsRadius*((w[1]+w[0])/2))*cos(poseRobot[2]+(wRobot*T/2));
  pose[1] = poseRobot[1]+(T*wheelsRadius*((w[1]+w[0])/2))*sin(poseRobot[2]+(wRobot*T/2));
  pose[2] = poseRobot[2]+(T*wheelsRadius*(w[1]-w[0]))/wheelsDistance;

  return pose;
}


float *pursuit(float pose[3], float pose_final[3], float Map[40][40], float Map_logs[40][40], float old_erro, float obj_pose[2]){
  float *output = (float*)calloc(5,sizeof(float)); 

  float theta = VFF(pose,pose_final,Map,Map_logs);
  //float theta = 0.7;
  
  float deltat = 0.01;
  int dk =3;
  
  float *new_pose_obj = (float*)calloc(2,sizeof(float)); 

  //Calculo da nova pose objetivo à custa da posiçao objetivo anterior
  new_pose_obj[0] = obj_pose[0] + cos(theta)*1;
  new_pose_obj[1] = obj_pose[1] + sin(theta)*1;

  //erro integrao
  float novo_erro = sqrt(pow(new_pose_obj[0]- pose[0],2) + pow(new_pose_obj[1]- pose[1],2)) - dk;
  float erro = novo_erro* deltat + old_erro;

  float kv = 1.5;
  float ki = 0.5;
  float ks = 2;

  float vRobot = kv*novo_erro + ki*old_erro;
  float aux = atan2(new_pose_obj[1]- pose[1],new_pose_obj[0]- pose[0]) - pose[2];
  float wRobot = ks * atan2(sin(aux),cos(aux));


  float pose_errorx = (abs(pose_final[0]- pose[0])/ pose_final[0])*100;
  float pose_errory = (abs(pose_final[1]- pose[1])/ pose_final[1])*100;
  float pose_error = sqrt(pow(pose_errorx,2) +  pow(pose_errory,2));
  printf("pose error: %.6f: \n", pose_error);
  float threshold = 10;

  if(pose_error < threshold){
    output[0] = vRobot;
    output[1] = wRobot;
    output[2] = 1;
    output[3] = new_pose_obj[0];
    output[4] = new_pose_obj[1];
    output[5] = erro;
  } 
  else{
    output[0] = vRobot;
    output[1] = wRobot;
    output[2] = 0;
    output[3] = new_pose_obj[0];
    output[4] = new_pose_obj[1];
    output[5] = erro;
  }
  return output;
}


float VFF(float pose[3], float pose_final[3], float Map[40][40], float Map_logs[40][40]){
  
  int Fca = 2;
  int Fcr = 200;

  //Calculo da Força Atrativa entre a pose final do robot e a inicial
  float d = sqrt( pow(pose_final[0] - pose[0],2) +  pow(pose_final[1] - pose[1],2));
  float Fax = Fca * (pose_final[0] -  pose[0])/d;
  float Fay = Fca * (pose_final[1] -  pose[1])/d;

  //mapeamento da pose do Robot para o mapa 40x40;
  int xcell = floor(pose[0]/5) + 1;
  int ycell = floor(pose[1]/5) + 2;

  int jativa = 5;
  int x_inicial = 0;
  int x_final = 0;
  int y_inicial = 0;
  int y_final = 0;


  if(xcell - jativa <= 1){
    x_inicial = 1;
  }
  else{
    x_inicial = xcell-jativa;
  }

  if(xcell + jativa >= 40){
    x_final = 40;
  }
  else{
    x_final = xcell+jativa;
  }
  
  if(ycell - jativa <= 1){
    y_inicial = 1;
  }
  else{
    y_inicial = ycell-jativa;
  }

  if(ycell + jativa >= 40){
    y_final = 40;
  }
  else{
    y_final = ycell+jativa;
  }


  Map[1][1] = 69.3;    
  printf("Map[1][1]: %f\n", Map[1][1]);

  float theta = 0;
  return theta;
}

