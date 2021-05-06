// Coded by Luís Afonso 11-04-2019
#include "mbed.h"
#include "BufferedSerial.h"
#include "Robot.h"
#include "Communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstdio>
#include <iterator>
//joao


BufferedSerial pc(USBTX, USBRX);


#define pi 3.141592654
#define PPR 1440


float *velocity1(float *pose,float *pose_objetivo,float v_ant, float w_ant);
float *velRobot2velWheels(float vRobot, float wRobot, float wheelRadius ,float wheelsDistance);
float *pursuit(float *pose, float *pose_obj,float Map[40][40], float Map_logs[40][40], float old_erro, float *pose_final);
float VFF(float pose[3], float pose_final[3], float Map[40][40], float Map_logs[40][40]);
float *LRDistance(int countsLeft, int countsRight,float wheelRadius,float wheelsDistance);
float *nextPoseV2(float *pose,float *Dist);
float *componentes(float *Dist, float itter_time, float wheelRadius,float wheelsDistance);
float *map(float *x, float in_min, float in_max, float out_min, float out_max);

int i = 0;

int main() {
   
    float odomX, odomY, odomTheta;
    pc.set_baud(115200);
    init_communication(&pc);


  //////////////////////////////////////////////////////////////////////////////// 
  //POSES DO ROBOT:


  /////////////////////////////////////////////////////////////////////////////
  //PARAMETROS DO ROBOT:
  float wheelsRadius=3.5;
  float wheelsDistance=13;
  float T = 0.1;

  ////////////////////////////////////////////////////////////////////////////
  float vRobot = 0;
  float wRobot = 0;

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
     float *pose_atual = (float*)calloc(3,sizeof(float));
  float *pose_final = (float*)calloc(3,sizeof(float));
  pose_atual[0] = 0; 
  pose_atual[1] = 0;
  pose_atual[2] = 0;
  pose_final[0] = 200;
  pose_final[1] = 0;
  pose_final[2] = 0;
    
    float *Dist = (float*)calloc(2,sizeof(float));
    float *velocidades = (float*)calloc(4,sizeof(float));
    float *W_wheels = (float*)calloc(2,sizeof(float));
    float TDistance;
    float itter_time = 0.1; //ittertime é igual a 0.1 segundos
    float theta;
    float velRobot;
    float *next_pose = (float*)calloc(3,sizeof(float)); 
    float *ww = (float*)calloc(2,sizeof(float));
    float *pose_obj= (float*)calloc(2,sizeof(float));
    float *parametros = (float*)calloc(6,sizeof(float));  // para o pursuit apenas
    
    
    pose_obj[0] = pose_atual[0];
    pose_obj[1] = pose_atual[1];

    float D;
    float deltatheta;
    float aux;
    float flag = 0;

     float old_erro = 0;
     int i = 1;
    while(flag==0) {
        getCountsAndReset();
        Dist = LRDistance(countsLeft, countsRight, wheelsRadius,wheelsDistance);
        //printf("%d %d\n", countsLeft, countsRight);
       //printf("countsLeft %.2f, countsRight; %.2f", countsLeft, countsRight);
     //conversao da pose do robot radeanos para graus

        //velocidades = velocity1(pose_atual, pose_final, vRobot, wRobot);
        //vRobot = velocidades[0];
        //wRobot = velocidades[1];
        //flag = velocidades[2]
        
        
        parametros = pursuit(pose_atual, pose_obj, Map, Map_logs, old_erro, pose_final);
        vRobot = parametros[0];
        wRobot = parametros[1];
        flag = parametros[2];
        pose_obj[0] = parametros[3];
        pose_obj[1] = parametros[4];
        old_erro = parametros[5];
    

        ww = velRobot2velWheels(vRobot, wRobot,wheelsRadius , wheelsDistance);
        //printf(" [%.2f, %.2f] \n",ww[0], ww[1]);
        W_wheels = map(ww,-15, 15, -100, 100);
        printf("C:[%d] D:[%.1f]  CountsLeft:[%d] CountsRight:[%d]  Pose_atual: [%.2f, %.2f]   Pose_obj:[%.2f, %.2f] WL,WR:[%.2f , %.2f] Erro:[%.2f]\n",i, Dist[0],countsLeft, countsRight,pose_atual[0], pose_atual[1],pose_obj[0],pose_obj[1], W_wheels[0], W_wheels[1], old_erro);
        setSpeeds(W_wheels[0], W_wheels[1]);
        wait_us(100000); //0.1 s
        next_pose =  nextPoseV2(pose_atual, Dist);
        pose_atual[0] = next_pose[0];
        pose_atual[1] = next_pose[1];
        pose_atual[2] = next_pose[2]; 
    
        i = i+1;
    }
    setSpeeds(0,0);

  //printf("wrapToPi teste %f\n",atan2(sin(-9.4),cos(-9.4)));
  free(pose_atual);
    free(parametros);
  free(velocidades);
  free(ww);

  return 0;
}

float *LRDistance(int countsLeft, int countsRight,float wheelRadius,float wheelsDistance){
   float *Distances = (float*)calloc(2,sizeof(float)); 
   
   float DTL = ((countsLeft * 2*pi*wheelRadius)/ PPR); //Deslocamento da roda esquerda a cada wait
   float DTR = ((countsRight * 2*pi*wheelRadius)/ PPR); //Deslocamento da roda direita a cada wait 
   float D = (DTL + DTR) /2;
   float delta_theta = (DTR - DTL)/wheelsDistance;
   
   
   Distances[0] = D;
   Distances[1] = delta_theta;
   return Distances;
} 


float *nextPoseV2(float *pose ,float *Dist){// ???
    float *next_pose = (float*)calloc(3,sizeof(float)); 
    float delta_D = Dist[0];
    float delta_theta  = Dist[1];
  
    next_pose[0] = pose[0] + delta_D*cos(pose[2]) + delta_theta/2;
    next_pose[1] = pose[1] + delta_D*sin(pose[2]) + delta_theta/2;
    next_pose[2] = pose[2] + delta_theta;
    next_pose[2] = atan2(sin(next_pose[2]),cos(next_pose[2]));
    
    return next_pose;
}

float *map(float *w, float in_min, float in_max, float out_min, float out_max) {
    float *w_saida = (float*)calloc(2,sizeof(float));
    

    w_saida[0]=  (w[0] - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    w_saida[1]=  (w[1] - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


    if(w_saida[0] <35 && w_saida[0] > 0){
        w_saida[0] = 35;
    }
    if(w_saida[1] <35 && w_saida[1] > 0){
        w_saida[1] = 35;
    }

     if(w_saida[0] > -35 && w_saida[0] < 0) {
        w_saida[0] = -35;
    }
     if(w_saida[1] > -35 && w_saida[1] < 0){
        w_saida[1] = -35;
    }

    if(w[0] > 120){
        w[0] = 120;
    }

    if(w[1] > 120){
        w[1] = 120;
    }
    

    return w_saida;
}


float *velocity1(float *pose,float *pose_objetivo,float v_ant, float w_ant){
  
  float *output = (float*)calloc(3,sizeof(float));   

  float errox = (abs(pose_objetivo[0]- pose[0]));
  float erroy = (abs(pose_objetivo[1]- pose[1]));
  float erro = sqrt(pow(errox,2) + pow(erroy,2));
  float threshold = 5;


  if (erro < threshold){
    output[0] = v_ant;
    output[1] = w_ant;
    output[2] = 1;
  }
  else{
      float kv = 0.4;
      float kw = 2;
      float v = kv * sqrt(pow(pose_objetivo[0] - pose[0],2) + pow(pose_objetivo[1] - pose[1],2));
      float phi = atan2(pose_objetivo[1] -pose[1],pose_objetivo[0]- pose[0]);
      float aux = phi-pose[2];
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


float *pursuit(float *pose, float *pose_obj,float Map[40][40], float Map_logs[40][40], float old_erro, float *pose_final){
  float *output = (float*)calloc(6,sizeof(float)); 
  float theta = 0;
  
  float deltat = 0.1;
  int dk =3;
  
  float *new_pose_obj = (float*)calloc(2,sizeof(float)); 

  //Calculo da nova pose objetivo à custa da posiçao objetivo anterior
  new_pose_obj[0] = pose_obj[0] + 0.6*cos(theta);
  new_pose_obj[1] = pose_obj[1] + 0.6*sin(theta);

  //erro integrao
  float novo_erro = sqrt(pow(new_pose_obj[0]- pose[0],2) + pow(new_pose_obj[1]- pose[1],2)) - dk;
  float erro = novo_erro*deltat + old_erro;

//kv = 0.2; ki = 0.1; ks = 1.2; valores otimos 0.6 cos theta

  float kv = 0.2;
  float ki = 0.1;
  float ks = 1.2;

  float vRobot = kv*novo_erro + ki*erro;
  float aux = atan2(new_pose_obj[1]- pose[1],new_pose_obj[0]- pose[0]) - pose[2];
  float wRobot = ks * atan2(sin(aux),cos(aux));


  float pose_errorx = (abs(pose_final[0]- pose[0]));
  float pose_errory = (abs(pose_final[1]- pose[1]));
  float pose_error = sqrt(pow(pose_errorx,2) +  pow(pose_errory,2));
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

