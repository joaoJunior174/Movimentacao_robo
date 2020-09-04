/*
 * File:          campo_potencial.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
struct Obstaculos{

  int i;
  int j;

};
//retorna a distância euclidiana  
float dist(float x1,float y1,float x2, float y2){

  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
 
} 
 //localiza qual o obstáculo mais próximo da célula
int obstaculoProximo(int x,int y,struct Obstaculos ob[10]){
  
   float max=10000;
   float aux=-1;
   
   for(int index=0;index<10;index++){
   
   
        float d = dist(x,y,ob[index].i,ob[index].j);
        
        if(d < max){
        
           max=d;
           aux=index;
        }
       
   }
   
   return aux;

} 
 //função que calcula o gradiente para cada celula
void calcularPotencial(int xg, int yg,float grad[20][20],struct Obstaculos ob[10],float po,float k){

  for(int x=0;x<20;x++){
  
      for(int y=0;y<20;y++){
       
         int o=obstaculoProximo(x,y,ob);
        
         float pq = dist(x,y,ob[o].i,ob[o].j);
         float urep = 0;
         
         if(pq ==0)
            pq=0.1;
          //aqui calcula Urep, com a formula que est no vdeo
         if(pq<po){
             //printf("%f\n",pq);
             urep=0.5*k*pow(((1.0/pq)+(1.0/po)),2);
         }   
          //aqui  o restante, Uatt + Urep 
         grad[x][y]=0.5*k*pow(dist(x,y,xg,yg),2) + urep;
        
      }
     
  }

} 
 //aqui imprime a matriz, ou melhor,  o grid od mundo
void mostrarMatriz(float grad[20][20]){

 for(int x=0;x<20;x++){
  
      for(int y=0;y<20;y++){
       
        printf("%.1f ",grad[x][y]);
        
      }
      
      printf("\n");
     
  }

} 
//aqui a função pega o menor valor dos vizinhos de uma celula na posicao x,y, e coloca sua posicao no vetor pos[2]
void proximoVizinho(int pos[2],int x,int y, float grad[20][20]){

 float max=1000;
 
 
 if(y-1>=0 && grad[x][y-1] < max){
 
    max=grad[x][y-1];
    pos[0]=x;pos[1]=y-1;
    
 }
 if(y+1<=19 && grad[x][y+1] < max){
 
    max=grad[x][y+1];
    pos[0]=x;pos[1]=y+1;
 }
 if(x-1>=0 && grad[x-1][y] < max){
 
    max=grad[x-1][y];
    pos[0]=x-1;pos[1]=y;
 }
 if(x+1<=19 && grad[x+1][y] < max){
 
    max=grad[x+1][y];
    pos[0]=x+1;pos[1]=y;
 }
 
 
 
}
 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
 //aqui inicializa as variaveis e motores
 WbDeviceTag wheel1,wheel2,wheel3,wheel4;
  wheel1 = wb_robot_get_device("wheel1");
  wheel2 = wb_robot_get_device("wheel2");
  wheel3 = wb_robot_get_device("wheel3");
  wheel4 = wb_robot_get_device("wheel4");
  
   wb_motor_set_position(wheel1,INFINITY);
   wb_motor_set_position(wheel2,INFINITY);
   wb_motor_set_position(wheel3,INFINITY);
   wb_motor_set_position(wheel4,INFINITY);
  //aqui pega o dispositivo gps para poder ter a localizacao do robo
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
    int xg=19;//posicao goal em x
    int yg=19;//posicao goal em y
    float grad[20][20];//mundo
    struct Obstaculos ob[10];//quantidade de obstaculos
    float po=4.0;//distancia toleravel minima
    float k=0.1;//aqui esse  o k para ambos os U
    ob[0].i=3;ob[0].j=3;//aqui estou dizendo a posicao na qual tem um obtaculo dentro do meu mundo
    ob[1].i=1;ob[1].j=11;
    ob[2].i=5;ob[2].j=9;
    ob[3].i=5;ob[3].j=17;
    ob[4].i=9;ob[4].j=1;
    ob[5].i=9;ob[5].j=13;
    ob[6].i=14;ob[6].j=4;
    ob[7].i=15;ob[7].j=8;
    ob[8].i=15;ob[8].j=17;
    ob[9].i=17;ob[9].j=1;
    
    calcularPotencial(xg,yg,grad,ob,po,k);//chama função que calcula o campo potencial
    
    mostrarMatriz(grad);
    
    while (wb_robot_step(TIME_STEP) != -1) {
  
      const double *valores = wb_gps_get_values(gps);//pega coordenadas do gps
      //aqui transforma as coordenadas do gps em indices da matriz do grid
      int x = abs(valores[2]*20/14);
      int y= abs(valores[0]*20/14);
      
      //printf("%i %i\n",x,y);
      //variavel auziliar 
      int pos[2];
      //aqui retorna a posicao na qual o robo deve ir, e coloca no vetor pos
      proximoVizinho(pos,x,y,grad);
  
      int xm=pos[0]-x;
      int ym=pos[1]-y;
      float v1=0.7,v2=0.7,v3=0.7,v4=0.7;
      
     // printf("proximo vizinho %i %i\n",pos[0],pos[1]);
       //printf("cond %i %i\n",xm,ym);
      //esses if abaixo faz a logica do (x2-x1,y2-y1) apresentada no relatório
      if(xm<0 && ym ==0){//carro sobe
     
        v1=0.7;
        v2=-0.7;
        v3=-0.7;
        v4=0.7;
        
      
      }else if(xm ==0 && ym >0){//carro vai para frente
      
       v1=0.7;
       v2=v1;
       v3=v1;
       v4=v1;
      
      }else if(xm ==0 && ym <0){//carro vai para tras
      
       v1=-0.7;
       v2=v1;
       v3=v1;
       v4=v1;
      
      
      }else if(xm > 0 && ym ==0){//carro desce
       
        v1=-0.7;
        v2=0.7;
        v3=0.7;
        v4=-0.7;
      
      }
  
        //aplica as velocidades necessárias para o carrinho se movimentar
        wb_motor_set_velocity(wheel1,v1);
        wb_motor_set_velocity(wheel2,v2);
        wb_motor_set_velocity(wheel3,v3);
        wb_motor_set_velocity(wheel4,v4);
     
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
