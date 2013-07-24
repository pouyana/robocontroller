#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>

/**
 * Coordinate struct with x and y and also the angle
 * used instructions here:
 * http://www.cs.usfca.edu/~wolber/SoftwareDev/C/CStructs.htm
 */
typedef struct {
    double x;
    double y;
    double alpha;
} Coord;

/*
 *  print how it works. how the program works.
 */
void print_usage(){
    printf("Usage: please use -h to see help.");
}

/*
 * print help out. how to use the program is defined here. 
 */
void print_help(){
    printf("Controller.o is a program to generate trajectory for a 2wheel\nrobot. It is licensed with GNU GPL 2.\n---------------------\nTo pass arguements to the application you can use these options:\n-s : passes the start coordinates, default values are set with (0,0,90).\n-e : passes the destiation coordinates. must be set.\n-h : show you this help.\nExample:\ncontroller.o -s 0,0,90  -e 1,1,-90\nwill give you the trajectory from 0,0,looking upward (x,y) to 1,1 looking downward.\n");
}

/*
 * deg to rad converter sinus/cos and tan need rad.
 */
double degtorad(double rad){
    return rad * M_PI / 180;
}

/*
 * string split function with strok, needed to get values.
 * see: stackoverflow.com/q/9210528/
 */
Coord str_split(char * a_str,const char * a_delim)
{
    char * del_str;
    Coord coord;
    del_str = strtok(a_str,a_delim);
    int counter = 0;
    while (del_str != NULL){
        if(counter == 0){
            coord.x = atof(del_str);
        }
        if(counter == 1){
            coord.y = atof(del_str);
        }
        if(counter == 2){
            coord.alpha = degtorad(atof(del_str));
        }
        counter++;
        del_str = strtok(NULL,",");
    }
    return coord;
}

/*
 *  Claculating the new x,y with the help of sin and cos function
 *  sample time as it is stated is 100ms
 */
int CalcNextPosition(double x,double y, double * u,  double * omega, double * x_n, double * y_n,double *v_x,double *v_y)
{
    * v_x = * u * cos(*omega);
    * v_y = * u * sin(*omega);
    //    printf("LastX=%f,Lasty=%f\n",x,y);
    * x_n = x + * v_x * 0.1; //100ms (0.1 sec);
    * y_n = y + * v_y * 0.1;
    return 0;
}

/*
 *  Contorller gives you the next velocity and angle
 */
int CalcNextVelocities(double u_max, double x,double y, double phi, double *u, double *omega)
{

    double c, theta;
    double alpha;
    double e;
    const double gamma=1.0, beta=2.9,h=2.0;
    const double epsilon = 0.00001;

    e = sqrt(x*x+y*y);

    if(e > epsilon) {                   //nicht am Ziel
        theta = atan2(-y,-x);
        if(theta >  M_PI)   theta -= 2.0 * M_PI;
        if(theta < -M_PI)   theta += 2.0 * M_PI;

        alpha = theta - phi;
        if(alpha > M_PI )   alpha -= 2.0 * M_PI;
        if(alpha < -M_PI)   alpha += 2.0 * M_PI;

        *u = gamma * e;
        if (*u >= u_max) {
            *u = u_max;
        }

        if (fabs(alpha) > epsilon) {
            c = (sin(alpha) + (h * theta * sin(alpha)/alpha +beta * alpha))/e;
        }
        else {
            c = (alpha + alpha * beta + h * theta) / e;
        }
        *omega = c * *u;
    }
    else {
        * u = 0.0;
        * omega = 0.0;
    }
    return 0;
}

/*
 *  main loop with arguments validation
 */
int main(int argc, char **argv) {
    //default valus are set, when nothing is set.
    int option = 0;
    char *  startvals = NULL;
    char *  endvals = NULL;
    char *  max_vel = NULL;
    Coord startCoord;
    startCoord.x = 0.0;
    startCoord.y = 0.0;
    startCoord.alpha = degtorad(90);
    Coord endCoord;
    //(:) means that it has a value.
    while ((option = getopt(argc, argv,"s:v:he:")) != -1){
        switch (option) {
            case 's' : 
                startvals = optarg ;
                break;
            case 'e' : 
                endvals = optarg;
                break;
            case 'h' : 
                print_help(); 
                break;
            case 'v' :
                max_vel = optarg;
                break;
            default: 
                print_usage(); 
                exit(EXIT_FAILURE);
        }
        double max_speed = atof(max_vel);
        //double max_speed = atof(max_vel);
        if(startvals){
            startCoord = str_split(startvals,",");
        }
        //if the end coordinates are set then do the calculation
        if(endvals && max_speed){
            endCoord=str_split(endvals,",");
            printf("\nEnd Coordniates:\nx=%5.2f,y=%5.2f,alpha=%5.2f\n",endCoord.x,endCoord.y,endCoord.alpha);
            printf("\nStart Coordinates:\nx=%5.2f,y=%5.2f,alpha=%5.2f\n",startCoord.x,startCoord.y,startCoord.alpha);
            //start calculation
            double omega[10];
            double u[10];
            double x_n[10];
            double y_n[10];
            double v_x[10];
            double v_y[10];
            int sim_count = 0;
            //max speed set as six will be added as a paramter.
            CalcNextVelocities(max_speed,endCoord.x,endCoord.y,endCoord.alpha,u,omega);
            CalcNextPosition(startCoord.x,startCoord.y,u,omega,x_n,y_n,v_x,v_y);
            //CalcNextVelocities(20,12,-4,M_PI/2,u,omega);
            //CalcNextPosition(0,0,u,omega,x_n,y_n,v_x,v_y);
            printf("Starting Simulation:\n");
            while(*u>00.1){
                sim_count++;
                CalcNextVelocities(max_speed,*x_n-endCoord.x ,*y_n-endCoord.y,*omega,u,omega);
                //CalcNextVelocities(20,*x_n-12,*y_n+4,*omega,u,omega);
                CalcNextPosition(*x_n,*y_n,u,omega,x_n,y_n,v_x,v_y);
                printf("X=%2.5f\tY=%2.5f\t,V=%2.5f\t,alpha=%2.5f\t,Vx=%2.5f\t,Vy=%2.5f\n",*x_n,*y_n,*u,*omega,*v_x,*v_y);
            }
            printf ("Finished Simulation with %i simulations\n",sim_count);
        }
        else {
            print_usage();
        }
    }
}
