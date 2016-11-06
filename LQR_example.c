#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

//to change the plant constants, change the initialization of the struct system in line 24
//(although then the controller would, most likely, have to be tuned again)
//to activate live-print of the current control input and plant output, uncomment line 117

sem_t s; // declaration of semaphore

//the shared resources
struct shared {
    double u, x, xd; //values that need to be shared
    bool eof; //flag to signalize that the end of the setpointvalues.txt file has been reached

    struct system {
        const double h, k, t; //plant constants
        double A, B, C; //state-space model
    }sys;
}data = {0.0, 0.0, 0.0, 0, {0.1, 2.0, 3.0}};

void lqr_gains(struct shared *d, double *K, double *F);
double lqr(struct shared *d, double r, double K, double F);
double plant(struct shared *d);

//thread to handle the controller part
//reads the setpoint values one at a time and calculates the respective control outputs
void *controllerThread(void *arg)
{
    struct timespec sleepValue;
    sleepValue.tv_sec = 1; // 1s
    sleepValue.tv_nsec = 0;

    int BUF = 128;        //line buffer
    char line[BUF];
    int n = 0;

    struct shared *d = (struct shared*)arg; //cast to struct pointer

    //compute the control gains
    double K = 0;
    double F = 0;
    lqr_gains(d, &K, &F);

    //open the setpointvalues.txt file
    FILE *in = fopen("setpointvalues.txt", "r");
    if (in == NULL) {
        printf("unable to read setpointvalues.txt\n");
        return NULL;
    }

    int i = 0;

    //read setpoints line by line
    while (fgets(line, BUF, in)) {
        double r;
        sscanf(line, "%lf", &r);

        //don't sem_wait on the first execution to make sure that the first control output comes before the first plant output
        if (i++ > 0)
            sem_wait(&s);

        /* critical section */
        d->u = lqr(d,r,K,F);    //calculate the current control output
        printf("r:%f\n",r);
        /* critical section */

        sem_post(&s);
        nanosleep(&sleepValue, NULL);
    }
    sem_wait(&s);
    d->eof = 1; //signalize that the end of file has been reached
    sem_post(&s);

    fclose(in);
    return NULL;
}

//thread to handle the plant part
//calculates the next state and plant output based on the current control input
//writes all plant outputs to a .txt file
void *plantThread(void *arg)
{
    struct timespec sleepValue;
    sleepValue.tv_sec = 0;
    sleepValue.tv_nsec = 100000000; // 0.1s

    struct shared *d = (struct shared*)arg; //cast to struct pointer

    //open the setpointvalues.txt file
    FILE *out = fopen("out.txt", "w");
    if (out == NULL) {
        printf("unable to open out.txt\n");
        return NULL;
    }

    int i = 0;

    while (1) {
        sem_wait(&s);

        //stop when there are no more setpoints to read
        if (d->eof) {
            sem_post(&s);
            break;
        }

        /* critical section */
        d->xd = d->x;        //next cycle
        double y = plant(d);    //compute current plant output

        fprintf(out, "%f\n", y); //print plant output to file
        //printf("u:%f, y:%f\n",d->u,y);
        /* critical section */

        sem_post(&s);
        nanosleep(&sleepValue, NULL);
        i++;
    }
    fclose(out);
    printf("%d output values written to out.txt\n",i);
    return NULL;
}

//compute the optimal LQR control policy based on the plant model
//in: struct shared d pointer, double K pointer, double F pointer
//returns K and F that are used by reference
void lqr_gains(struct shared *d, double *K, double *F)
{
    //state-space model
    double A = d->sys.A;
    double B = d->sys.B;
    double C = d->sys.C;

    //LQR tuning parameters
    double R = 1.0;
    double Q = C*C*3.0;

    //control policy
    double p = ((R-A*R*A-B*Q*B)/B*B);
    double q = -(R*Q)/(B*B);
    double P = -(p)/2 - sqrt((p/2)*(p/2) - q); //important to choose the second solution here

    //gains
    *K = (B*P*A)/(R+B*P*B);
    *F = 1/(C*B/(1-(A+B**K)));
}

//compute the controller output
//in: struct shared d pointer, current setpoint r, control gain K, setpoint gain F
//return: control output u
double lqr(struct shared *d, double r, double K, double F)
{
    //LQR with setpoint tracking
    double u = K*d->xd + F*r;

    return u;
}

//compute the plant output
//in: struct shared d pointer
//return: plant output y
double plant(struct shared *d)
{
    //state space model derived from the given TF
    double A = d->sys.A;
    double B = d->sys.B;
    double C = d->sys.C;

    d->x = A*d->xd + B*d->u;
    double y = C*d->xd;

    return y;
}

int main()
{
    pthread_t threadArray[2];    // array of thread IDs
    int status;        // error code
    pthread_attr_t threadAttribute;    // thread attribute

    // initialize semaphore s
    // it will be set to 1 after the first control calculation and then act as a mutual exclusion
    // (this is to make sure that the control thread comes first in the beginning)
    sem_init(&s, 0, 0);

    // initialize the thread attribute object
    status = pthread_attr_init(&threadAttribute);
    pthread_attr_setscope(&threadAttribute, PTHREAD_SCOPE_SYSTEM);

    // prepare state-space model
    data.sys.A = exp((1/data.sys.t)*data.sys.h);
    data.sys.B = 1.0;
    data.sys.C = (data.sys.k-data.sys.k*exp((1/data.sys.t)*data.sys.h));

    printf("Starting the simulation...\n");

    // Create our two threads and store their IDs in array threadArray
    pthread_create(&threadArray[0], &threadAttribute, controllerThread, (void *)&data);
    pthread_create(&threadArray[1], &threadAttribute, plantThread, (void *)&data);

    status = pthread_attr_destroy(&threadAttribute);    // destroy the attribute object

    // Wait for threads to finish
    status = pthread_join(threadArray[0], NULL);
    status = pthread_join(threadArray[1], NULL);

    printf("Simulation finished.\n");

    // Destroy semaphore s
    sem_destroy(&s);

    return 0;
}
