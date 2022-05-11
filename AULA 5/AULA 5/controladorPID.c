#include<stdio.h>
#include<stdlib.h>

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

void PIDController_Init(PIDController *pid) {

	// C(s) = Kp + Ki/s + Kd.s/((s * tau) + 1)

	//Ganhos 
	pid->Kp = 100;
	pid->Ki = 0.01;
	pid->Kd = 0.01;
	pid->tau = 3.55; //Filtro Derivativo
	
	//Saturação 
	pid->limMax = 65535;
	pid->limMin = 32786;


	//Anti-wind-up
	pid->limMinInt = 65535;
	pid->limMaxInt = 32786;

	//Amostragem
	pid->T = 0.1;

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}

int main (void)
{
	PIDController controladorPID; 
	
	PIDController_Init(&controladorPID);

	printf("%f, %f, %f , %f \n",controladorPID.Kp, controladorPID.Ki,controladorPID.Kd,controladorPID.prevError);

	 float setpointValue = 50000;
	 float measurementValue = 40000;

	 float acaoDeControle; 
	 acaoDeControle = PIDController_Update(&controladorPID , setpointValue, measurementValue);
	 printf("%f, %f, %f, Integrador = %f \n", setpointValue, measurementValue,acaoDeControle, controladorPID.integrator);

	printf("%f, %f, \n",controladorPID.limMaxInt,controladorPID.limMinInt);
	system("pause");
	return 0;
}