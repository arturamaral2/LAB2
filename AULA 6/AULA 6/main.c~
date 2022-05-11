#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <errno.h>

/**************************************************************************
* Projeto Final De Curso - Engenharia de Controle e Automação  - UFMG     *
* Aluno: Diógenes Azevedo Evangelista                                     *
* Orientadora: Maria Auxiliadora Muanis Persechini                        *
* Controlador PID Industrial                                              *
* Placa PCI-6221 da National Instruments                                  *
***************************************************************************/

/**************************************************************************/
// Bibliotecas
/**************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <sys/io.h>
#include <rtai_mbx.h>
#include <math.h>

//Comedi
#include <comedilib.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <math.h>
#include "examples.h"
#include "common.c"
#include "rtai_sched.h"
#include <rtai.h>

/**************************************************************************/
// Definicoes
/**************************************************************************/
//Valor maximo de conversao = 10V
#define VALOR_MAX 65535
//Valor DAQ de 0 Volts na saída analógica
#define ZERO_VOLTS 32780
// Tamanho buffer onde os dados lidos da placa sao armazenados
#define TAM_BUFFER 100
// Temporizacao para transferencia de dados para buffer auxiliar
// e definicao do tamanho do buffer auxiliar
#define MULTIPLICADOR 10
//converte segundo em nanosegundo
#define PARA_NS 1000000000

//numero de amostras
#define AMOSTRAS 1200
/**************************************************************************/
// Variaveis globais
/**************************************************************************/
int valor=1;
int A = 45882;
int B = 52433;
unsigned int buffer_acao_controle[AMOSTRAS];
unsigned int buffer_temperatura[AMOSTRAS];
float buffer_temperatura_filtrado[AMOSTRAS];
unsigned int buffer_potencia[AMOSTRAS];
// Guarda informaçoes do tempo de duração de cada ciclo leitura-calculo-atuação
RTIME cicloInicio[AMOSTRAS];
// Guarda informações sobre a placa
struct parsed_options options;
comedi_t *device;

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

PIDController controladorPID; 
//Threads de ligação entre GUI e Controlador PID
pthread_t threadDados;
pthread_t threadProcessamento;
pthread_t threadTempoReal;
//Variaveis de Entrada de Dados:
float T=0.1;//segundos

int continua=1;
unsigned int k=0;
//Encontrar o numero do erro
extern int errno;

void PIDController_Init(PIDController *pid) {

	// C(s) = Kp + Ki/s + Kd.s/((s * tau) + 1)

	//Ganhos 
	pid->Kp = -14.405;
	pid->Ki = -0.521461;
	pid->Kd = 0;
	pid->tau = 0; //Filtro Derivativo
	
	//Saturação 
	pid->limMax = 65535;
	pid->limMin = 32780;


	//Anti-wind-up
	pid->limMaxInt = 65535;
	pid->limMinInt = 32780;
	
	//Amostragem
	pid->T = 0.1;


	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 50998.0f;

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

/***************************************************************************/
// Funcao para ler um dado na placa
/****************************************************************************/
unsigned int Le_Placa(){
   int ret;
   unsigned int data;
   ret = comedi_data_read(device, options.subdevice, options.channel, options.range, options.aref, &data);
   if(ret < 0){
      comedi_perror("Le_Placa: /dev/comedi0");
      //exit(-1);
   }
   return data;
}// Le_Placa

/**************************************************************************/
// Funcao para escrever um dado na placa
/**************************************************************************/
void Escreve_Placa(unsigned int data){
   int ret;
   unsigned int output=1;//output
   ret = comedi_data_write(device,output, options.channel, options.range, options.aref, data);
   if(ret < 0){
      comedi_perror("Escreve_Placa: /dev/comedi0");
      //exit(-1);
   }
}// Escreve_Placa

/**************************************************************************/
// Funcao para ler dados da placa e armazenar num buffer circular global de
// tamanho definido por TAM_BUFFER. A leitura é ciclica, sendo utilizado um timer
/**************************************************************************/
void* Thread_Carga_Processamento(){
    printf("Inicio Thread_Carga_Processamento\n");
    while(continua){
        pow(2,exp(5));
    }
   printf("Fim Thread_Carga_Processamento\n");
   return 0;
} //Thread_Leitura

/**************************************************************************/
// Funcao para ler dados do buffer global, copiar dados para um buffer auxiliar e,
// apos a liberacao do semaforo para acesso do buffer global, copiar os dados
// do buffer auxiliar para um arquivo
/**************************************************************************/
void* Thread_Carga_Arquivo(){
    int i=0;
    //Arquivo:
    FILE *ArqDados;
    printf("Inicio Thread_Carga_Arquivo\n");
    while(continua){
            ArqDados = fopen( "Dados.txt", "w+" );
        if( ArqDados == NULL ){
             printf("\n  Erro ao Criar Arquivo Carga");
             printf("\n  Significado:%s \n", strerror( errno));
        }
        i=0;
        for(i=0; i<AMOSTRAS; i++){
            fprintf(ArqDados,"%d,%d,%f \n",buffer_acao_controle[i],buffer_temperatura[i],buffer_temperatura_filtrado[i]);
            fflush(ArqDados);
        }
        fclose(ArqDados);
    }
    printf("Fim Thread_Carga_Arquivo\n");
    return 0;
}//  Thread_Escrita_Arquivo

//**************************************************************************/
// Função para inicializar a placa de aquisição de dados
//************************comando voltar terminal em linux**************************************************/
void Inicializa_placa(){
   init_parsed_options(&options);
   options.filename="/dev/comedi0"; //de acordo com o usado em Comedi_script que associou a placa ao comedi0
   options.subdevice=0;	//analog input
   options.channel= 0;	//há 16 canais, escolhi um
   options.range= 0; 	//[-10V a 10V]
   options.aref = AREF_DIFF;// tipo de ligação
   ++options.verbose;	// para exibir informações da leitura

   device = comedi_open(options.filename);
   if(!device){
            comedi_perror(options.filename);
            exit(-1);
   }
    printf("measuring device=%s subdevice=%d channel=%d range=%d analog reference=%d\n",
                     "/dev/comedi0", options.subdevice, options.channel, options.range, options.aref);
}

//**************************************************************************/
// Thread para testar comportamento do sistema com o RTAI
//**************************************************************************/
void *rt_thread(){
    // Variavel que recebe o valor lido da Placa
    unsigned int    dado1;
    float dado_filtrado;
    int itr = 0;
    k=0;
    unsigned int acao_controle;
    unsigned int j=0;
    float set_point = 41335;
    const float a = 0.0141; 
    const float b = 0.9718;
   printf("Inicio rt_thread\n");
    FILE* ArqDados = fopen( "Tempo.txt", "w+" );
    if( ArqDados == NULL ){
        printf("\n  Erro ao Criar Arquivo Tempo");
        printf("\n  Significado: \n", strerror( errno));
    }

    RT_TASK* Main_Task;
    rt_allow_nonroot_hrt();
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("Could not lock memory\n");
        exit(1);
    }
    start_rt_timer(20);
    if (!(Main_Task = rt_task_init(nam2num("MAIN"), 1, 0, 0))) {
        printf("CANNOT INIT MAIN\n");
        exit(1);
    }

    //FAZ TAREFA SER "ACORDADA" PERIODICAMENTE
    rt_task_make_periodic_relative_ns(Main_Task, 0, T*PARA_NS);
    printf("tempo de ciclo : %f segundos\n",T);
    rt_make_hard_real_time();

    buffer_temperatura_filtrado[0] = Le_Placa(); 
    //Programa real
    // Loop que aguarda temporizacao, efetua leitura e escrita de dado na placa
    while(k<AMOSTRAS){
        cicloInicio[k] = rt_get_time_ns();//inicio
        k++;
        //LE
        dado1 = Le_Placa();
        buffer_temperatura[itr] = dado1;

        
        if ((dado1 > 1.5*buffer_temperatura[itr-1]) && itr > 0){
         dado1  = buffer_temperatura[itr-1];
        }
        if( itr > 0 ) {
        
        buffer_temperatura_filtrado[itr] = a*buffer_temperatura[itr] + a*buffer_temperatura[itr-1] + 
        b*buffer_temperatura_filtrado[itr-1];
        
        }
        
        printf("Amostra : %d \t", itr);
       
        acao_controle = (unsigned int) PIDController_Update(&controladorPID,set_point,buffer_temperatura_filtrado[itr]);
        buffer_acao_controle[itr] = acao_controle;
        printf("Temperatura_bruta : %d  \tTemperatura : %f  \t" , dado1 , (((buffer_temperatura_filtrado[itr]-40873)*0.004331) + 29));
        printf("Acao controle : %d \n " , acao_controle);
        
        itr++;
        //ESCREVE
        Escreve_Placa(acao_controle);
        //DORME
        rt_task_wait_period();
    }

    for(j=0;j<AMOSTRAS;j++){
        //SALVA
        fprintf(ArqDados,"%lld\n", cicloInicio[j]);
        fflush(ArqDados);
    }
    //Fim do programa real
   stop_rt_timer ();
   rt_make_soft_real_time();
   rt_task_delete(Main_Task);
   printf("Fim rt_thread\n");
   return 0;
 //  exit(0);
}

void inicializar (){
    int res;
    // Habilita Placa
    Inicializa_placa();
    res = pthread_create(&threadProcessamento, NULL, Thread_Carga_Processamento, NULL);
    if(res != 0){
        perror("Thread_Carga_Processamento creation failed");
        exit(-1);
    }  res = pthread_create(&threadDados, NULL, Thread_Carga_Arquivo, NULL);
    if(res != 0){
        perror("Thread_Carga_Arquivo creation failed");
        exit(-1);
    }
    res = pthread_create(&threadTempoReal, NULL, rt_thread, NULL);
    if(res != 0){
        perror("rt_thread creation failed");
        exit(-1);
    }
}

int main (int argc, char *argv[]){
    //char c;
    k=0;
    
	 PIDController_Init(&controladorPID);
    inicializar();
    while(k<AMOSTRAS){
    }
    continua=0;
    sleep(1);
    return 0;
}
