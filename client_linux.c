//Proyecto Tiempo real

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define TAMBUF 32
#define H 0
#define M 1
#define S 2
#define w 3
#define d 4
#define m 5
#define y 6
#define X 7
#define Y 8
#define Z 9

int Modo_flag = 0, out_flag = 0; //flag para sincronizar los hilos

pthread_mutex_t mutex; 		//mutex para protejer variables

int find_num(char *str);
int write_data(int fd, void* buf);
int read_data(int fd, void* buf);
int write_and_read(int fd, int comando);
int GetTime(int num, int selec_time);
int num_cifra(int numero);

void *Log_function(void *arg){ 	//Funcion de hilo secundario recepcion de datos de reloj
    
    int fd = *((int *)arg); 	//Obtener el descriptor de fichelo de uart
    int log_fd; 		//declarar el descriptor de fichero de archivo texto
    int num_read = 0, num_write = 0; 
    char log_buf[40]; 		//buffer para guardar datos recibido
    
    //Abrir o crear fichero de texto    
    log_fd = open("log.txt", O_WRONLY|O_CREAT, 0777);
    if(log_fd==-1) 		//En caso no ha creado o abierto el fichero de texto
    { 
      printf("No se puede abrir el archivo\n");
    }
    else			//En caso de haya creado correctamente
    { 
    	    printf("Archivo creado\n");

	    while(1){ 
		if(!Modo_flag)	// 1: En modo de enviar comando cliente, 0: En modo de guardar log 
		{ 
		    pthread_mutex_lock(&mutex); 
		    memset(log_buf, 0, sizeof(log_buf));  //Vaciar el mutex
		    num_read = read(fd, log_buf, 33); 	  //Leer datos desde uart
		    if(log_buf[0] == 'L' && num_read > 0) //Comrpbar si dato de valido
		    { 
				num_write = write(log_fd, log_buf, strlen(log_buf)); //Escribir en el fichero de texto
		    }
		    tcflush(fd,TCIFLUSH); 		 //Vaciar el buffer
		    pthread_mutex_unlock(&mutex);
		}
		if(out_flag) break; 			//Señal para indicar que va cerrar el programa
		sleep(1);
	    }
	    close(log_fd); //Cerrar el programa
    }

}

int main(int argc, char *argv[])
{
        //Declarar variables
	int Com_num, fd, option, sub_option;
	char Com_dev[16] = {};
	char buf[TAMBUF];

        pthread_t Log_thread;

	if(argc < 2)  //Comprobar si haya iniciado correctamente
	{     
		printf("Introduce el numero de puerto correspondiente.\n");
		//exit(1);
		return 1;
	}

	Com_num = find_num(argv[1]); //Comprobar el puerto
	if(Com_num < 0)
	{
		printf("Introduce el numero de puerto correcto.\n");
		//exit(2);
		return 2;
	}
	
        //Generar la direccion de puerto
	sprintf(Com_dev, "/dev/ttyACM%d", Com_num);
	
	//Abrir el archivo
	fd = open(Com_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0)
	{
		perror("Error al abrir el puerto serie");
		return 3;
	}
    
        //Configurar las propiedades de puerto
	struct termios configuration;

	memset(&configuration,0,sizeof(configuration));
	cfsetispeed(&configuration,B115200);
	cfsetospeed(&configuration,B115200);
	configuration.c_cflag|=CS8;
	configuration.c_cflag&=~PARENB;
	configuration.c_cflag&=~CSTOPB;
	configuration.c_iflag = IGNPAR;
	configuration.c_oflag = 0;
	configuration.c_cc[VTIME]=10;
	configuration.c_cc[VMIN]=3;
    
	tcsetattr(fd,TCSANOW,&configuration);
        //Vaciar los buffer de recepcion y envio
	tcflush(fd,TCIOFLUSH);
	//Realizar un envio
	write_data(fd, "C:00 ");
	memset(buf, 0, TAMBUF);

       //Crear el hilo
       if(pthread_create(&Log_thread, NULL, (void *)Log_function, &fd))
       {
           	printf("error al crear el multihilo");
        	return 0;
    	}
    
    	//proceso de interfaz
	do{
		system("clear");
		printf("\nEl puerto de dispositivo es: /dev/ttyACM%d", Com_num);
		printf("\nCliente para Interacción con Placa stm32f411re\n");
		printf("--------------------------------------\n");
		printf(" 1. Configurar las horas\n");
		printf(" 2. Configurar las fechas\n");
		printf(" 3. Configurar el alarma\n");
		printf(" 4. Salir\n");
		printf("--------------------------------------\n");
		printf("Elige una opción (1-4): ");
		fgets(buf,sizeof(buf),stdin);
		sscanf(buf,"%d",&option);
		switch(option){
			case 1: //Configurar las horas, minutos y segundos
                do{
                    system("clear");
                    printf("El puerto de dispositivo es: /dev/ttyACM%d", Com_num);
                    printf("\nCliente para Interacción con Placa stm32f411re\n");
                    printf(" Configurar las horas:\n");
                    printf("--------------------------------------\n");
                    printf(" 1. Configurar la hora\n");
                    printf(" 2. Configurar los minutos\n");
                    printf(" 3. Configurar los segundos\n");
                    printf(" 4. Salir\n");
                    printf("--------------------------------------\n");
                    printf("Elige una opción (1-4): ");
                    fgets(buf,sizeof(buf),stdin);
                    sscanf(buf,"%d",&sub_option);
                    switch(sub_option){
                        case 1:
                            do{ //Configurar la hora
                                printf("\n Indica la hora (0 - 23)h: ");
                            }while(!write_and_read(fd, H));
                            break;
                        case 2:
                            do{ //Configurar los minutos
                                printf("\n Indica el minuto (0 - 59)min: ");
                            }while(!write_and_read(fd, M));
                            break;
                        case 3:
                            do{//Configurar los segundos
                                printf("\n Indica el segundo (0 - 59)sec: ");
                            }while(!write_and_read(fd, S));
                            break;
                        case 4://salir de configuracion de las horas
                            printf("\n Salir de la configuracion.");
                            break;
                        default://En caso el dato no esta dentro de rango
                            printf("\n Introducir la opcion correcta.");
                            break;
                    }
                }while(sub_option != 4);
                sub_option = 0;
				break;
			case 2: //Configurar la fecha
                 do{
                    system("clear");
                    printf("El puerto de dispositivo es: /dev/ttyACM%d", Com_num);
                    printf("\nCliente para Interacción con Placa stm32f411re\n");
                    printf(" Configurar las fechas (dd/mm/yy):\n");
                    printf("--------------------------------------\n");
                    printf(" 1. Configurar la fecha\n");
                    printf(" 2. Salir\n");
                    printf("--------------------------------------\n");
                    printf("Elige una opción (1-2): ");
                    fgets(buf,sizeof(buf),stdin);
                    sscanf(buf,"%d",&sub_option);
                    switch(sub_option){
                        case 1:
                            do{ //Configurar la semana
                                printf("\n Indica la semana (1->Lunes,...,7->Domingo): ");
                            }while(!write_and_read(fd, w));

                            do{ //Configurar el dia
                                printf("\n Indica el dia (dd): ");
                            }while(!write_and_read(fd, d));

                            do{//Configurar el mes
                                printf("\n Indica el mes (mm): ");
                            }while(!write_and_read(fd, m));

                            do{//Configurar el año
                                printf("\n Indica el año (yy): ");
                            }while(!write_and_read(fd, y));
                            break;
                        case 2:
                            printf("\n Salir de la configuracion.");
                            break;
                        default:
                            printf("\n Introducir la opcion correcta.");
                            break;
                    }
                }while(sub_option != 2);
                sub_option = 0;
				break;
			case 3: //configurar el alarma
                do{
                    system("clear");
                    printf("El puerto de dispositivo es: /dev/ttyACM%d", Com_num);
                    printf("\nCliente para Interacción con Placa stm32f411re\n");
                    printf(" Configurar las horas del alarma:\n");
                    printf("--------------------------------------\n");
                    printf(" 1. Configurar la hora del alarma\n");
                    printf(" 2. Configurar los minutos del alarma\n");
                    printf(" 3. Configurar los segundos del alarma\n");
                    printf(" 4. Salir\n");
                    printf("--------------------------------------\n");
                    printf("Elige una opción (1-4): ");
                    fgets(buf,sizeof(buf),stdin);
                    sscanf(buf,"%d",&sub_option);
                    switch(sub_option){
                        case 1:
                            do{ //Configurar la hora
                                printf("\n Indica la hora (0 - 23)h: ");
                            }while(!write_and_read(fd, X));
                            break;
                        case 2:
                            do{ //Configurar los minutos
                                printf("\n Indica el minuto (0 - 59)min: ");
                            }while(!write_and_read(fd, Y));
                            break;
                        case 3:
                            do{//Configurar los segundos
                                printf("\n Indica el segundo (0 - 59)sec: ");
                            }while(!write_and_read(fd, Z));
                            break;
                        case 4:
                            printf("\n Salir de la configuracion.");
                            break;
                        default:
                            printf("\n Introducir la opcion correcta.");
                            break;
                    }
                }while(sub_option != 4);
                sub_option = 0;
				break;
            case 4:
                printf("\n Salir del cliente.");
				break;
			default:
                printf("\n Introducir la opcion correcta.");
                break;
		}
	}while(option != 4);
	
	out_flag = 1;

    pthread_join(Log_thread, NULL); //esperar que el hilo haya finalizado
	
	printf("\n Aplicacion cerrada.\n");

	close(fd); //cerrar el puerto
}



//Obtener los numeros de un string
int find_num(char *str)
{
        //Buscar el numero de puerto
        //En caso el numero no es seguido se devuelve un -1 indicando que es erroneo
	int i, num = 0, Num_flag = 0, flag;
	for (i = 0; i < strlen(str); i++)
	{
		flag = 0;
		if(str[i] >= '0' && str[i] <= '9')
		{
			flag = 1;
			num = num * 10;
			num += str[i] - '0';
		}
		if(flag) Num_flag = 1;
		if(!flag && Num_flag && i != strlen(str)) return -1;
	}
	return num;	
};

int write_data(int fd, void* buf) //realiza el envio de mensaje y vaciar el buffer
{
	int num_sent = 0;
	num_sent = write(fd, buf, strlen(buf));
	tcflush(fd,TCOFLUSH);
	return num_sent;
};
int read_data(int fd, void* buf) //realiza el envio de mensaje y vaciar el buffer
{
	int num_rec = 0;
	num_rec = read(fd, buf, 3);
	tcflush(fd,TCIFLUSH);
	return num_rec;
};

int write_and_read(int fd, int comando){
 
        //Envio y construccion de mensaje
   	Modo_flag = 1;
   	pthread_mutex_lock(&mutex);
   	tcflush(fd,TCIFLUSH);
   	write_data(fd, "O:00 "); 	//realiza un envio antes de enviar el mensaje
    
   	char buf[TAMBUF], buf_get[TAMBUF];
    	int numb = 0, next = 0;
   	int num_write = 0, num_read = 0;
    	int count = 0;

	//Comando y respuesta
	char *COMANDOS[] = {"H","M","S","w","d","m","y","X","Y","Z"};
	char *RESPUESTAS[] = {"Y","N"};

	//Obtener el numero y comprobar el rango
	memset(buf_get, 0, TAMBUF);
	fgets(buf_get,sizeof(buf_get),stdin);
	sscanf(buf_get,"%d",&numb);

    	if(comando == y && (num_cifra(numb) == 2 || num_cifra(numb) == 4)) numb = numb%100;
	
	if(GetTime(numb, comando))
	{


		//Juntar el comando y el dato en formato -> comando:dato
		memset(buf, 0, TAMBUF);
		sprintf(buf, "%s:%-3d", COMANDOS[comando], numb);

		//Enviar el comando
		num_write = write_data(fd, buf);
		
		memset(buf, 0, TAMBUF);

		//Recibir el respuesta
		num_read = read_data(fd, buf);
		
		memset(buf, 0, TAMBUF);

		if(num_read > 0)
		{
			next = 1;
		}
		else	//indicar informacion de transmicion y recepcion si haya producido un error
		{ 
		    printf("\n Numero: %d. Comando: %s", num_write, buf);
		    printf("\n Numero: %d, Respuesta: %s", num_read, buf);
            	    printf("\n No ha recibido la respuesta");
		    next = 0;
		}
	}
	else
	{
		printf("\n Introduce la hora correcta");
		next = 0;
	}

        pthread_mutex_unlock(&mutex);
        Modo_flag = 0;

        return next;
}

int GetTime(int num, int selec_time)
{
    	//comprobar si el numero introducido esta dentro de rango, el rango esta indicada por el selec_time
    	int range_high[10] = {23, 59, 59, 7, 31, 12, 9999, 23, 59, 59};
    	int range_low[10] = {0, 0, 0, 1, 1, 1, 1, 0, 0, 0};

    	if(range_low[selec_time] <= num && num <= range_high[selec_time]){ return 1;}
    	else{return 0;}
}

int num_cifra(int numero){ //funcion para indicar el numero de cifra que tiene un numero
    	int count = 0;
    	do{
        	numero /= 10;
        count++;
    	}while(numero != 0);
    	return count;
}
