// Link layer protocol implementation

#include "link_layer.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// MISC
// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256

#define FLAG 0x7e
#define A 0x03
#define A_receiver 0x01
#define C_SET 0x03
#define C_I 0X00
#define C_UA 0x07
#define C_RR 0x85
#define C_REJ 0x01
#define C_DISC 0x0b
#define BCC_SET A^C_SET
#define BCC_UA A^C_UA
#define BCC_I A^C_I
#define BCC_RR A^C_RR
#define BCC_REJ A^C_REJ
#define BCC_DISC A^C_DISC
#define BCC_DISC_endConnection A_receiver^C_DISC
#define BCC_UA_endConnection A_receiver^C_UA


volatile int STOP = FALSE;
struct termios oldtio;

typedef enum{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_received,
    BCC_2_received,
    STOPS
}STATE;

STATE state = START;


int alarmEnabled = FALSE;
int alarmCount = 0;

int received_UA = FALSE;
int received_RR = FALSE;
int received_UA_endConnection = FALSE;

int Ns = 0;
int Nr = 1;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    state = STOPS;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////


void llopenT (int fd){

    (void)signal(SIGALRM, alarmHandler);

    unsigned char SET[5];
    unsigned char buf_UA_received[BUF_SIZE + 1] = {0};
    
    SET[0] = FLAG;
    SET[1] = A;
    SET[2] = C_SET;
    SET[3] = BCC_SET;
    SET[4] = FLAG;
    

    int bytes_UA_received = 0;
    
    while (!received_UA && alarmCount < 4){
    
        state = START;
        int bytes_SET_transmitted = write (fd, SET, 5);
        printf("%d SET bytes written\n", bytes_SET_transmitted);
    	
        // Wait until all bytes have been written to the serial port
        sleep(1);
    
        alarm(3);
        alarmEnabled = TRUE;
    
        while (state != STOPS) {
	  bytes_UA_received += read(fd, buf_UA_received, 1);
	  if (bytes_UA_received > 0){  
	    switch(state) {
	        case START:
		    if (buf_UA_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    
		    break;

	        case FLAG_RCV:
		    if (buf_UA_received[0] == A) {
		       state = A_RCV;
		    }
		    else if (buf_UA_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }

		    break;

	        case A_RCV:
		    if (buf_UA_received[0] == C_UA) {
		        state = C_RCV;
		    } 
		    else if (buf_UA_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }
		   
		    break;

	        case C_RCV:
		    if (buf_UA_received[0] == (BCC_UA)) {
		        state = BCC_OK;
		    }
		    else if (buf_UA_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else { 
		        state = START;
		    }

		    break;

	        case BCC_OK:
		    if (buf_UA_received[0] == FLAG){
		        state = STOPS;
		        received_UA = TRUE;
		    }
		    else{
		        state = START;
		    }
		    break;
	    }
	    
	  }  
        }
    
    }
    
    printf("%d UA bytes received\n", bytes_UA_received);
    
    if (received_UA == FALSE){
        exit(1);
    }
}

void llopenR(int fd){
    unsigned char buf_SET_received[BUF_SIZE + 1] = {0};

    int bytes_SET_received = 0;
    
    while (state != STOPS) {
      bytes_SET_received += read(fd, buf_SET_received, 1);
      if (bytes_SET_received > 0){  
        switch(state) {
            case START:
                if (buf_SET_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                    
                break;

            case FLAG_RCV:
                if (buf_SET_received[0] == A) {
                   state = A_RCV;
                }
                else if (buf_SET_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else {
                    state = START;
                }

                break;

            case A_RCV:
                if (buf_SET_received[0] == C_SET) {
                    state = C_RCV;
                }
                else if (buf_SET_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else {
                    state = START;
                }

                break;

            case C_RCV:
                if (buf_SET_received[0] == (BCC_SET)) {
                    state = BCC_OK;
                }
                else if (buf_SET_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else { 
                    state = START;
                }

                break;

            case BCC_OK:
                if (buf_SET_received[0] == FLAG){
                    state = STOPS;
                }
                else{
                    state = START;
                }
                
                break;
        }
        
      }       
    }
    
    printf("%d SET bytes received\n", bytes_SET_received);
    
    unsigned char UA[5];
    	UA[0] = FLAG;
    	UA[1] = A;
    	UA[2] = C_UA;
    	UA[3] = BCC_UA;
    	UA[4] = FLAG;
    	
    	/*while (1) {
            bytes_SET_received = read(fd, buf_SET_received, 1);
            printf("%x\n", buf_SET_received[0]);
    	}*/
    
    	int bytes_UA_written = write (fd, UA, 5);
    	printf("%d UA bytes written\n", bytes_UA_written); 
}


int llopenEstablishConnection(LinkLayer connectionParameters)
{

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    //struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize, int fd, unsigned char *I)
{
    printf("Entered write \n");
    received_RR = FALSE;
      
    
    

    (void)signal(SIGALRM, alarmHandler);
    
    unsigned char buf7[bufSize];
    for (int i = 4; i < 4 + bufSize; i++){
        buf7[i-4] = buf[i-4];
    }

    unsigned char BCC_2=0x00;
    printf("BCC_2 = %x\n", BCC_2);
    I[0] = FLAG;
    I[1] = A;
    I[2] = C_I ^ (Ns<<6);
    I[3] = I[1] ^ I[2];
    int j = 0;
    printf("buf_size = %d\n", bufSize);
    for (int i = 4; i < 4 + bufSize; i++){
        //printf("%x\n", I[i]);
        BCC_2 = BCC_2 ^ buf7[i-4];
        printf("%x\n", buf7[i-4]);
    }
    
    
    int k_stuf = 4;
   

    
    for (int i = 4; i < 4 + bufSize; i++){
        
        if (buf7[i-4] == 0x7e){
            I[k_stuf] = 0x7d;
            k_stuf++;
            I[k_stuf] = 0x5e;
            
        }
        else if (buf7[i-4] == 0x7d){
            I[k_stuf] = 0x7d;
           
            k_stuf++;
            I[k_stuf] = 0x5d;
            
        }
        else{
            
            I[k_stuf] = buf7[i-4];
            
        }
        j = k_stuf;
        k_stuf++;
        
    }
    
    
    //printf("Size of buf %d\n", j);
    I[j+1] = BCC_2;
    I[j+2] = FLAG;
    
    //printf("I[4] =  %x\n", I[4]);
    //printf("I[5] =  %x\n", I[5]);
    //printf("I[6] =  %x\n", I[6]);
    
    
    unsigned char buf_RR_received[BUF_SIZE + 1] = {0};
    
    int bytes_RR_received = 0;
    
    int bytes_I_sent = 0;
    
    
    while (!received_RR && alarmCount < 4){
	    state = START;
	    bytes_I_sent = write(fd, I, j+3);
	    printf ("%d I bytes written\n", bytes_I_sent);
	    
	    // Wait until all bytes have been written to the serial port
	    sleep(1);
	    
	    alarm(3);
	    alarmEnabled = TRUE;
	    
	    
	    while (state != STOPS) {
		  bytes_RR_received += read(fd, buf_RR_received, 1);
		  //printf("FLAG RR = %x\n", buf_RR_received[0]);
		  if (bytes_RR_received > 0){  
		    switch(state) {
			case START:
			    if (buf_RR_received[0] == FLAG) {
			    	
				state = FLAG_RCV;
			    }
			    
			    break;

			case FLAG_RCV:
			    if (buf_RR_received[0] == A) {
			      
			       state = A_RCV;
			    }
			    else if (buf_RR_received[0] == FLAG) {
				state = FLAG_RCV;
			    }
			    else {
				state = START;
			    }

			    break;

			case A_RCV: 
			    printf("Nr = %d", Nr);
			    if (buf_RR_received[0] == (C_RR ^ (Nr<<7))) {
			    	
				state = C_RCV;
			    } 
			    /*else if (buf_RR_received[0] == (C_REJ(Nr<<7)){
			        break;
			    }*/
			    else if (buf_RR_received[0] == FLAG) {
				state = FLAG_RCV;
			    }
			    else {
				state = START;
			    }
			   
			    break;

			case C_RCV:
			    if (buf_RR_received[0] == (A^(C_RR ^ (Nr<<7)))) {
				state = BCC_OK;
			    }
			    else if (buf_RR_received[0] == FLAG) {
				state = FLAG_RCV;
			    }
			    else { 
				state = START;
			    }

			    break;

			case BCC_OK:
			    if (buf_RR_received[0] == FLAG){
				state = STOPS;
				received_RR = TRUE;
				
			    }
			    else{
				state = START;
			    }
			    break;
		    }
		}
	   }	
    }	
    for(int i=0;i<26;i++){
        printf("I= %x \n", I[i]);
    }	
    
    printf("%d RR bytes received\n", bytes_RR_received);
    
    if (received_RR == FALSE){ //what happens in this case?

        exit(1);
 
    }
   
    Ns^=1;
    Nr^=1;
    return (bytes_I_sent - 6);
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet, int packetSize, int fd)
{

    //printf("packet[3] = %x\n", packet[3]);
    //printf("sizeofpacket = %d\n", packetSize);
        
        
    unsigned char dataBeforeDestuffing[packetSize*2+2];  
    unsigned char BCC_2;
    unsigned char FLAG2;  
    unsigned char BCC_2_ = 0x00;
    //printf("BCC_2_ = %x\n", BCC_2_);
    
    
    //printf("PACKET SIZE %d\n", sizeof(packet));

    int bytes_I_received = 0;
    int i = 0;
    
    unsigned char buf_I_received[1] = {0};
    unsigned char dataAfterDestuffing[packetSize*2+2];
    int aux = 0;

    
    state = START;
    while (state != STOPS) {
        bytes_I_received += read(fd, buf_I_received, 1);
        if (bytes_I_received > 0){
          switch(state) {
          printf("state= %x\n", state);
            case START:
                printf("buf_I_received[0] FLAG = %x\n", buf_I_received[0]);
                if (buf_I_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                    
                break;

            case FLAG_RCV:
                printf("buf_I_received[0] A = %x\n", buf_I_received[0]);
                if (buf_I_received[0] == A) {
                   state = A_RCV;
                }
                else if (buf_I_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else {
                    state = START;
                }

                break;

            case A_RCV:
                printf("buf_I_received[0] C_I = %x\n", buf_I_received[0]);
                 printf("buf_I_received[0] C_I2 = %x\n", (C_I ^ (Ns<<6)));

                if (buf_I_received[0] == (C_I ^ (Ns<<6))) {
                    printf("Estou aqui!\n");
                    state = C_RCV;
                }
                else if (buf_I_received[0] == FLAG) {
                    printf("Estou aqui!2\n");
                    state = FLAG_RCV;
                }
                else {
                    printf("Estou aqui3\n");
                    state = START;
                }

                break;

            case C_RCV:
            	printf("buf_I_received[0] BCC_I = %x\n", buf_I_received[0]);
                if (buf_I_received[0] == (A^(C_I ^ (Ns<<6)))) {
                    state = BCC_OK;
                }
                else if (buf_I_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else { 
                    state = START;
                }

                break;
                
             case BCC_OK:
                printf("I = %d\n", i);
                printf("buf_I_received[0] DATA = %x\n", buf_I_received[0]);
                dataBeforeDestuffing[i] = buf_I_received[0];
		if (dataBeforeDestuffing[i] == FLAG){
		    BCC_2 = dataBeforeDestuffing[i - 1];
		    FLAG2 = dataBeforeDestuffing[i];
		    //printf ("BCC_2 = %x\n", BCC_2);
		    //printf("FLAG2 = %x\n", FLAG2);
		}
		
		else{		    
		    i++;
		    continue;
		}
		
		for (int j = 0; j < i-1; j++){
		    //printf("%d\n", j);
		    //printf("dataBeforeDestuffing[%d] = %x\n", j, dataBeforeDestuffing[j]);
		}
                state = DATA_received;
                break;
                
                                   
            case DATA_received:
                //printf ("packet size = %d", packetSize);
                //printf("buf_I_received[0] BCC_2 = %x\n", packet[0]);
                //printf ("i = %d\n", i);
                for (int j = 0; j < i-1; j++){
                    //printf("data[%d]= %x\n", j, dataBeforeDestuffing[j]);
                    if (dataBeforeDestuffing[j] == 0x7d){
                        j++;
                        if (dataBeforeDestuffing[j] == 0x5e){
                            dataAfterDestuffing[aux] = 0x7e;
                        }
                        else if (dataBeforeDestuffing[j] == 0x5d) {
                            dataAfterDestuffing[aux] = 0x7d;
                        }
                    }
                    else{
                         dataAfterDestuffing[aux] = dataBeforeDestuffing[j];
                    }
                    aux++;

                    
                    //printf("j = %d\n", j);
                    
                }
                /*for (int j = 0; j < i-1; j++){
                    printf("data[%d] After dstuf= %x\n", j, dataAfterDestuffing[j]);
                    
                }*/
                for (int j = 0; j < aux-1; j++){
                    //printf("data[%d] = %x\n", j, data[j]);
                    BCC_2_ = BCC_2_ ^ dataAfterDestuffing[j];
                    
                }
                
                printf("BBC_2= %x\n", BCC_2);
                printf("BBC_2_= %x\n", BCC_2_);
                
                if (BCC_2 == BCC_2_){
                    printf("Entered MAE DO MARCOS");
                    
                    state = BCC_2_received;
                }
                
                /*else{
                    unsigned char REJ[5];
    	 	    REJ[0] = FLAG;
    		    REJ[1] = A;
    		    REJ[2] = C_REJ ^ (Nr<<7);
    		    REJ[3] = REJ[1] ^ REJ[2];
    		    REJ[4] = FLAG;
    		    
    		    int bytes_REJ_written = write(fd, REJ, 5);
    		    printf("%d REJ bytes written\n", bytes_REJ_written);
                }*/
                  
                break;  

            case BCC_2_received:
                //printf("buf_I_received[0] FLAG = %x\n", packet[0]);
                if (FLAG2 == FLAG){
                    state = STOPS;
                    //Nr^=1;
    		    //Ns^=1;

                }
                else{
                    state = START;
                }
                
                break;
          } 
        }
    }
    
    printf("%d I bytes received\n", bytes_I_received);
    
    unsigned char RR[5];
    RR[0] = FLAG;
    RR[1] = A;
    RR[2] = C_RR ^ (Nr<<7);
    RR[3] = RR[1] ^ RR[2];
    RR[4] = FLAG;
    Nr^=1;
    Ns^=1;
    
    /*while (1) {
        bytes_I_received = read(fd, packet, 1);
        printf("%x\n", packet[0]);
    }*/
    
    int bytes_RR_written = write(fd, RR, 5);
    printf("%d RR bytes written\n", bytes_RR_written);
    memcpy(packet, dataAfterDestuffing, packetSize+6);
    return (bytes_I_received + 4);
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

void llcloseT(int fd){


    	unsigned char abc[5];
    	unsigned char buf_DISC2_received[1] = {0};
    
    	abc[0] = FLAG;
    	abc[1] = A;
    	abc[2] = C_DISC;
    	abc[3] = BCC_DISC;
    	abc[4] = FLAG;
    
    	int bytes_DISC2_received = 0;
    	
    	printf("abc[0] = %x\n", abc[0]);
    	printf("abc[1] = %x\n", abc[1]);
    	printf("abc[2] = %x\n", abc[2]);
    	printf("abc[3] = %x\n", abc[3]);
    	printf("abc[4] = %x\n", abc[4]);
    
   
        int bytes_DISC_transmitted = write(fd, abc, 5);
        printf("%d abc bytes written\n", bytes_DISC_transmitted);
    	
        // Wait until all bytes have been written to the serial port
        sleep(3);
        
        state = START;
    
        while (state != STOPS) {
	     bytes_DISC2_received += read(fd, buf_DISC2_received, 1);
	     if (bytes_DISC2_received > 0){  
	      switch(state) {
	        case START:
		    if (buf_DISC2_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    
		    break;

	        case FLAG_RCV:
		    if (buf_DISC2_received[0] == A_receiver) {
		       state = A_RCV;
		    }
		    else if (buf_DISC2_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }

		    break;

	        case A_RCV:
		    if (buf_DISC2_received[0] == C_DISC) {
		        state = C_RCV;
		    } 
		    else if (buf_DISC2_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }
		   
		    break;

	        case C_RCV:
		    if (buf_DISC2_received[0] == (BCC_DISC_endConnection)) {
		        state = BCC_OK;
		    }
		    else if (buf_DISC2_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else { 
		        state = START;
		    }

		    break;

	        case BCC_OK:
		    if (buf_DISC2_received[0] == FLAG){
		        state = STOPS;
		    }
		    else{
		        state = START;
		    }
		    break;
	    }
	    
	  }  
        }
    
    	printf("%d DISC2 bytes received\n", bytes_DISC2_received);
    
        unsigned char UA_endConnection[5];
    	UA_endConnection[0] = FLAG;
    	UA_endConnection[1] = A_receiver;
    	UA_endConnection[2] = C_UA;
    	UA_endConnection[3] = BCC_UA_endConnection;
    	UA_endConnection[4] = FLAG;
    	
    	/*while (1) {
            bytes_DISC2_received = read(fd, buf_DISC2_received, 1);
            printf("%x\n", buf_DISC2_received[0]);
    	}*/
    
    	int bytes_UA_endConnection_written = write (fd, UA_endConnection, 5);
    	printf("%d UA_endConnection bytes written\n", bytes_UA_endConnection_written);

}

int llcloseR (int fd){
    
    unsigned char buf_DISC_received[BUF_SIZE + 1] = {0};

    int bytes_DISC_received = 0;
    
    state = START;
    
    while (state != STOPS) {
      bytes_DISC_received += read(fd, buf_DISC_received, 1);
      if (bytes_DISC_received > 0){  
        switch(state) {
            case START:
                if (buf_DISC_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                    
                break;

            case FLAG_RCV:
                if (buf_DISC_received[0] == A) {
                   state = A_RCV;
                }
                else if (buf_DISC_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else {
                    state = START;
                }

                break;

            case A_RCV:
                if (buf_DISC_received[0] == C_DISC) {
                    state = C_RCV;
                }
                else if (buf_DISC_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else {
                    state = START;
                }

                break;

            case C_RCV:
                if (buf_DISC_received[0] == (BCC_DISC)) {
                    state = BCC_OK;
                }
                else if (buf_DISC_received[0] == FLAG) {
                    state = FLAG_RCV;
                }
                else { 
                    state = START;
                }

                break;

            case BCC_OK:
                if (buf_DISC_received[0] == FLAG){
                    state = STOPS;
                }
                else{
                    state = START;
                }
                
                break;
        }
        
      }       
    }
    
    printf("%d DISC bytes received\n", bytes_DISC_received);
    
    unsigned char DISC2[5];
    	DISC2[0] = FLAG;
    	DISC2[1] = A_receiver;
    	DISC2[2] = C_DISC;
    	DISC2[3] = BCC_DISC_endConnection;
    	DISC2[4] = FLAG;
    	
    	/*while (1) {
            bytes_DISC_received = read(fd, buf_DISC_received, 1);
            printf("%x\n", buf_DISC_received[0]);
    	}*/ 
    	
    	
    	int bytes_UA_endConnection_received = 0;
    	unsigned char buf_UA_endConnection_received[BUF_SIZE+1] = {0};
    	
    	(void)signal(SIGALRM, alarmHandler);
    	
    	while (!received_UA_endConnection && alarmCount < 4){
    	
    	  state = START;
    	
    	  int bytes_DISC2_written = write (fd, DISC2, 5);
     	  printf("%d DISC2 bytes written\n", bytes_DISC2_written);
     	  
     	  sleep(1);
     	  
     	  alarm(3);
     	  
     	  alarmEnabled = TRUE;
    	
    	  while (state != STOPS) {
	  bytes_UA_endConnection_received += read(fd, buf_UA_endConnection_received, 1);
	  if (bytes_UA_endConnection_received > 0){  
	    switch(state) {
	        case START:
		    if (buf_UA_endConnection_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    
		    break;

	        case FLAG_RCV:
		    if (buf_UA_endConnection_received[0] == A_receiver) {
		       state = A_RCV;
		    }
		    else if (buf_UA_endConnection_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }

		    break;

	        case A_RCV:
		    if (buf_UA_endConnection_received[0] == C_UA) {
		        state = C_RCV;
		    } 
		    else if (buf_UA_endConnection_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else {
		        state = START;
		    }
		   
		    break;

	        case C_RCV:
		    if (buf_UA_endConnection_received[0] == (BCC_UA_endConnection)) {
		        state = BCC_OK;
		    }
		    else if (buf_UA_endConnection_received[0] == FLAG) {
		        state = FLAG_RCV;
		    }
		    else { 
		        state = START;
		    }

		    break;

	        case BCC_OK:
		    if (buf_UA_endConnection_received[0] == FLAG){
		        state = STOPS;
		        received_UA_endConnection = TRUE;
		    }
		    else{
		        state = START;
		    }
		    break;
	    }
	    
	  }  
        }
      }  
     
      
      printf("%d UA_endConnection bytes received\n", bytes_UA_endConnection_received);
      
      if (received_UA_endConnection == FALSE){
          exit(1);
      }
      else{
          return TRUE;
      }
      
      
}

int llcloseEndConnection(int showStatistics, int fd)
{

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    // TODO

    return 0;
}    
