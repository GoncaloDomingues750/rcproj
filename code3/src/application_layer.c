// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include<string.h> 
#include<stdlib.h> 
#include<math.h> 

// MISC
// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1


#define BLOCK_SIZE 128
#define PACKET_SIZE (4*BLOCK_SIZE)
#define DATA_FIELD_SIZE (PACKET_SIZE - 4)
#define S_FRAME_SIZE 7
#define BUF_SIZE (2*BLOCK_SIZE + S_FRAME_SIZE)
#define DATA_PACKET_HEADER 4 
#define CTRL_PACKET_HEADER 3 

enum ControlField
{
    DATA = 0x01,
    START_CTRL = 0x02,
    END_CTRL = 0x03
};

#define TYPE_FILESIZE   0X00
#define TYPE_FILENAME   0X01


#define FLAG 0x7e
#define A 0x03
#define C_SET 0x03
#define C_I 0X00
#define C_UA 0x07
#define BCC_SET A^C_SET
#define BCC_UA A^C_UA


LinkLayer connectionParameters;
int showStatistics = FALSE;
 typedef struct infoPacket{ 
 int size; 
 unsigned char*packet; 
 }infoPacket; 

struct infoPacket makeControlPacket(enum ControlField c, int file_size){ 
     unsigned char* packet = malloc(PACKET_SIZE * sizeof(unsigned char)); 
     infoPacket pi; 
      
     switch(c){ 
        case START_CTRL: 
        packet[0] = START_CTRL;
        break; 
        
        case END_CTRL: 
        packet[0] = END_CTRL; 
        break; 
        
        default: 
        break; 
     } 

      
     packet[1] = 0x00; //type:filesize 
     int octate_num=0; 
     int file_size_copy=file_size; 
      
     while(file_size_copy!=0) 
     { 
        octate_num++; 
        file_size_copy=file_size_copy >> 8;
     } 
      
     packet[2] = octate_num;
      
      
     for(int i=1;i<=octate_num;i++){ 
        packet[2+i]=(file_size>>(8*(octate_num-i))&0xff); 
     } 
      
     pi.packet=packet; 
     pi.size=CTRL_PACKET_HEADER+octate_num; 
      
     return pi; 
 } 

struct infoPacket getNextPacket(FILE *fp, unsigned char seqNum){ 
     unsigned char* packet = malloc(PACKET_SIZE * sizeof(unsigned char)); 
     size_t bytesRead=0; 
     infoPacket pi; 
      
     packet[0]=DATA; 
     packet[1]=seqNum % 255; 
      
     if((bytesRead=fread(packet+4,1,DATA_FIELD_SIZE,fp))<0){ 
     printf("Errorreadingfromfile"); 
     } 
      
     packet[2]=(bytesRead & 0xFF00) >> 8; 
     packet[3]=bytesRead & 0x00FF; 
      
     pi.packet=packet; 
     pi.size=bytesRead+DATA_PACKET_HEADER; 
      
      
     return pi; 
 } 

 int readControlPacket(enum ControlField c,struct infoPacket pi){ 
     unsigned char *packet=pi.packet; 
     
     printf("packet[0]== %x", packet[0]);
  
     if((c==START_CTRL) && (packet[0]!=START_CTRL)){ 
        return-1;} 
     if(c==END_CTRL && packet[0]!=END_CTRL)return-1; 
     if(c==DATA)return-1;     
     if(packet[1]!=0){
        return-1;
     }   
      
      
     int k=packet[2]; 
     int filesize=0; 
     for(int i=0;i<k;i++){ 
     filesize=filesize<<8; 
     filesize+=packet[3+i]; 
     } 
     
     printf("Filesize == %d", filesize);
     return filesize; 
 } 

 //returns size of data read 
 int parseNextPacket(struct infoPacket pi,FILE*fp,unsigned char seqNum){ 
     unsigned char*packet=pi.packet; 
     if(packet[0]!=DATA){ 
     printf("Not data packet!"); 
     return-1; 
     } 
     if(packet[1]!=seqNum){ 
     printf("Not right packet sequence number!"); 
     return-1; 
     } 
      
     printf("%d,%d\n",packet[2],packet[3]); 
      
     int packetsize=(packet[2]*256)+packet[3]; 
      
     fwrite(packet+4,packetsize,1,fp); 
      
     return packetsize; 
 }

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{


    strcpy(connectionParameters.serialPort, serialPort);
    if(strcmp(role, "tx") == 0){
        connectionParameters.role = LlTx;
    }
    else if (strcmp(role, "rx") == 0){
        connectionParameters.role = LlRx;
    }
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    
    int ctrlPacketSize;
    
    int fd = llopenEstablishConnection(connectionParameters);
    
    unsigned char bytes_I_sent;
    int closeConnection = FALSE;
    unsigned char *packet; 
    int bytes; 
    unsigned char seqNum=0;
   
    if (connectionParameters.role == LlTx){
            
	    llopenT(fd);
        //read/write packet
        
        FILE *fp=NULL; 
        //char *temp= malloc(50*sizeof(char)); 
        //sprintf(temp,"../%s",filename); 
        fp=fopen("penguin.gif","rb");//read-binarymode 
          
        if(fp==NULL){ 
        perror("Unable to openfile\n"); 
        } 
          
        fseek(fp,0,SEEK_END); 
        int file_size=ftell(fp);
        fseek(fp,0,SEEK_SET); 
          
        printf("filesize:%d\n",file_size); 
        //sendstartpacket 
        struct infoPacket pi=makeControlPacket(START_CTRL,file_size); 
        for(int i=0;i<pi.size;i++)printf("%x",pi.packet[i]); 
        printf("Packet size: %d",pi.size);
        unsigned char I[pi.size];
        ctrlPacketSize = pi.size;
        bytes=llwrite(pi.packet,pi.size, fd, &I);
        int tbytes = 0;

        while(TRUE) 
        { 
            pi=getNextPacket(fp,seqNum); 
            packet=pi.packet; 
            int packet_size=pi.size; 
            unsigned char I1[packet_size];
             
            if(packet_size>DATA_PACKET_HEADER){ 
            printf("Packet size is: %d \n", packet_size);
            bytes=llwrite(packet,packet_size,fd, &I1);
            printf("%d",bytes); 
            } 
            else{ 
            break; 
            } 
          
            free(packet); 
          
            if(bytes==-1) return; 
            tbytes += bytes;
          
            seqNum++; 
            printf("Sent bytes: %d\n", tbytes);
        } 
          
        fclose(fp); 
          
        //sendendpacket 
        pi=makeControlPacket(END_CTRL,file_size); 
        unsigned char I2[pi.size];
        bytes=llwrite(pi.packet,pi.size,fd,&I2); 
      
      
        llcloseT(fd);
        sleep(1);
    }
    
        
    if (connectionParameters.role == LlRx){
    	llopenR(fd);
        packet= malloc(PACKET_SIZE*sizeof(unsigned char));//usar realloc no llread 
  
         //read start packet 
         bytes=llread(packet,1, fd);
         struct infoPacket pi={bytes,packet}; 
         int file_size= readControlPacket(START_CTRL,pi);
         printf("Filesize: %d \n", file_size);
          
         if(file_size==-1){ 
         printf("Error reading startcontrol packet!"); 
         } 
         

         //readdatapackettofile 
         char *temp= malloc(50*sizeof(char));
         //sprintf(temp,"received-%s",filename); 
         //printf("tempfilename:%s",temp); 

         FILE *fp = fopen("penguin-received.gif","wb");//read-binarymode 
          
         if(fp==NULL){ 
            perror("Unable to open file\n");
            return; 
         } 
          
         free(temp); 
          
         //getpackets 
         int bytes_received=0; 
         while(bytes_received<file_size) 
         { 
            printf("packet[0] == %d \n",packet[0]);
            printf("BytesRecieved %d \n", bytes_received);
            if((file_size-bytes_received)/PACKET_SIZE==0) bytes=llread(packet,(file_size - bytes_received)%PACKET_SIZE, fd); 
            else bytes=llread(packet,PACKET_SIZE, fd);
            printf("Im hereeeee");
            pi.size=bytes; 
            pi.packet=packet; 
            bytes=parseNextPacket(pi,fp,seqNum); 
            printf("\n\nBytes:%d",bytes); 
            bytes_received+=(bytes); 
            printf("\nBytesreceived:%d\n",bytes_received); 
            seqNum++; 

         } 
          
         if(bytes_received!=file_size){ 
            printf("Recieved different number of bytesthanfilesize.\n"); 
         } 
          
         fclose(fp); 
          
         //readendpacket 
         bytes=llread(packet,1, fd);
         struct infoPacket pi2={bytes,packet};
         int file_size2=readControlPacket(END_CTRL,pi2); 
         if(file_size2==-1){ 
            printf("Errorreadingendcontrolpacket!"); 
         } 
         if(file_size!=file_size2)printf("Errorinpacketfilesizes"); 
         free(packet); 
    
    	closeConnection = llcloseR(fd);    
    	sleep(1);
    	if (closeConnection == TRUE){
            llcloseEndConnection(showStatistics, fd);
            printf("connection closed\n");
        }
          
    } 
        
        
    
    
    // TODO
}
