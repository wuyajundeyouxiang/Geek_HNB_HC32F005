#ifndef mycommon_H_
#define mycommon_H_

#define QUEUE_SIZE		  			        20
#define QUEUE_TYPE                          unsigned char
#define MSG                                 QUEUE_TYPE

#define N_MSG                               1       	//  no click
#define N_KEY                               N_MSG   	//  no MSG
#define S_KEY                               2       	//  single click        
#define D_KEY                               3       	//  double click        
#define T_KEY                               4       	//  triple click        
#define Q_KEY                               5       	//  quadruple click     
#define F_KEY                               6       	//  five click  
#define Six_KEY                             7       	//  six click 
#define L_KEY                               8      	    //  long press          
#define L_KEY_Rel                           9
#define MSG_TIM                             10
#define MSG_RT_ADC                          11
#define MIC_I                               12
#define MIC_O                               13
#define MOD_INSERT                          14
#define MOD_PULLOUT                         15
#define MOD_FLAG                            16
#define CHG_NOFULL                          17
#define CHG_FULL                            18
#define CHG_PLUG                            19
#define CHG_UNPLUG                          20
#define ENTER_CHG                           21
#define NOCHG                               22

typedef enum {
    nul_state = 0,
    sleepD,
    sleep,
    standby,
    smk,
    charge,
    preHeat,
    debug,
    upgrade,
} STATE;

extern unsigned char            	msg;
extern char   	                	front;
extern char   	                	rear;
extern STATE  	                	state;

MSG getMsg(void);
void sendMsg(MSG msg);
int get_size(void);
QUEUE_TYPE front_value(void);
unsigned char is_full(void);
unsigned char is_empty(void);
QUEUE_TYPE dequeue(void);
void enqueue(QUEUE_TYPE value);

#endif

