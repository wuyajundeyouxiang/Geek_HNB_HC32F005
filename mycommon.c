#include "mycommon.h"

unsigned char            	msg;
char   	                	front;
char   	                	rear;
STATE  	                	state;

QUEUE_TYPE  queue[QUEUE_SIZE + 1]   =   {0};

void enqueue(QUEUE_TYPE value) 
{	
	if(!is_full())
	{
		if (is_empty())
			queue[front] = value;
		queue[rear] = value;
		rear = (rear + 1) % (QUEUE_SIZE + 1);
	}
}

QUEUE_TYPE dequeue(void) 
{	
	int temp = N_MSG;
	if(!is_empty())
	{
		temp = queue[front];
		front = (front + 1) % (QUEUE_SIZE + 1);			
	}
	return temp;
}

unsigned char is_empty(void) 
{   
	return rear == front;
}

unsigned char is_full(void) 
{   
	return (rear + 1) % (QUEUE_SIZE + 1) == front;
}

QUEUE_TYPE front_value(void)
{
	return queue[front];
}

int get_size(void) 
{
	return (rear - front + QUEUE_SIZE + 1) % (QUEUE_SIZE + 1);
}

void sendMsg(MSG msg)
{
	enqueue(msg);
}

MSG getMsg(void)
{
	return dequeue();
}

