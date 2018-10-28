#include <stdio.h>

#define MAX 100

typedef struct Queues{
    char data[MAX];
    int head;
    int tail;
}Queue;

Queue queue;

void initQueue(){
    queue.head = queue.tail = 0;
}
char getQueueSize() {
    if(queue.head > queue.tail) {
        return (queue.head - queue.tail);
	} else {
        return (queue.tail - queue.head);
	}
}
void PushQueue(char value){
    if((queue.tail+1) % MAX != queue.head) {
        queue.data[queue.tail] = value;
        queue.tail = (queue.tail+1) % MAX;
    } else {
        //队列已满
        //printf("PushQueue Is Full\n");
        //printf(".");
    }
}
char PopQueue( ){
    if(queue.head != queue.tail) {
        int value = queue.data[queue.head];
        queue.head = (queue.head+1)%MAX;
        return value;
    } else {
        //队列为空
        //printf("PopQueue Is Empty\n");
    }
    return -1;
}
void testQueue() {
    int i;
	char flag = 0;
    for(i = 1 ; i<=250 ; i++) {
        PushQueue(i);
    }	
    for(i = 1 ; i<=MAX ; i++){
        printf("%d ",PopQueue());
    }
}
