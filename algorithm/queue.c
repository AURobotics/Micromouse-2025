#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    int head,tail,size,counter;
    int*items;
}queue;
queue *initialise(int size)
{
   queue*q=malloc(sizeof(queue));
   q->head=0;
   q->tail=0;
   q->size=size;
   q->counter=0;
   q->items=malloc(size*sizeof(int));
   return(q);
}
int isfull(queue*);
int isempty(queue*);
void enqueue(queue*q,int value)
{
    if(!isfull(q))
    {
        q->items[q->tail]=value;
        q->tail=(q->tail+1)%q->size;
        q->counter++;
    }
}
int dequeue(queue*q)
{
    if(!isempty(q))
    {
        int result;
        result=q->items[q->head];
        q->head=(q->head+1)%q->size;
        q->counter--;
        return result;
    }
}
int isfull(queue*q)
{
    if(q->counter==q->size)
        return(1);
    else
        return(0);
}
int isempty(queue*q)
{
    if(q->counter==0)
        return(1);
    else
        return(0);
}

void insert(queue*q,int value,int index)
{
    if(isfull(q))
        q->size++;
    int size=q->counter+1;
    int copy[size];
    for(int i=0;i<size;i++)
    {
        if(i!=index)
            copy[i]=dequeue(q);
        else
            copy[i]=value;
    }
    for(int j=0;j<size;j++)
        enqueue(q,copy[j]);

}

