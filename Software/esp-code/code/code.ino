#define MAX_H 18
#define MAX_W 18
#define QUEUE_MAX (MAX_H * MAX_W)
using queue = struct queue;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void log(const String& text) {
  Serial.println(text);
}

String tostr(short x)
{
    //log("tostr");
    if(x == 0)return "0";
    String ret = "";
    while(x)
    {
        ret = char((x%10)+'0') + ret;
        x/=10;
    }
    return ret;
}

// queue implementation
struct queue
{
    short head,tail,size,counter;
    char items[MAX_H*MAX_W];
};
void initialise(queue &q,short size)
{
   q.head=0;
   q.tail=0;
   q.size=size;
   q.counter=0;
}
bool isfull(queue &q)
{
    if(q.counter==q.size)
        return(1);
    else
        return(0);
}
bool isempty(queue &q)
{
    if(q.counter==0)
        return(1);
    else
        return(0);
}

void enqueue(queue &q, char value)
{
    if(!isfull(q))
    {
        q.items[q.tail]=value;
        q.tail=(q.tail+1)%q.size;
        q.counter++;
    }
}
char dequeue(queue &q)
{
    if(!isempty(q))
    {
        char result;
        result=q.items[q.head];
        q.head=(q.head+1)%q.size;
        q.counter--;
        return result;
    }
}

//end of queue implementation