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

char curr_dir = 0; // 0--> North, 1 --> East, 2 --> South, 3 --> West
char curr_r = 16, curr_c = 1;

int current_run;
int previous_run;

bool maze [MAX_H][MAX_W][5] = {0}; // represents the maze, first 4 bits represent the walls N E S W, the last bit represents the visiting status
//leh mn3melsh byte/char maze[MAX_H][MAX_W] ?

short dis [MAX_H][MAX_W] = {
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16}
};

void update_mms_maze()
{
    for(int i=1;i<=16;i++)
    {
        for(int j=1;j<=16;j++)
        {
            setText(j-1,16-i,tostr(dis[i][j]));
        }
    }
}
void set_wall()
{
    Serial.println("\ncoloring walls" + String(curr_c - 1) + " " + String(16 - curr_r));
    for(int i=0;i<4;i++) std::cerr<<maze[curr_r][curr_c][i]<<" ";
    std::cerr<<std::endl;
    if(maze[curr_r][curr_c][0])setWall(curr_c-1,16-curr_r,'n');
    if(maze[curr_r][curr_c][1])setWall(curr_c-1,16-curr_r,'e');
    if(maze[curr_r][curr_c][2])setWall(curr_c-1,16-curr_r,'s');
    if(maze[curr_r][curr_c][3])setWall(curr_c-1,16-curr_r,'w');
}
