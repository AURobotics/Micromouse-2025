#include <iostream>
#include <cstdlib>
// #include "API.h" // micromouse simulator api

#define MAX_H 18
#define MAX_W 18
// queue implementation
typedef struct
{
    int head,tail,size,counter;
    int*items;
}queue;
queue *initialise(int size)
{
   queue*q=(queue*)malloc(sizeof(queue));
   q->head=0;
   q->tail=0;
   q->size=size;
   q->counter=0;
   q->items=(int*)malloc(size*sizeof(int));
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
//end of queue implementation

char curr_dir = 0; // 0--> North, 1 --> East, 2 --> South, 3 --> West
char curr_r = 0, curr_c = 0;


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

// change r, c to move to: N, E, S, W
signed char r_mov [4] =   {1, 0,-1, 0};
signed char c_mov [4] =   {0, 1, 0,-1};

bool isValid (char r, char c) {
    return ((r >= 0) && (r <= MAX_H)) && ((c >= 0) && (c <= MAX_W));
}

bool isAccessible (char dir) {
    return ! maze [curr_r][curr_c];
}
queue*c_q=initialise(MAX_H); //queue initialisation for storing row and coloumn
queue*r_q=initialise(MAX_W);
void flood () {
    for(int i=0;i<MAX_W;i++){ //initialize all cells with -1
        for(int j=0;j<MAX_H;j++){
            dis[i][j]=-1;
        }
    }
    for(int x=3;x<5;x++){ //change middle cells with 0
        for(int w=3;w<5;w++){
           dis[x][w]=0; 
        }
    }
    enqueue(r_q,3);
    enqueue(c_q,3);
    while (! isempty (c_q) && ! isempty (r_q)) {
        int r=dequeue (r_q);
        int c=dequeue (c_q);
        for (int i = 0; i < 4; i ++) {
            if (! maze [r][c][i]&&dis [r + r_mov [i]][c + c_mov [i]]==-1) {
                dis [r + r_mov [i]][c + c_mov [i]] = dis [r][c] + 1;
                enqueue (r_q,r + r_mov [i]);
                enqueue (c_q,c + c_mov [i]);
            }
        }
    }
}

void moveTo (char r, char c) {
    // get where I want to move relative to abolute direction (y3ny lw el robot bases north) ana lesa m2alef el term dah
    int dir;
    if(c < curr_c) dir = 0;
    if(c > curr_c) dir = 2;
    if(r < curr_r) dir = 3;
    if(r > curr_r) dir = 1;

    // compare the movement direction to the current directoin to know how should I turn 

    if(dir - curr_dir == -1)// turn left
    {
        API::turnLeft();
        API::moveForward();
    } 
    else if(dir - curr_dir == 1 ) // turn right 
    {
        API::turnRight();
        API::moveForward();
    }
    else if(dir == curr_dir) // move forward 
    {
        API::moveForward();
    }
    else // turn 180
    {
        API::turnRight();
        API::turnRight();
        API::moveForward();
    }

}

void exploreToCenter () {
    while (! (((curr_r == MAX_H / 2 - 1) || (curr_r == MAX_H / 2)) && ((curr_c == MAX_W / 2 - 1) || (curr_c == MAX_W / 2)))) {
        maze [curr_r][curr_c][4] = 1;

        bool walls [4];
        walls [0] = API::wallFront();
        walls [1] = API::wallRight();
        walls [2] = API::wallBack();
        walls [3] = API::wallLeft();

        char d = curr_dir, w = 0;
        do {
            maze [curr_r][curr_c][d] = walls [w];
            d = (d + 1) % 4;
            w ++;
        } while (d != curr_dir);

        char next_r = curr_r, next_c = curr_c;

        for (char i = 0; i < 4; i ++) {
            if (isValid (curr_r + r_mov [i], curr_c + c_mov [i]) && isAccessible (i) && dis [curr_r + r_mov [i]][curr_c + c_mov [i]] < dis [next_r][next_c]) {
                next_r = curr_r + r_mov [i];
                next_c = curr_c + c_mov [i];
            }
        }

        if ((next_r == curr_r) && (next_c == curr_c))
            flood ();
        else 
            moveTo (next_r, next_c);
    }
}

main () {
    exploreToCenter ();
}