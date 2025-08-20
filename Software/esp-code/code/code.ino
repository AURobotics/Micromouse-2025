#define MAX_H 18
#define MAX_W 18
#define QUEUE_MAX (MAX_H * MAX_W)


using queue = struct queue;

// void log(const String& text) {
//   Serial.println(text);
// }

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

queue r_q;
queue c_q;

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
    // Serial.println("\ncoloring walls" + String(curr_c - 1) + " " + String(16 - curr_r));
    // for(int i=0;i<4;i++) Serial.println(maze[curr_r][curr_c][i]);
    if(maze[curr_r][curr_c][0])setWall(curr_c-1,16-curr_r,'n');
    if(maze[curr_r][curr_c][1])setWall(curr_c-1,16-curr_r,'e');
    if(maze[curr_r][curr_c][2])setWall(curr_c-1,16-curr_r,'s');
    if(maze[curr_r][curr_c][3])setWall(curr_c-1,16-curr_r,'w');
}

// change r, c to move to: N, E, S, W
signed char r_mov [4] =   {-1, 0,1, 0};
signed char c_mov [4] =   {0, 1, 0,-1};

bool isValid (char r, char c) {
    return ((r >= 0) && (r < MAX_H)) && ((c >= 0) && (c < MAX_W));
}

bool isAccessible (char r, char c,char dir) {
    return ! (maze [r][c][dir] || maze[r+r_mov[dir]][c + c_mov[dir]][(dir+2)%4]);
}


void flood (bool goal = 1) { // make goal = 0 to change the goal to the start

    for(char i=0;i<MAX_W;i++){ //initialize all cells with -1
        for(char j=0;j<MAX_H;j++){
            dis[i][j]=-1;
        }
    }

    if(goal)
    {
        for(char x=MAX_W/2 - 1;x<MAX_W/2+1;x++){ //change middle cells with 0
            for(char w=MAX_H/2 - 1;w<MAX_H/2+1;w++){
            dis[x][w]=0; 
            enqueue(r_q,x);
            enqueue(c_q,w);
            }
        }
    }
    else 
    {
        dis[16][1] = 0;
        enqueue(r_q,16);
        enqueue(c_q,1);
    }

    while (! isempty (c_q) && ! isempty (r_q)) {
        char r=dequeue (r_q);
        char c=dequeue (c_q);
        for (int i = 0; i < 4; i ++) {
            if (isValid(r + r_mov[i],c + c_mov[i])&&isAccessible(r,c,i)&&dis [r + r_mov [i]][c + c_mov [i]]==-1) {
                dis [r + r_mov [i]][c + c_mov [i]] = dis [r][c] + 1;
                enqueue (r_q,r + r_mov [i]);
                enqueue (c_q,c + c_mov [i]);
            }
        }
    }
    update_mms_maze();
}

void moveTo (char r, char c) {
    // get where I want to move relative to abolute direction (y3ny lw el robot bases north) ana lesa m2alef el term dah
    short dir;
    if(r < curr_r) dir = 0;
    if(r > curr_r) dir = 2;
    if(c < curr_c) dir = 3;
    if(c > curr_c) dir = 1;    
    // Serial.println(String((int) r)+" "+ String((int) c));
    log(String("direction: ") + tostr(dir));

    // compare the movement direction to the current directoin to know how should I turn 

    if(dir - curr_dir == -1 || dir-curr_dir == 3)// turn left
    {
        turnLeft();
        moveForward();
        curr_dir += 3; // b3mel +3 msh -1 because el negative numbers don't work/work differently fel mod// 
    } 

    else if(dir - curr_dir == 1 ||dir - curr_dir == -3) // turn right 
    {   
        turnRight();
        moveForward();
        curr_dir++;
    }
    else if(dir == curr_dir) // move forward 
    {
        moveForward();
    }
    else // turn 180
    {
        turnRight();
        turnRight();
        moveForward();
        curr_dir += 2;
    }

    curr_dir %= 4;
    curr_c = c;
    curr_r = r;

}

void exploreToCenter () {
    while (! (((curr_r == MAX_H / 2 - 1) || (curr_r == MAX_H / 2)) && ((curr_c == MAX_W / 2 - 1) || (curr_c == MAX_W / 2)))) {
        maze [curr_r][curr_c][4] = 1;
        log("start: " + tostr(dis[curr_r][curr_c]));
        log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
        bool walls [4];
        walls [0] = wallFront();
        walls [1] = wallRight();
        //walls [2] = wallBack(); wall back isn't working
        walls [3] = wallLeft();
        //if(curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
        //walls[2] = 0;
        //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
        //std::cerr<<std::endl;

        char d = curr_dir, w = 0;
        do {
            maze [curr_r][curr_c][d] = walls [w];
            d = (d + 1) % 4;
            w ++;
        } while (d != curr_dir);
        
        set_wall();
        char next_r = curr_r, next_c = curr_c;

        for (char i = 0; i < 4; i ++) {
            if (isValid (curr_r + r_mov [i], curr_c + c_mov [i]) && isAccessible(curr_r,curr_c,i) && dis [curr_r + r_mov [i]][curr_c + c_mov [i]] < dis [next_r][next_c]) {
                next_r = curr_r + r_mov [i];
                next_c = curr_c + c_mov [i];
            }
        }
        log("to: " + tostr(dis[next_r][next_c]));

        if ((next_r == curr_r) && (next_c == curr_c))
            flood ();
        else 
            moveTo (next_r, next_c);
    }
}

void exploreToStart()
{
    while (!(curr_c == 1 && curr_r == 16)) {
        maze [curr_r][curr_c][4] = 1;
        log("start: " + tostr(dis[curr_r][curr_c]));
        log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
        bool walls [4];
        walls [0] = wallFront();
        walls [1] = wallRight();
        //walls [2] = wallBack(); wall back isn't working
        walls [3] = wallLeft();
        if(curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
        else walls[2] = 0;
        //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
        //std::cerr<<std::endl;

        

        char d = curr_dir, w = 0;
        do {
            maze [curr_r][curr_c][d] = walls [w];
            d = (d + 1) % 4;
            w ++;
        } while (d != curr_dir);
        
        set_wall();
        char next_r = curr_r, next_c = curr_c;

        for (char i = 0; i < 4; i ++) {
            if (isValid (curr_r + r_mov [i], curr_c + c_mov [i]) && isAccessible(curr_r,curr_c,i) && dis [curr_r + r_mov [i]][curr_c + c_mov [i]] < dis [next_r][next_c]) {
                next_r = curr_r + r_mov [i];
                next_c = curr_c + c_mov [i];
            }
        }
        log("to: " + tostr(dis[next_r][next_c]));

        if ((next_r == curr_r) && (next_c == curr_c))
            flood (0);
        else 
            moveTo (next_r, next_c);
    }
}

void setup() {
  Serial.begin(19200);
  initialise(c_q,MAX_H * MAX_W); //queue initialisation for storing row and coloumn
  initialise(r_q,MAX_H* MAX_W);
  update_mms_maze();
  log("Khalast setup");
//   Serial.println("Khalast setup");


}

void loop() {
    log("Dakhalt el loop");
    while(1) {
    flood();
    previous_run = current_run;
    exploreToCenter ();
    current_run = dis[16][1];
    if(current_run != 0 && current_run == previous_run) break;
    flood(0);
    exploreToStart ();
    log("done!!!! The best run is "+ current_run);
    }
}




