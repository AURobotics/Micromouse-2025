#include <iostream>
#include "API.h" // micromouse simulator api

#define MAX_H 18
#define MAX_W 18

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
    return ! maze [curr_r][curr_c][dir];
}

void flood () {
    
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
        curr_dir += 3; // b3mel +3 msh -1 3shan el negative numbers don't work/work differently fel mod 
    } 
    else if(dir - curr_dir == 1 ) // turn right 
    {   
        API::turnRight();
        API::moveForward();
        curr_dir++;
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
        curr_dir += 2;
    }

    curr_dir %= 4;
    curr_c = c;
    curr_r = r;

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