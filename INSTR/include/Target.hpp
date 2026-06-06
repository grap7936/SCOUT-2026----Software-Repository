#ifndef TARGET_HPP
#define TARGET_HPP

class Graph;

class Target {

public:
    int id;
    int size;
    int x, y;
    int nx, ny;
    float is_star;
    
    Graph* proximity;
    Target* next_instance;
    Target* prev_instance;
    
    Target(int input) {
        id = input;
    }

};

#endif