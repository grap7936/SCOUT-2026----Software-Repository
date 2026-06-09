#ifndef TARGET_HPP
#define TARGET_HPP
#include <opencv4/opencv2/opencv.hpp>
class Graph ; 

class Target {


public:
int x, y, size, id, nx, ny; 
float vx, vy, kx, ky;
Target* next_instance;
Target* prev_instance;
Graph* proximity;
cv::KalmanFilter* kf;
    Target(int x, int y, int size, int id,int nx, int ny, float vx, float vy, float kx, float ky, Target*next_instance, Target*prev_instance, Graph*proximity);
        

    int getID();

    void setID(int id) ;

    int getX() ;

    void setX(int x) ;

    int getY() ;
    void setY(int y) ;
    

    int getSize() ;

    void setSize(int size) ;

    int getNx();
    
    void setNx(int nx);
    int getNy();
    void setNy(int ny) ;
    float getVx() ;
    void setVx(float vx) ;
    float getVy() ;
    void setVy(float vy) ;

    float getKx() ;
    void setKx(float kx) ;
    float getKy() ;
    void setKy(float ky) ;
    Target* get_next_instancePtr() ;

    void set_next_instancePtr(Target* next_instance) ;

    Target* get_prev_instancePtr() ;
    void set_prev_instancePtr(Target* prev_instance) ;

    Graph* get_proximity() ;
    void set_proximity(Graph* proximity) ;
    
    KalmanFilter* get_kf() ;
    void set_kf(KalmanFilter* kf) ;
};
        















#endif