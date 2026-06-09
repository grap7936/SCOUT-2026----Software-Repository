#ifndef TARGET_HPP
#define TARGET_HPP
#include <opencv4/opencv2/opencv.hpp>
class Graph ; 

class Target {


public:
    int id;
    int size;
    int x, y;
    int nx, ny; 
    float vx, vy;
    float kx, ky;
    Target* next_instance;
    Target* prev_instance;
    Graph* proximity;
    cv::KalmanFilter* kf;

    Target(int x, int y, int size);
        
    int getID();

    void setID(int id);

    int getX();

    void setX(int x);

    int getY();

    void setY(int y);

    int getSize();

    void setSize(int size);

    int getNx();
    
    void setNx(int nx);

    int getNy();

    void setNy(int ny);

    float getVx();

    void setVx(float vx);

    float getVy();

    void setVy(float vy);

    float getKx();

    void setKx(float kx);

    float getKy();

    void setKy(float ky);

    Target* getNextInstancePtr();

    void setNextInstancePtr(Target* next_instance);

    Target* getPrevInstancePtr();

    void setPrevInstancePtr(Target* prev_instance);

    Graph* getProximity();

    void setProximity(Graph* proximity);
    
    cv::KalmanFilter* getKf();

    void setKf(cv::KalmanFilter* kf) ;
};
        
#endif