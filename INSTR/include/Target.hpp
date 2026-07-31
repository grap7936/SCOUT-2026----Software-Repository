#ifndef TARGET_HPP
#define TARGET_HPP
#include <opencv2/opencv.hpp>
class Graph ; 

class Target {
private: // moved from public
    int id;
    int size;
    float x, y;
    float nx, ny; 
    float vx, vy;
    float kx, ky;
    int debris_likelihood;
    int frame_num;
    Target* next_instance;
    Target* prev_instance;
    Graph* proximity;
    // std::shared_ptr<cv::KalmanFilter> kf; move to external array

public:

    Target(float x, float y, int size);
        
    int getID();

    void setID(int id);

    float getX();

    void setX(int x);

    float getY();

    void setY(int y);

    int getSize();

    void setSize(int size);

    float getNx();
    
    void setNx(int nx);

    float getNy();

    void setNy(int ny);

    float getVx();

    void setVx(float vx);

    float getVy();

    void setVy(float vy);

    float getKx();

    void setKx(float kx);

    float getKy();

    void setKy(float ky);

    int getFrameNum();

    void setFrameNum(int frame_num);

    int getDebrisLikelihood();

    void setDebrisLikelihood(int);

    Target* getNextInstancePtr();

    void setNextInstancePtr(Target* next_instance);

    Target* getPrevInstancePtr();

    void setPrevInstancePtr(Target* prev_instance);

    Graph* getProximity();

    void setProximity(Graph* proximity);
    
    // std::shared_ptr<cv::KalmanFilter> getKf();

    // void setKf(std::shared_ptr<cv::KalmanFilter> kf) ;
};
        
#endif