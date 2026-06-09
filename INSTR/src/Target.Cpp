#include "Target.hpp"


Target::Target(int x, int y, int size, int id,int nx, int ny, float vx, float vy, float kx, float ky, Target*next_instance, Target*prev_instance, Graph*proximity) {
    this->x = x;
    this->y = y;
    this->size = size;
    this->id = id;
    this->nx = nx;
    this->ny = ny;
    this->vx = vx;
    this->vy = vy;
    this->kx = kx;
    this->ky = ky;
    this->next_instance = next_instance;
    this->prev_instance = prev_instance;
    this->proximity = proximity;
}

int Target::getID() {
    return this->id;
}

void Target::setID(int id) {
    this->id = id;
}

int Target::getX() {
    return this->x;
}

void Target::setX(int x) {
    this->x = x;
}

int Target::getY() {
    return this->y;
}

void Target::setY(int y) {
    this->y = y;
}

int Target::getSize() {
    return this->size;
}

void Target::setSize(int size) {
    this->size = size;
}
int Target::getNx() {
    return this->nx;
}
void Target::setNx(int nx) {
    this->nx = nx;
}
int Target::getNy() {
    return this->ny;
}
void Target::setNy(int ny) {
    this->ny = ny;
}    
float Target::getVx() {
    return this->vx;
}
void Target::setVx(float vx) {

    this->vx = vx;
}
float Target::getVy() {
    return this->vy;
}
void Target::setVy(float vy) {
    this->vy = vy;
}
float Target::getKx() {
    return this->kx;
}
void Target::setKx(float kx) {
    this->kx = kx;
}
float Target::getKy() { 
    return this->ky;
}
void Target::setKy(float ky) {
    this->ky = ky;
}
Target* Target::get_next_instancePtr() {
    return this->next_instance;
}
Target* Target::set_next_instancePtr(Target* next_instance) {
    this->next_instance = next_instance;
    return this->next_instance;
}
Target* Target::get_prev_instancePtr() {
    return this->prev_instance;
}
Target* Target::set_prev_instancePtr(Target* prev_instance) {
    this->prev_instance = prev_instance;
    return this->prev_instance;
}
Graph* Target::get_proximity() {
    return this->proximity;
}
Graph* Target::set_proximity(Graph* proximity) {
    this->proximity = proximity;
    
}
KalmanFilter* Target::get_kf() {
    return this->kf;
}
KalmanFilter* Target::set_kf(KalmanFilter* kf) {
    this->kf = kf;
    
}

    