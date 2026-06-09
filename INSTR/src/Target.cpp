#include "Target.hpp"


Target::Target(int x, int y, int size) {
    this->x = x;
    this->y = y;
    this->size = size;
    this->id = -1;
    this->nx = -1;
    this->ny = -1;
    this->vx = 0;
    this->vy = 0;
    this->kx = -1;
    this->ky = -1;
    this->next_instance = NULL;
    this->prev_instance = NULL;
    this->proximity = NULL;
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

Target* Target::getNextInstancePtr() {
    return this->next_instance;
}

void Target::setNextInstancePtr(Target* next_instance) {
    this->next_instance = next_instance;
}

Target* Target::getPrevInstancePtr() {
    return this->prev_instance;
}

void Target::setPrevInstancePtr(Target* prev_instance) {
    this->prev_instance = prev_instance;
}

Graph* Target::getProximity() {
    return this->proximity;
}

void Target::setProximity(Graph* proximity) {
    this->proximity = proximity;
}

cv::KalmanFilter* Target::getKf() {
    return this->kf;
}

void Target::setKf(cv::KalmanFilter* kf) {
    this->kf = kf;
}

    