/////////////////////////////////////////////////////////////
/*
Code Summary:
The Target class is the core data record for a single detected object in one frame.
Every contour the Detector finds becomes a Target, and the Selector/Sentry pipeline
threads these per-frame records together over time to form tracks. A Target stores:
  - identity/measurement:   id, size, raw detected position (x, y)
  - Kalman state:           predicted position (nx, ny), corrected position (kx, ky),
                            and smoothed velocity (vx, vy)
  - bookkeeping:            debris_likelihood score, frame_num it was detected on
  - track links:            prev_instance / next_instance (the same physical object in
                            the previous / next frame) and proximity (the cost graph
                            built from this target to every candidate in the next frame)

All members are private; the rest of the pipeline reads and writes them exclusively
through the simple accessors below.

Note: the Kalman filter itself no longer lives on the Target (the old shared_ptr<kf>
member was removed); filters are now stored in an external array keyed by Target id
inside the Selector class.
*/
/////////////////////////////////////////////////////////////

#include "Target.hpp"

/* Constructor( int x, int y, int size )
 * description:
 *  Builds a fresh target from a raw detection. Position and size come from the
 *  detected contour; every tracking/Kalman field is initialized to a neutral
 *  "not yet known" sentinel (-1 for positions that have not been estimated,
 *  0 for velocities and scores) and all track-link pointers start null.
 * inputs:
 *  int x, int y - centroid pixel coordinates of the detected contour.
 *  int size     - area (pixel count) of the detected contour.
 * returns:
 *  none - constructs the object in place.
 */
Target::Target(float x, float y, int size) {
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
    this->debris_likelihood = 0;
    this->frame_num = 0;
    this->next_instance = nullptr;
    this->prev_instance = nullptr;
    this->proximity = nullptr;
}

// --- Identity & measurement ------------------------------------------------

// Returns the unique track ID assigned to this target (-1 until initTarget runs)
int Target::getID() {
    return this->id;
}

// Sets the unique track ID for this target
void Target::setID(int id) {
    this->id = id;
}

// Returns the raw detected x (centroid) pixel coordinate
float Target::getX() {
    return this->x;
}

// Sets the raw detected x (centroid) pixel coordinate
void Target::setX(int x) {
    this->x = x;
}

// Returns the raw detected y (centroid) pixel coordinate
float Target::getY() {
    return this->y;
}

// Sets the raw detected y (centroid) pixel coordinate
void Target::setY(int y) {
    this->y = y;
}

// Returns the contour area (pixel count) of this target
int Target::getSize() {
    return this->size;
}

// Sets the contour area (pixel count) of this target
void Target::setSize(int size) {
    this->size = size;
}

// --- Kalman predicted position (nx, ny) ------------------------------------

// Returns the Kalman-predicted next x coordinate
float Target::getNx() {
    return this->nx;
}

// Sets the Kalman-predicted next x coordinate
void Target::setNx(int nx) {
    this->nx = nx;
}

// Returns the Kalman-predicted next y coordinate
float Target::getNy() {
    return this->ny;
}

// Sets the Kalman-predicted next y coordinate
void Target::setNy(int ny) {
    this->ny = ny;
}

// --- Kalman smoothed velocity (vx, vy) -------------------------------------

// Returns the Kalman-smoothed x velocity (pixels/frame)
float Target::getVx() {
    return this->vx;
}

// Sets the Kalman-smoothed x velocity (pixels/frame)
void Target::setVx(float vx) {
    this->vx = vx;
}

// Returns the Kalman-smoothed y velocity (pixels/frame)
float Target::getVy() {
    return this->vy;
}

// Sets the Kalman-smoothed y velocity (pixels/frame)
void Target::setVy(float vy) {
    this->vy = vy;
}

// --- Kalman corrected position (kx, ky) ------------------------------------

// Returns the Kalman-corrected x coordinate (smoothed best estimate)
float Target::getKx() {
    return this->kx;
}

// Sets the Kalman-corrected x coordinate
void Target::setKx(float kx) {
    this->kx = kx;
}

// Returns the Kalman-corrected y coordinate (smoothed best estimate)
float Target::getKy() {
    return this->ky;
}

// Sets the Kalman-corrected y coordinate
void Target::setKy(float ky) {
    this->ky = ky;
}

// --- Bookkeeping -----------------------------------------------------------

// Returns the frame number this target was detected on
int Target::getFrameNum() {
    return this->frame_num;
}

// Sets the frame number this target was detected on
void Target::setFrameNum(int frame_num) {
    this->frame_num = frame_num;
}

// Returns the accumulated debris-likelihood score for this track
int Target::getDebrisLikelihood() {
    return debris_likelihood;
}

// Sets the accumulated debris-likelihood score for this track
void Target::setDebrisLikelihood(int count) {
    this->debris_likelihood = count;
}

// --- Track links -----------------------------------------------------------

// Returns the pointer to this object's instance in the next frame (null if none)
Target* Target::getNextInstancePtr() {
    return this->next_instance;
}

// Sets the pointer to this object's instance in the next frame
void Target::setNextInstancePtr(Target* next_instance) {
    this->next_instance = next_instance;
}

// Returns the pointer to this object's instance in the previous frame (null if none)
Target* Target::getPrevInstancePtr() {
    return this->prev_instance;
}

// Sets the pointer to this object's instance in the previous frame
void Target::setPrevInstancePtr(Target* prev_instance) {
    this->prev_instance = prev_instance;
}

// Returns the proximity (cost) graph built from this target to next-frame candidates
Graph* Target::getProximity() {
    return this->proximity;
}

// Sets the proximity (cost) graph for this target
void Target::setProximity(Graph* proximity) {
    this->proximity = proximity;
}
