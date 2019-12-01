#include <VariableTimedAction.h>

#ifndef CarContrpol_h
#define CarContrpol_h


enum CAR_EVENT {
  STOP,
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT
};

class CarState {
  protected:
    CarMovement *carMovement;
  public:
    CarState(CarMovement *carMovement){
      this->carMovement = carMovement;
    }
    virtual CarState* transition(CAR_EVENT event);
    virtual long execute();
};

class StoppedState : public CarState {
  public:
     StoppedState(CarMovement *carMovement);
    CarState* transition(CAR_EVENT event);
    long execute();
};

class MovingForwardState : public CarState {
  
  private:
    static const int SPEED_ACCEL = 20;
    CarState* accelerate(int increment);
    CarState* slowDown(int increment);
  
  public:
    MovingForwardState(CarMovement *carMovement);
    CarState* transition(CAR_EVENT event);
    long execute();
};

class MovingBackwardState : public CarState {
  
  private:
    static const int SPEED_ACCEL = 20;
    CarState* accelerate(int increment);
    CarState* slowDown(int increment);
  
  public:
    MovingBackwardState(CarMovement *carMovement);
    CarState* transition(CAR_EVENT event);
    long execute();
};

// Stopped -----------------------------------

StoppedState :: StoppedState(CarMovement *carMovement) : CarState(carMovement) {
  this->carMovement->stop();
};

CarState* StoppedState::transition(CAR_EVENT event) {
  switch(event){
    case FORWARD:
      return new MovingForwardState(this->carMovement);
    case BACKWARD:
      return new MovingBackwardState(this->carMovement);
  }
  return this;
};

long StoppedState::execute() {
  return 0;
}

// Moving Forward -----------------------------------

MovingForwardState :: MovingForwardState(CarMovement *carMovement) : CarState(carMovement) {
  this->accelerate(MovingForwardState::SPEED_ACCEL);
};

CarState* MovingForwardState::accelerate(int increment){
  this->carMovement->moveForward(this->carMovement->getCurrentSpeed() + increment);
  return this;
};

CarState* MovingForwardState::slowDown(int increment){
  if (this->carMovement->getCurrentSpeed() == 0){
    return new MovingBackwardState(this->carMovement);
  }
  this->carMovement->moveForward(this->carMovement->getCurrentSpeed() - increment);
};

CarState* MovingForwardState::transition(CAR_EVENT event) {
  switch(event){
    case STOP:
      return new StoppedState(this->carMovement);
    case FORWARD:
      return this->accelerate(MovingForwardState::SPEED_ACCEL);
    case BACKWARD:
      return this->slowDown(MovingForwardState::SPEED_ACCEL);
  }
  return this;
};

long MovingForwardState::execute() {
  return 0;
};

// Moving Backward -----------------------------------

MovingBackwardState :: MovingBackwardState(CarMovement *carMovement) : CarState(carMovement) {
  this->accelerate(MovingBackwardState::SPEED_ACCEL);
};

CarState* MovingBackwardState::accelerate(int increment){
  this->carMovement->moveBackward(this->carMovement->getCurrentSpeed() + increment);
  return this;
};

CarState* MovingBackwardState::slowDown(int increment){
  if (this->carMovement->getCurrentSpeed() == 0){
    return new MovingForwardState(this->carMovement);
  }
  this->carMovement->moveBackward(this->carMovement->getCurrentSpeed() - increment);
};

CarState* MovingBackwardState::transition(CAR_EVENT event) {
  switch(event){
    case STOP:
      return new StoppedState(this->carMovement);
    case FORWARD:
      return this->accelerate(MovingBackwardState::SPEED_ACCEL);
    case BACKWARD:
      return this->slowDown(MovingBackwardState::SPEED_ACCEL);
  }
  return this;
};

long MovingBackwardState::execute() {
  return 0;
};


//    enum STATE {   
//        STOPPED, 
//        MOVING_FORWARD,
//        MOVING_FORWARD_RIGHT,
//        MOVING_FORWARD_LEFT,
//        MOVING_BACKWARD,
//        MOVING_BACKWARD_LEFT,
//        MOVING_BACKWARD_RIGHT,
//        TURNING_RIGHT,
//        TURNING_LEFT
//    };


class CarControl : public VariableTimedAction{

 private:
    CarMovement *carMovement;
    CarState *currentState;
    long timeWait = 0;

    unsigned long run() {
      return this->currentState->execute();
    }

  public:
    CarControl(CarMovement *carMovement);
    void transition(CAR_EVENT event);
};

CarControl::CarControl(CarMovement *carMovement){
  this->carMovement = carMovement;
  this->currentState = new StoppedState(carMovement);
};

void CarControl::transition(CAR_EVENT event){
  CarState *oldState = this->currentState;
  this->currentState = this->currentState->transition(event);
  if (oldState != this->currentState){
    delete oldState;
  }
};

#endif
