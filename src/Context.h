#include <Arduino.h>
#include <iostream>

using namespace std;

class Context;
class State;

class State
{
protected:
  Context *context;

public:
  virtual ~State() {}
  void set_context(Context *context)
  {
    this->context = context;
  }

  virtual unsigned int getStateID() = 0;
};

class State0 : public State
{
public:
  unsigned int getStateID() override
  {
    return 0;
  }
};

class State1 : public State
{
public:
  unsigned int getStateID() override
  {
    return 1;
  }
};

class State2 : public State
{
public:
  unsigned int getStateID() override
  {
    return 2;
  }
};

class State3 : public State
{
public:
  unsigned int getStateID() override
  {
    return 3;
  }
};

class Context
{
private:
  State *state;
  unsigned int stride;
  unsigned int ledQty;
  unsigned int lastSentState;

public:
  Context(State *state, unsigned int stride = 6, unsigned int ledQty = 3)
      : state(nullptr), stride(stride), ledQty(ledQty), lastSentState(ledQty + 1)
  {
    this->transitionTo(state);
  }

  ~Context()
  {
    delete state;
  }

  void transitionTo(State *state)
  {
    Serial.print("Context: Transition to state ");
    Serial.println(state->getStateID());
    if (this->state != nullptr)
      delete this->state;
    this->state = state;
    this->state->set_context(this);
  }

  unsigned int getStateID()
  {
    return this->state->getStateID();
  }

  void setStride(unsigned int newStride)
  {
    stride = newStride;
  }

  void setLedQty(unsigned int newLedQty)
  {
    ledQty = newLedQty;
  }

  unsigned int determineState(float distance)
  {
    unsigned int state = distance / stride;
    return (state <= ledQty) ? state : ledQty;
  }

  bool stateChanged(unsigned int &actualState)
  {
    return actualState != lastSentState;
  }

  void changeContext(unsigned int actualState)
  {
    lastSentState = actualState;

    if (actualState == 0)
    {
      transitionTo(new State0());
    }
    else if (actualState == 1)
    {
      transitionTo(new State1());
    }
    else if (actualState == 2)
    {
      transitionTo(new State2());
    }
    else
    {
      transitionTo(new State3());
    }
  }
};