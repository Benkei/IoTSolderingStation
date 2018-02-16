// LedFsm.h has been generated automatically by StateBuilderCpp 3.0
// Executable: 
// file:/C:/StateForge/StateBuilderCpp/bin/StateBuilderCpp.exe
// Command line options: 
//  F:\Projects\SolderingStation\Neuer Ordner\Led.fsmcpp
// Date: 08.02.2018 01:17:56

#ifndef LEDFSM_H
#define LEDFSM_H


// Includes
#include <cstring>
#include <fsm/Fsm.h>

// Object classes forward declaration

// Context classes forward declaration
class LedContext;
// Parallel classes forward declaration
// State classes forward declaration
class LedSwitchedOnState;
class LedLedState;
class LedSwitchedOffState;

// Object classes forward declaration within the custom namepace
class LedPrivate;


// Context declaration for state Led
class LedContext : public fsm::Context<LedLedState, LedContext> {
public:
  // Constructor
  LedContext(LedPrivate& ledPrivate);

  // Destructor
  virtual ~LedContext();


  static const fsm::StateNameToId* GetStateNameToId();
  // SwitchEvent's events 
  void On();
  void Off();


  // Enter the initial state: walk the onEntry chain from the top state to the initial state.
  void EnterInitialState();

  // Leave the current state: walk the onExit chain from the current state to the top state.
  void LeaveCurrentState();

  LedPrivate& getLedPrivate(){return m_ledPrivate;}

private:
  LedPrivate& m_ledPrivate;


};


// State LedLedState
class LedLedState : public fsm::State<LedContext, LedLedState> {
public:
  // Constructor
  LedLedState(const char* pName, int id);

  // Singleton pattern
  static const LedLedState& GetInstance();

  // Destructor
  virtual ~LedLedState();

  // OnEntry and OnExit
  virtual void OnEntry(LedContext& context) const;
  virtual void OnExit(LedContext& context) const;

  // Events
  virtual void On(LedContext &context) const ;
  virtual void Off(LedContext &context) const ;

};


// State LedSwitchedOffState
class LedSwitchedOffState : public LedLedState {
public:
  // Constructor
  LedSwitchedOffState(const char* pName, int id);

  // Singleton pattern
  static const LedSwitchedOffState& GetInstance();

  // Destructor
  virtual ~LedSwitchedOffState();

  // OnEntry and OnExit
  virtual void OnEntry(LedContext& context) const;
  virtual void OnExit(LedContext& context) const;

  // Events
  virtual void On(LedContext &context) const ;

};


// State LedSwitchedOnState
class LedSwitchedOnState : public LedLedState {
public:
  // Constructor
  LedSwitchedOnState(const char* pName, int id);

  // Singleton pattern
  static const LedSwitchedOnState& GetInstance();

  // Destructor
  virtual ~LedSwitchedOnState();

  // OnEntry and OnExit
  virtual void OnEntry(LedContext& context) const;
  virtual void OnExit(LedContext& context) const;

  // Events
  virtual void Off(LedContext &context) const ;

};

#endif //LEDFSM_H
