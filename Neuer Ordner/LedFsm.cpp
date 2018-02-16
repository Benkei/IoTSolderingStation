// LedFsm.cpp has been generated automatically by StateBuilderCpp 3.0
// Executable: 
// file:/C:/StateForge/StateBuilderCpp/bin/StateBuilderCpp.exe
// Command line options: 
//  F:\Projects\SolderingStation\Neuer Ordner\Led.fsmcpp
// Date: 08.02.2018 01:17:56

// Includes
#include "LedFsm.h"

#include <fsm/Fsm.hpp>
#include <LedPrivate.h>

// Disable some MS compiler warnings
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4355) // 'this' : used in base member initialiser list
#pragma warning(disable:4189) // local variable is initialised but not referenced
#pragma warning(disable:4100) // unreferenced formal parameter
#endif

// Disable some GCC compiler warnings
#if ((__GNUC__ * 100) + __GNUC_MINOR__) >= 402
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// State names and ids
const fsm::StateNameToId kStateNameToIdLedContext[] = 
{
  {"Led", 0},
  {"SwitchedOff", 1},
  {"SwitchedOn", 2},
  {NULL, 0}
};

// Transition names and ids
const fsm::TransitionNameToId kTransitionNameToIdLedContext[] = 
{
  {"On", 0},
  {"Off", 1},
  {NULL, 0}
};

// State Instantiation
static const LedLedState kLedLedState(kStateNameToIdLedContext[0].pcName, 0);
static const LedSwitchedOffState kLedSwitchedOffState(kStateNameToIdLedContext[1].pcName, 1);
static const LedSwitchedOnState kLedSwitchedOnState(kStateNameToIdLedContext[2].pcName, 2);

/**************************************************************
  * Contexts implementations  
  *************************************************************/
/**************************************************************
  * Context LedContext
  *************************************************************/
LedContext::LedContext(LedPrivate& ledPrivate) : 
  fsm::Context<LedLedState, LedContext>()
  , m_ledPrivate(ledPrivate)
{
  SetName("LedContext");
  SetInitialState(LedSwitchedOffState::GetInstance());
}

LedContext::~LedContext(){}


const fsm::StateNameToId* LedContext::GetStateNameToId()
{
  return kStateNameToIdLedContext;
}

void LedContext::EnterInitialState()
{
  fsm::WalkChainEntry<LedContext, LedLedState>(*this, &LedLedState::GetInstance() , &GetState());
}

void LedContext::LeaveCurrentState()
{
  fsm::WalkChainExit<LedContext, LedLedState>(*this, &GetState(), &LedLedState::GetInstance());
}

// SwitchEvent synchronous events 
void LedContext::On()
{
  GetState().On(*this);
}

void LedContext::Off()
{
  GetState().Off(*this);
}

// Timer start implementation

/**************************************************************
  * States implementations  
 **************************************************************/
/**************************************************************
 * State implementation for LedLedState
 **************************************************************/

// State Constructor
LedLedState::LedLedState(const char* pName, int id) : fsm::State<LedContext, LedLedState>(pName, id)
{
}

// State Destructor
LedLedState::~LedLedState()
{
}

// State GetInstance
const LedLedState& LedLedState::GetInstance()
{
  return kLedLedState;
}

// OnEntry
void LedLedState::OnEntry(LedContext &context) const
{
  // OnEntry for state LedLedState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnEntry(context.GetName(), LedLedState::GetInstance().GetName());
  }
}

// OnExit
void LedLedState::OnExit(LedContext &context) const
{
  // OnExit for state LedLedState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnExit(context.GetName(), LedLedState::GetInstance().GetName());
  }
}

// Events implementation for state  LedLedState
void LedLedState::On(LedContext &context) const
{
  // 0 transition(s)
}

void LedLedState::Off(LedContext &context) const
{
  // 0 transition(s)
}


/**************************************************************
 * State implementation for LedSwitchedOffState
 **************************************************************/

// State Constructor
LedSwitchedOffState::LedSwitchedOffState(const char* pName, int id) : LedLedState(pName, id)
{
  m_pStateParent = &LedLedState::GetInstance();
}

// State Destructor
LedSwitchedOffState::~LedSwitchedOffState()
{
}

// State GetInstance
const LedSwitchedOffState& LedSwitchedOffState::GetInstance()
{
  return kLedSwitchedOffState;
}

// OnEntry
void LedSwitchedOffState::OnEntry(LedContext &context) const
{
  // OnEntry for state LedSwitchedOffState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnEntry(context.GetName(), LedSwitchedOffState::GetInstance().GetName());
  }
  LedPrivate& ledPrivate = context.getLedPrivate();
  // 1 action(s) to do
  ledPrivate.DoOff();
}

// OnExit
void LedSwitchedOffState::OnExit(LedContext &context) const
{
  // OnExit for state LedSwitchedOffState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnExit(context.GetName(), LedSwitchedOffState::GetInstance().GetName());
  }
}

// Events implementation for state  LedSwitchedOffState
void LedSwitchedOffState::On(LedContext &context) const
{
  // 1 transition(s)
  LedPrivate& ledPrivate = context.getLedPrivate();
  context.SetTransitionName("On");
  fsm::ProcessTransitionPre<LedContext, LedLedState>(context, &LedSwitchedOnState::GetInstance());
  // No action to do
  fsm::ProcessTransitionPost<LedContext, LedLedState>(context, &LedSwitchedOnState::GetInstance());
  return;
}



/**************************************************************
 * State implementation for LedSwitchedOnState
 **************************************************************/

// State Constructor
LedSwitchedOnState::LedSwitchedOnState(const char* pName, int id) : LedLedState(pName, id)
{
  m_pStateParent = &LedLedState::GetInstance();
}

// State Destructor
LedSwitchedOnState::~LedSwitchedOnState()
{
}

// State GetInstance
const LedSwitchedOnState& LedSwitchedOnState::GetInstance()
{
  return kLedSwitchedOnState;
}

// OnEntry
void LedSwitchedOnState::OnEntry(LedContext &context) const
{
  // OnEntry for state LedSwitchedOnState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnEntry(context.GetName(), LedSwitchedOnState::GetInstance().GetName());
  }
  LedPrivate& ledPrivate = context.getLedPrivate();
  // 1 action(s) to do
  ledPrivate.DoOn();
}

// OnExit
void LedSwitchedOnState::OnExit(LedContext &context) const
{
  // OnExit for state LedSwitchedOnState
  fsm::IObserver *pObserver = context.GetObserver();
  if(pObserver){
    pObserver->OnExit(context.GetName(), LedSwitchedOnState::GetInstance().GetName());
  }
}

// Events implementation for state  LedSwitchedOnState
void LedSwitchedOnState::Off(LedContext &context) const
{
  // 1 transition(s)
  LedPrivate& ledPrivate = context.getLedPrivate();
  context.SetTransitionName("Off");
  fsm::ProcessTransitionPre<LedContext, LedLedState>(context, &LedSwitchedOffState::GetInstance());
  // No action to do
  fsm::ProcessTransitionPost<LedContext, LedLedState>(context, &LedSwitchedOffState::GetInstance());
  return;
}




// Reenable some compiler warnings
#ifdef _MSC_VER
#pragma warning(pop)
#endif

