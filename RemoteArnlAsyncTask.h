#ifndef REMOTEARNLASYNCTASK_H
#define REMOTEARNLASYNCTASK_H

/*
Copyright (c) 2015 Adept MobileRobots LLC
All rights reserved. 
*/

#include "Aria.h"
#include "ArNetworking.h"


/**
  Use this to help run your own custom tasks or activities, triggered when a
  remote server with ARNL navigation reaches goals. 

  When the ARNL server mode and status indicate that it has successfully reached a goal, a new thread is created to perform your
  custom (potentially long-running) task.  Optionally, a goal's name must match either
  the given @a prefix or @a suffix.  (Default behavior is to run at each goal.)

  The ArNetworing client thread continues to execute asynchronously.

  You may call nextGoal("goal name"); to plan to another goal if desired.
  (This is just a shortcut to sending a request to the server goal mode)

  The API for this class is generally identical to ArnlASyncTask, which is used
  for setting up goal-triggered async tasks in the server program.

  @note Since ArClientBase continues to execute asynchronously, it is posnsible
  for it to begin navigating to another goal while your task is also still executing
  (for example, sent via MobileEyes.)  You can use ArPathPlanningInterface::addNewGoalCB()
  to register a callback in order to be notified if a new goal is set.

  @note Since the server continues executing asynchronously, it is possible for
  the ARNL server to reach other goals while your task is also still executing
  after reaching a previous goal.  
  One instance of this class is normally created for the
  whole program, but new threads may be created at any time (whenever ARNL happens
  to reach a goal), these threads are sharing access to the variables withhin
  this class instance, and so this access is synchronized using a mutex. Make certain you do
  not keep the mutex locked during any long running operations or operations of
  indeterminate duration, and that in all logical paths through the code, the
  mutex is eventually unlocked if locked.


*/
class RemoteArnlAsyncTask : public virtual ArASyncTask
{
public:

  /**


   */
  RemoteArnlAsyncTask(ArClientBase *netclient, ArArgumentParser *argParser = NULL,
    const std::string& goalPrefix = "", const std::string& goalSuffix = "") :
    myClient(netclient), 
    myHaveGoalNamePrefix(false), myHaveGoalNameSuffix(false)
	{
		//myPathPlanningTask->addGoalDoneCB(&myGoalDoneCB);
    // todo create a client update handler that checks for goals
    if(goalPrefix != "")
      runIfGoalNamePrefix(goalPrefix);
    if(goalSuffix != "")
      runIfGoalNameSuffix(goalSuffix);
  }

  /** Override this method in your subclass. */
  virtual void runTask() = 0;

  /** Override this method in your subclass. */
  virtual const char *getName() const = 0;

  virtual const char *getConfigSectionName() const {
    return getName();
  }

  bool addConfigParam(ArConfigArg &arg) {
    return Aria::getConfig()->addParam(arg, getConfigSectionName());
  }

  void lock() {
    myMutex.lock();
  }

  void unlock() {
    myMutex.unlock();
  }

  /// Set a prefix that a goal name must match for the task to runTask
  void runIfGoalNamePrefix(const std::string& prefix)
  {
    myHaveGoalNamePrefix = true;
    myGoalNamePrefix = prefix;
  }

  void runIfGoalNameSuffix(const std::string& suffix)
  {
    myHaveGoalNameSuffix = true;
    myGoalNameSuffix = suffix;
  }

  /// Call path planning task to set new goal
  void nextGoal(const std::string goalName)
  {
    ArLog::log(ArLog::Normal, "%s: Going to new goal: %s", getName(), goalName.c_str());
    myPathPlanningTask->pathPlanToGoal(goalName.c_str());
  }

protected:
	ArPathPlanningTask *myPathPlanningTask;
  ArFunctor1C<RemoteArnlAsyncTask, ArPose> myGoalDoneCB;
  ArRobot *myRobot;
  bool myEnabled;
  ArMutex myMutex;
  bool myHaveGoalNamePrefix, myHaveGoalNameSuffix;
  std::string myGoalNamePrefix, myGoalNameSuffix;

  // ArASyncTask() calls runThread(), but this renames it to remove the unused void* return type
  virtual void *runThread()
  {
    ArLog::log(ArLog::Normal, "%s: Running", getName());
    runTask();
    return 0;
  }

  /// Utility in case you are using ArRobot::move() in a task but want to wait in that task thread for the movement
  void waitForMoveDone()
  {
    while(!myRobot->isMoveDone())
      ArUtil::sleep(100);
  }

  /* This is the "goal done" callback called by the ARNL path planning thread
   * when the a goal point is sucessfully reached.  We run a new thread here to
   * perform our task.
   */
	void goalDone(ArPose pose)
	{
    if(myEnabled && matchCriteria())
      runAsync();
	}

  /* Check whether any criteria for running the task match the current goal */
  bool matchCriteria()
  {
    if(myHaveGoalNamePrefix && goalNamePrefixMatch(myGoalNamePrefix))
      return true;
    if(myHaveGoalNameSuffix && goalNameSuffixMatch(myGoalNameSuffix))
      return true;
    // TODO tags from goal (change "ICON" to tags)
    return false;
  }

  bool goalNamePrefixMatch(const std::string& prefix)
  {
    // todo more efficient compare
    return (myPathPlanningTask->getCurrentGoalName().compare(0, prefix.size(), prefix) == 0);
  }

  bool goalNameSuffixMatch(const std::string& suffix)
  {
    // todo more efficient compare
    const std::string& currentname = myPathPlanningTask->getCurrentGoalName();
    return (currentname.compare(currentname.size()-suffix.size(), currentname.size(), suffix));
  }
};

#endif
