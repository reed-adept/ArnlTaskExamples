#ifndef ARNLASYNCTASK_H
#define ARNLASYNCTASK_H

/*
Copyright (c) 2017 Omron Adept MobileRobots LLC
All rights reserved. 
*/

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningTask.h"

/**
  Use this to help run your own custom tasks or activities, triggered when ARNL navigation
  reaches goals.

  When ARNL (@ pp) successfully reaches a goal, a new thread is created to perform your
  custom (potentially long-running) task.  Optionally, a goal's name must match either
  the given @a prefix or @a suffix.  (Default behavior is to run at each goal.)

  The ARNL path planning thread continues to execute asynchronously.

  To define a task for your application, either supply a callback
  function or define a new subclass of ArnlAsyncTAsk.  

  To use a callback function in C++, create a functor object, and pass it to 
  the ArASyncTask constructor, with a task name.  In Python, you can also just 
  supply a reference to a function, or a lambda expression.   You can supply 
  either a pointer or copy of any ArFunctor. If that functor can recieve a goal 
  name (as a <tt>const std::string&</tt> argument) or goal position (as a 
  <tt>const ArPose&</tt>) (because a <tt>dynamic_cast</tt> is successful), then it 
  will be invoked with that argument.
 
  Use a callback if the action you want to perform can be done by
  calling a method of an existing class, or a global function.

  An alternative is to define a new subclass of ArnlAsyncTask which overrides the runTask() 
  and getName() methods.  
  You may also override the constructor, if needed, but be sure to call the
  base class constructor.  
  If, in addition to overriding the runTask()
  method, you supply a functor to ArnlASyncTask, the functor is called after
  runTask() is called.

  Use a subclass if you want to encapsulate complex behavior requiring multiple
  methods, and store data in member variables, in a class.

  Create an instance of this class in your program
  such that it will not be deleted for the duration of the process. For example,
  you could create it in your program's main() function, or allocate it with
  <tt>new</tt>.

  The base class adds a config section named for the task containing a flag
  to enable or disable the task.  You may add additional configuration
  parameters to this section if desired by calling addConfigParam().


  You may call nextGoal("goal name"); to plan to another goal if desired.
  (This is just a shortcut to calling ArPathPlanningTask::pathPlanToGoal()).


  C++ Code Examples:

  @code{.cpp}
  class MyTask : public virtual ArnlASyncTask
  {
  public:
    MyTask(int defaultValue, ArPathPlanningTask *path, ArRobot *robot, ArArgumentParser *argParser = NULL)  :
      ArnlASyncTask(path, robot, argParser),
      myValue(defaultValue)
    {
      addConfigParam(ArConfigArg("Example Value", &myValue));
    }

    virtual const char *getName() const
    {
      return "My Example Task";
    }

    virtual void runTask()
    {
      ArLog::log(ArLog::Normal, "%s: This is an example ARNL goal task.", getName());
      ArUtil::sleep(5000);
      ArLog::log(ArLog::Normal, "%s: Task ended.", getName());
      return 0;
    }
  };

  void goalfunc(const std::string& goalname, const ArPose& pose)
  {
    puts("Goal!");
  }
  @endcode

  ...

  @code{.cpp}
  MyTask task1(&pathPlanningTask, &robot, &argParser);

  task1.runIfGoalNameSuffix("-dotask1");

  ArnlASyncTask task2(&pathPlanningTask, &robot, &argParser, "Example Task 2",
    ArGlobalFunctor2<const std::string&, const ArPose&>(&goalFunc));

  task2.runIfGoalNameSuffix("-dotask2");
  @endcode


  Python Example:

  @code{.py}

  class MyTask: ArnlAsyncTask
    def init(self, value, pathTask, robot, argParser):
      ArnlAsyncTask.init(self, pathTask, robot, argParser)
      self.value = value
      addConfigParam(ArConfigArg('Example Value'), self.value)
    def runTask:
      ArLog.log(ArLog.Normal, '%s: This is an example ARNL goal task.' % getName())
      ArUtil.sleep(5000)
      ArLog.log(ArLog.Normal, '%s: Task ended.' % getName())
  @endcode


  @note Since ArPathPlanningTask continues to execute asynchronously, it is posnsible
  for it to begin navigating to another goal while your task is also still executing
  (for example, sent via MobileEyes.)  You can use ArPathPlanningInterface::addNewGoalCB()
  to register a callback in order to be notified if a new goal is set.

  @note one instance of this class is created for the
  whole program, but new threads may be created at any time (whenever ARNL happens
  to reach a goal), these threads are sharing access to the variables withhin the
  class, and so this access is synchronized using a mutex. Make certain you do
  not keep the mutex locked during any long running operations or operations of
  indeterminate duration, and that in all logical paths through the code, the
  mutex is eventually unlocked if locked.


*/
class ArnlASyncTask : public virtual ArASyncTask
{
public:

  typedef ArFunctor<const std::string&, const ArPose&> TaskFunctor;

  class NullTaskFunctor : public virtual TaskFunctor {
    virtual void invoke(const std::string&, const ArPose&) {}
  };

  /**
    @param functor A copy is stored.
   */
  ArnlASyncTask(ArPathPlanningTask *pp, ArRobot *robot, ArArgumentParser *argParser = NULL,
    const std::string& name = "unnamed ArnlASyncTask",
    TaskFunctor &functor = NullTaskFunctor(),
    const std::string& goalPrefix = "", const std::string& goalSuffix = ""
  ) :
    myName(name),
    myGoalDoneCB(this, &ArnlASyncTask::goalDone),
    myFunctor(functor)
  {
    init(pp, robot, argParser, goalPrefix, goalPrefix);
  }

protected:
  ArnlASyncTask(ArPathPlanningTask *pp, ArRobot *robot, ArArgumentParser *argParser = NULL,
    const std::string& goalPrefix = "", const std::string& goalSuffix = ""
  ) :
    myGoalDoneCB(this, &ArnlASyncTask::goalDone)
  {
    init(pp, robot, argParser, goalPrefix, goalSuffix);
  }


private:
  void init(ArPathPlanningTask *pp, ArRobot *robot, ArArgumentParser *argParser,
    const std::string& goalPrefix, const std::string& goalSuffix
  )
	{
    myPathPlanningTask = pp;
		myRobot = robot;
    myEnabled = true;
    myHaveGoalNamePrefix = false;
    myHaveGoalNameSuffix = false;
		ArConfig *config = Aria::getConfig();
		config->addParam(ArConfigArg("Enabled", &myEnabled, "Whether this task is enabled"), getConfigSectionName());
		myPathPlanningTask->addGoalDoneCB(&myGoalDoneCB);
    if(goalPrefix != "")
      runIfGoalNamePrefix(goalPrefix);
    if(goalSuffix != "")
      runIfGoalNameSuffix(goalSuffix);
  }

  virtual const char *getName() const { return myName.c_str(); }

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

protected:
  /** Override this method in a subclass. */
  virtual void runTask() {}


  /// Utility that you can use to easily set a new goal on the path planner task
  void nextGoal(const std::string goalName)
  {
    ArLog::log(ArLog::Normal, "%s: Going to new goal: %s", getName(), goalName.c_str());
    myPathPlanningTask->pathPlanToGoal(goalName.c_str());
  }

  /// Utility in case you are using ArRobot::move() in a task but want to wait in that task thread for the movement
  void waitForMoveDone()
  {
    myRobot->lock();
    while(!myRobot->isMoveDone())
    {
      myRobot->unlock();
      ArUtil::sleep(100);
      myRobot->lock();
    }
    myRobot->unlock();
  }

private:
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

private:
	ArPathPlanningTask *myPathPlanningTask;
  ArFunctor1C<ArnlASyncTask, ArPose> myGoalDoneCB;
  ArRobot *myRobot;
  bool myEnabled;
  ArMutex myMutex;
  bool myHaveGoalNamePrefix, myHaveGoalNameSuffix;
  std::string myGoalNamePrefix, myGoalNameSuffix;
  TaskFunctor myFunctor;
  ArPose myLastGoalPose;
  std::string myLastGoalName;

  // ArASyncTask() calls runThread(), but this renames it to remove the unused void* return type
  // It also invokes the functor.
  virtual void *runThread()
  {
    const std::string gn = myLastGoalName;
    const ArPose& p = myLastGoalPose;
    ArLog::log(ArLog::Normal, "%s: Running at %s (%.2f, %.2f, %.2f) ...", getName(), gn.c_str(), p.getX(), p.getY(), p.getTh());
    runTask();
    myFunctor.invoke(gn, p);
    return 0;
  }

  /* This is the "goal done" callback called by the ARNL path planning thread
   * when the a goal point is sucessfully reached.  We run a new thread here to
   * perform our task.
   */
	void goalDone(ArPose pose)
	{
    if(myEnabled && matchCriteria())
    {
      myLastGoalPose = pose;
      myLastGoalName = myPathPlattingTask->GetCurrentGoalName();
      runAsync();
    }
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
