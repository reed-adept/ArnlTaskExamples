#ifndef ARNLREMOTEASYNCTASK_H
#define ARNLREMOTEASYNCTASK_H

/*
Copyright (c) 2017 Omron Adept MobileRobots LLC
All rights reserved. 
*/

#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientHandlerRobotUpdate.h"

/**
  Use this to help run your own custom tasks or activities, triggered when a 
  remote ARNL server reaches goals.

  When the ARNL server successfully reaches a goal, a new thread is created to perform your
  custom (potentially long-running) task.  Optionally, a goal's name must match either
  the given @a prefix or @a suffix.  (Default behavior is to run at each goal.)

  The ARNL server continues to execute asynchronously.

  Note: Since new threads are created to trigger each task, it is possible for
  more than one task thread to be running simultaneously (e.g. if ARNL is sent
  to a new goal and reaches it before a prior task has completed.)   If you wish
  to avoid this you can use a static flag and ArMutex, or use an ArCondition object to
  coordinate the threads.  Or design your application to always request goals at
  the end of a task thread. 

  To define a task for your application, either supply a callback
  function or define a new subclass of ArnlRemoteASyncTask.  

  To use a callback function in C++, create a functor object, and pass a pointer to
  the ArnlRemoteASyncTask constructor, with a task name.  In Python, you can also just 
  supply a reference to a function, or a lambda expression.  
  If that functor can recieve a goal 
  name (as a <tt>const std::string&</tt> argument) or goal position (as a 
  <tt>const ArPose&</tt>) (because a <tt>dynamic_cast</tt> is successful), then it 
  will be invoked with that argument.
 
  Use a callback if the action you want to perform can be done by
  calling a method of an existing class, or a global function.

  An alternative is to define a new subclass of ArnlRemoteAsyncTask which overrides the runTask() 
  and getName() methods.  
  You may also override the constructor, if needed, but be sure to call the
  base class constructor.  
  If, in addition to overriding the runTask()
  method, you supply a functor to ArnlRemoteASyncTask, the functor is called after
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

  You may call nextGoal("goal name"); to request a new goal.
  (This is just a shortcut to calling requestOnce() with "gotoGoal" and the new
  goal name.)

  @note one instance of this class is typically created for the
  whole program, but new threads may be created at any time (whenever ARNL happens
  to reach a goal), these threads are sharing access to the variables withhin the
  class, and so this access is synchronized using a mutex. Make certain you do
  not keep the mutex locked during any long running operations or operations of
  indeterminate duration, and that in all logical paths through the code, the
  mutex is eventually unlocked if locked.


  C++ Code Examples:

  @code{.cpp}
  class MyTask : public virtual ArnlRemoteASyncTask
  {
  public:
    MyTask(int defaultValue, ArClientBase *client, ArArgumentParser *argParser = NULL)  :
      ArnlRemoteASyncTask(client, argParser),
      myValue(defaultValue)
    {
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
      nextGoal("new goal");
      return;
    }
  };
  

  int main(int argc, char **argv)
  {
    //... initialize ARIA and connect client to server...

    MyTask task1(&server);
    task1.runIfGoalNameSuffix("-dotask1");

    // ...
  }
  @endcode

  @code{.cpp}
  void goalfunc(const std::string& goalname, const ArPose& pose)
  {
    puts("Goal!");
  }

  int main(int argc, char **argv)
  {
    //... initialize ARIA and connect client to server...
    ArGlobalFunctor2<const std::string&, const ArPose&> callback(&goalFunc));
    ArnlASyncTask task2(&server, &callback, "Example Task 2",
    task2.runIfGoalNameSuffix("-dotask2");
    // ...
  }
  @endcode


  Python Example:

  @code{.py}

  class MyTask: ArnlRemoteAsyncTask

    def init(self, value, server, argParser):
      ArnlRemoteAsyncTask.init(self, server, argParser)
      self.value = value

    def runTask:
      ArLog.log(ArLog.Normal, '%s: This is an example ARNL goal task.' % getName())
      ArUtil.sleep(5000)
      ArLog.log(ArLog.Normal, '%s: Task ended.' % getName())
      nextGoal('new goal')

  # ... initialize ARIA and ArNetworking and connect to server here ...

  task = MyTask(23, server, argParser)
  task.runIfGoalNameSuffix('-dotask1')
  @endcode



*/
class ArnlRemoteASyncTask : public virtual ArASyncTask
{
public:
  typedef ArFunctor2<const std::string&, const ArPose&> TaskFunctor;

protected:
  /// A default functor which does nothing.
  class NullTaskFunctor : public virtual TaskFunctor {
    virtual void invoke() {}
    virtual void invoke(const std::string&) {}
    virtual void invoke(const std::string&, const ArPose&) {}
  };


public:
  /**
    Supply a callback functor to call at goals.  See other constructor if
    defining a subclass instead.
    @param functor Functor referencing method or function to call. A copy of this functor is stored.
   */
  ArnlRemoteASyncTask(ArClientBase *client,
    const std::string& name, TaskFunctor *functor,
    ArArgumentParser *argParser = NULL,
    const std::string& goalPrefix = "", const std::string& goalSuffix = ""
  ) :
    myName(name),
    myUpdateHandler(client),
    myStatusChangedCB(this, &ArnlRemoteASyncTask::statusChanged),
    myFunctor(functor), myAllocatedFunctor(false)
  {
    init(client, argParser, goalPrefix, goalPrefix);
  }

protected:
  /** 
    Use this when definining a subclass that overrides runTask().
    @param goalPrefix If provided (not ""), beginning of goal name must match this prefix
      to run the task. If @a goalSuffix is also provided (not ""), both must match.
      If neither is provided (both are "") the task runs at all goals.
      (See also runIfGoalNamePrefix().)
    @param goalSuffix If provided (not ""), end of goal name must match this
      prefix to run the task. If @a goalPrefix is also provided (not ""), both must
      match. If neither is provided (both are "") the task runs at all goals.
      (See also runIfGoalNameSuffix().)
  */
  ArnlRemoteASyncTask(ArClientBase *client,
    const std::string& name = "unnamed ArnlASyncTask",
    ArArgumentParser *argParser = NULL,
    const std::string& goalPrefix = "", const std::string& goalSuffix = ""
  ) :
    myName(name),
    myUpdateHandler(client),
    myStatusChangedCB(this, &ArnlRemoteASyncTask::statusChanged),
    myFunctor(new NullTaskFunctor()), myAllocatedFunctor(true)
  {
    init(client, argParser, goalPrefix, goalPrefix);
  }

  virtual ~ArnlRemoteASyncTask()
  {
    myUpdateHandler.remStatusChangedCB(&myStatusChangedCB);
    myUpdateHandler.stopUpdates();
    if(myAllocatedFunctor) delete myFunctor;
  }

private:
  void init(ArClientBase *client, ArArgumentParser *argParser,
    const std::string& goalPrefix, const std::string& goalSuffix
  )
	{
    myClient = client;
    myHaveGoalNamePrefix = false;
    myHaveGoalNameSuffix = false;
    myUpdateHandler.addStatusChangedCB(&myStatusChangedCB);
    myUpdateHandler.requestUpdates();
    if(goalPrefix != "")
      runIfGoalNamePrefix(goalPrefix);
    if(goalSuffix != "")
      runIfGoalNameSuffix(goalSuffix);
  }

public:
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
  /** Override this method in a subclass to perform task actions, if no functor
    * has been supplied.. */
  virtual void runTask() {}


  /// Utility that you can use to easily set a new goal on the path planner task
  void nextGoal(const std::string goalName)
  {
    ArLog::log(ArLog::Normal, "%s: [%s] Sending request to go to new goal: %s", getName(), myClient->getHost(), goalName.c_str());
    myClient->requestOnceWithString("gotoGoal", goalName.c_str());
  }

protected:
  void lock() {
    myMutex.lock();
  }

  void unlock() {
    myMutex.unlock();
  }

  ArClientBase *getClient() { return myClient; }

private:
  std::string myName;
	ArClientBase *myClient;
  ArClientHandlerRobotUpdate myUpdateHandler;
  ArFunctor2C<ArnlRemoteASyncTask, const char*, const char*> myStatusChangedCB;
  ArMutex myMutex;
  bool myHaveGoalNamePrefix, myHaveGoalNameSuffix;
  std::string myGoalNamePrefix, myGoalNameSuffix;
  TaskFunctor* myFunctor;
  bool myAllocatedFunctor;
  ArPose myLastGoalPose;
  std::string myLastGoalName;

  /// ArASyncTask calls this in the new thread. Call subclass overloaded
  /// runTask() and invoke functor. (Either of which may be empty and do nothing
  /// depending on how the user is using this class.)
  /// @internal
  AREXPORT virtual void *runThread(void *)
  {
    const std::string gn = myLastGoalName;
    const ArPose& p = myLastGoalPose;
    ArLog::log(ArLog::Normal, "%s: [%s] Running at %s (%.2f, %.2f, %.2f) ...", getName(), getClient()->getHost(), gn.c_str(), p.getX(), p.getY(), p.getTh());
    runTask();
    myFunctor->invoke(gn, p);
    return 0;
  }

  /** This is the "goal done" callback called by the ARNL path planning thread
   * when the a goal point is sucessfully reached.  We run a new thread here to
   * perform our task.
   * @internal
   */
	void statusChanged(const char *m, const char *s)
	{
    const char *thisGoalName = getGoalNameFromStatus(s);

    if(thisGoalName == NULL)
    {
      return;
    }
      
    if(matchCriteria(thisGoalName))
    {
      myLastGoalPose = myUpdateHandler.getPose();
      myLastGoalName = thisGoalName;
      runAsync();
    }
	}

  /// Check whether any criteria for running the task match the current goal
  /// @internal
  bool matchCriteria(const std::string& gn)
  {
    if(myHaveGoalNamePrefix && prefixMatch(gn, myGoalNamePrefix))
      return true;
    if(myHaveGoalNameSuffix && suffixMatch(gn, myGoalNameSuffix))
      return true;
    // TODO tags from goal (change "ICON" to tags)
    return false;
  }

  /// @internal
  bool prefixMatch(const std::string& str, const std::string& prefix)
  {
    // todo more efficient compare
    return (str.compare(0, prefix.size(), prefix) == 0);
  }

  /// @internal
  bool suffixMatch(const std::string& str, const std::string& suffix)
  {
    // todo more efficient compare
    return (str.compare(str.size()-suffix.size(), str.size(), suffix));
  }

  const char *getGoalNameFromStatus(const std::string& s)
  {
    if(prefixMatch(s, "Arrived at"))
    {
      return s.substr(11).c_str(); // 11 == strlen("Arrived at ")
    }
    return NULL;
  }

};

typedef ArnlRemoteASyncTask ArnlRemoteAsyncTask;


#endif
