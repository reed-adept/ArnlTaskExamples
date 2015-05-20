/*
Adept MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.9.0

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006-2009 MobileRobots Inc.
Copyright (C) 2010-2015 Adept Technology, Inc.

All Rights Reserved.

Adept MobileRobots does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
Adept MobileRobots
10 Columbia Drive
Amherst, NH 03031
+1-603-881-7960

*/
#ifndef ARSERVERMODEGOTO2_H
#define ARSERVERMODEGOTO2_H

#include "Aria.h"
#include "ArServerBase.h"
#include "ArServerMode.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"

#include <deque>
#include <string>

//class ArPathPlanningTask;
class ArActionPlanAndMoveToGoal;

class ArServerModeGoto2 : public ArServerMode
{
public:
  AREXPORT ArServerModeGoto2(ArServerBase *server, ArRobot *robot, 
			    ArPathPlanningTask *pathTask, ArMapInterface *arMap, 
			    ArPose home = ArPose(0, 0, 0), 
			    ArRetFunctor<ArPose> *getHomePoseCB = NULL);
  AREXPORT virtual ~ArServerModeGoto2();
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
  AREXPORT void home(void);
  AREXPORT void gotoGoal(const char *goal);
  AREXPORT void gotoPose(ArPose pose, bool useHeading);

  /** Enter a "tour goals" mode, in which the robot is sent to each goal in the
   *  map in turn. This mode can be entered using the tourGoals networking
   *  request (this method is called internally when tourGoals is received). 
   */
  AREXPORT void tourGoals(void);

  /** Enter a "tour goals" mode, in which the robot is sent to each goal in the
   *  given list in turn.  This method is called internally when the
   *  tourGoalsInList simple command is received.
   *
   *  @todo Use an ArArgumentBuilder instead of a deque?
   */
  AREXPORT void tourGoalsInList(std::deque<std::string> goalList);

  /** Add a "tour" command to the given "simple commands" object. This
   *  simple (custom) command accepts a comma-separated list of goals,
   *  builds a list of goals, expanding items ending in a wildcard (*)
   *  to maching goals, and omitting invalid goals,
   *  and then calls tourGoalsInList().
   */
  AREXPORT void addTourGoalsInListSimpleCommand(ArServerHandlerCommands *commandsServer);

  /** Add a callback which is called for each goal when touring goals */
  AREXPORT void addTourGoalCallback(ArFunctor1<ArMapObject*> *callback);

  /** @internal */
  AREXPORT virtual bool isAutoResumeAfterInterrupt(void);

protected:
  AREXPORT void serverGetGoals(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverGotoPose(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverGotoGoal(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverHome(ArServerClient *client,
			   ArNetPacket *packet);
  AREXPORT void serverTourGoals(ArServerClient *client,
				ArNetPacket *packet);
  AREXPORT virtual void userTask(void);

  /** Reset state */
  void reset(void);

  /** Callback from ARNL when a goal is reached. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalDone(ArPose pose);

  /** Callback from ARNL when a goal fails. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalFailed(ArPose pose);

  /// Set myGoalName to the name of the next goal in the tour
  void findNextTourGoal(void);

  /** @return number of goals in current tour, or 0 if none */
  size_t numGoalsTouring();


  /// Keep trying to plan paths to goals in a tour, until either a plan succeeds or all the goals fail.
  void planToNextTourGoal();

  ArMapObject *getCurrentGoalObject();

  ArPose myGoalPose;
  bool myDone;
  bool myUseHeading;
  std::string myGoalName;
  bool myGoingHome;
  ArMapInterface *myMap;
  ArPose myHome;
  ArRetFunctor<ArPose> *myGetHomePoseCB;
  bool myTouringGoals;
  ArPathPlanningTask *myPathTask;
  ArFunctor1C<ArServerModeGoto2, ArPose> myGoalDoneCB;
  ArFunctor1C<ArServerModeGoto2, ArPose> myGoalFailedCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerGetGoalsCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerGotoGoalCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerGotoPoseCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerHomeCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerTourGoalsCB;
  ArFunctor2C<ArServerModeGoto2, ArServerClient *, ArNetPacket *> myServerGoalNameCB;

  void serverGoalName(ArServerClient* client, ArNetPacket* pkt);
  void pathPlannerStateChanged();
  std::deque<std::string> myTouringGoalsList; ///< @todo use an ArArgumentBuilder instead of a deque?
  bool myAmTouringGoalsInList;
  ArFunctor1C<ArServerModeGoto2, ArArgumentBuilder*> myTourGoalsInListSimpleCommandCB;
  AREXPORT void tourGoalsInListCommand(ArArgumentBuilder *args); ///< Used as callback from ArServerHandlerCommands (simple/custom commands)

  ArCallbackList1<ArMapObject*> myTourCallbacks;
};

#endif

