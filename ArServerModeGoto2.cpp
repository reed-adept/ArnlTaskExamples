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
#include "Aria.h"
#include "ArNetworking.h"
#include "ArExport.h"
//#include "ariaUtil.h"
#include "ArServerClasses.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"
#include <math.h>
#include <errno.h>

#include "ArServerModeGoto2.h"


AREXPORT ArServerModeGoto2::ArServerModeGoto2(
	ArServerBase *server, ArRobot *robot, ArPathPlanningTask *pathTask,
	ArMapInterface *arMap, ArPose home, ArRetFunctor<ArPose> *getHomePoseCB) :
  ArServerMode(robot, server, "Goto"),
  myGoalDoneCB(this, &ArServerModeGoto2::goalDone),
  myGoalFailedCB(this, &ArServerModeGoto2::goalFailed),
  myServerGetGoalsCB(this, &ArServerModeGoto2::serverGetGoals),
  myServerGotoGoalCB(this, &ArServerModeGoto2::serverGotoGoal),
  myServerGotoPoseCB(this, &ArServerModeGoto2::serverGotoPose),
  myServerHomeCB(this, &ArServerModeGoto2::serverHome),
  myServerTourGoalsCB(this, &ArServerModeGoto2::serverTourGoals),
  myServerGoalNameCB(this, &ArServerModeGoto2::serverGoalName),
  myTourGoalsInListSimpleCommandCB(this, &ArServerModeGoto2::tourGoalsInListCommand)
{

  myServer = server;
  myRobot = robot;
  myPathTask = pathTask;
  myGoingHome = false;
  myTouringGoals = false;
  myMap = arMap;
  myHome = home;
  myGetHomePoseCB = getHomePoseCB;
  myAmTouringGoalsInList = false;

  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);
  addModeData("gotoGoal", "sends the robot to the goal", 
	      &myServerGotoGoalCB, 
	      "string: goal", "none", "Navigation", "RETURN_NONE");
    
  addModeData("gotoPose", 
	      "sends the robot to a given x, y and optional heading", 
	      &myServerGotoPoseCB, 
	      "byte4: x byte4: y (optional) byte4: th", "none", "Navigation",
	      "RETURN_NONE");
  addModeData("home", "Sends the robot to where it started up",
	      &myServerHomeCB, "none", "none", "Navigation", "RETURN_NONE");
  myServer->addData("goalName", "current goal name", &myServerGoalNameCB, "none", "string", "Navigation", "RETURN_SINGLE");
  if (myMap != NULL)
  {
    addModeData("tourGoals",
		"sends the robot on a tour of all the goals",
		&myServerTourGoalsCB, "none", 
		"none", "Navigation", "RETURN_NONE");
  }
  myServer->addData("getGoals", "gets the list of goals", 
		    &myServerGetGoalsCB, "none", 
		    "<repeat> string: goal", "NavigationInfo", 
		    "RETURN_SINGLE");
}

AREXPORT ArServerModeGoto2::~ArServerModeGoto2()
{

}

AREXPORT void ArServerModeGoto2::activate(void)
{

  if (!baseActivate())
  {
    return;
  }

  if(myTouringGoals)
  {
    planToNextTourGoal();
  }
  else
  {
    if (myGoalName.size() > 0)
    {
      if(!myPathTask->pathPlanToGoal(myGoalName.c_str()))
      {
        ArLog::log(ArLog::Terse, "Error: Could not plan a path to \"%s\".", myGoalName.c_str());
        myStatus = "Failed to plan to ";
        myStatus += myGoalName;
      }
    }
    else
    {
      if(!myPathTask->pathPlanToPose(myGoalPose, myUseHeading))
      {
        ArLog::log(ArLog::Terse, "Error: Could not plan a path to point.");
        myStatus = "Failed to plan to point";
      }
    }
  }

  myDone = false;
  setActivityTimeToNow();
}

AREXPORT void ArServerModeGoto2::deactivate(void)
{
  baseDeactivate();
  myPathTask->cancelPathPlan();
}

AREXPORT void ArServerModeGoto2::userTask(void)
{
  
  if (!myDone)
  {
    setActivityTimeToNow();

  }
}

AREXPORT void ArServerModeGoto2::gotoPose(ArPose pose, bool useHeading)
{
  reset();
  myGoalPose = pose;
  myUseHeading = useHeading;
  myStatus = "Going to point";
  myMode = "Goto point";
  activate();
}

AREXPORT void ArServerModeGoto2::home(void)
{
  reset();
  if (myGetHomePoseCB != NULL)
    myGoalPose = myGetHomePoseCB->invokeR();
  else
    myGoalPose = myHome;
  myUseHeading = true;
  myGoingHome = true;
  myStatus = "Returning home";
  myMode = "Go home";
  activate();
}

AREXPORT void ArServerModeGoto2::gotoGoal(const char *goal)
{
  reset();
  myGoalName = goal;
  myMode = "Goto goal";
  myStatus = "Going to ";
  myStatus += goal;
  activate();
}

AREXPORT void ArServerModeGoto2::tourGoals(void)
{
  std::string onGoal;

  onGoal = myGoalName;
  reset();
  myGoalName = onGoal;
  myTouringGoals = true;
  myAmTouringGoalsInList = false;
  myMode = "Touring goals";
  ArLog::log(ArLog::Normal, "Touring goals");
  //findNextTourGoal(); moved to activate()
  activate();
}

/** Does not check if goals listed are valid goals. */
AREXPORT void ArServerModeGoto2::tourGoalsInList(std::deque<std::string> goalList)
{
  std::string onGoal = myGoalName;
  reset();
  myGoalName = onGoal;
  myTouringGoals = true;
  myAmTouringGoalsInList = true;
  myTouringGoalsList = goalList;
  myMode = "Touring goals";
  ArLog::log(ArLog::Normal, "Tour goals: touring %d goals from given list", goalList.size());
  //findNextTourGoal(); moved to activate()
  
  // reactivate (start tour over again)
  activate();
}

AREXPORT void ArServerModeGoto2::addTourGoalsInListSimpleCommand(ArServerHandlerCommands *commandsServer)
{
  commandsServer->addStringCommand("TourGoalsList", 
    "Tour goals in the given list. Separate goal names with commas. "\
    "To add multiple goals with a common prefix, use the prefix followed by a *.",
    &myTourGoalsInListSimpleCommandCB);
}



AREXPORT bool ArServerModeGoto2::isAutoResumeAfterInterrupt()
{
  return myTouringGoals;
}

#ifdef WIN32
// on Windows, strtok() uses thread-local storage, and can be safely called by multiple threads
char *strtok_r(char *str, const char *delim, char **thing)
{
  return strtok(str, delim);
}
#endif

AREXPORT void ArServerModeGoto2::tourGoalsInListCommand(ArArgumentBuilder *args)
{
  char *str = strdup(args->getFullString());
  char *strtokpriv;
  char *tok = strtok_r(str, ",", &strtokpriv);
  //ArArgumentBuilder splitArgs(512, ','); // ArgumentBuilder always splits on
  //space
  //splitArgs.add(args->getArg(0));
  std::deque<std::string> goals;
  for(size_t i = 0; tok != NULL; i++)
  {
    // Strip preceding and following whitespace
    while(*tok && isspace(*tok))
    {
      ++tok;
    }
    char *tokend = tok + (strlen(tok) - 1);
    while(tokend && *tokend && isspace(*tokend))
    {
      *tokend-- = '\0';
    }

    // If a goal by this name exists, add it
    std::string s = tok;
    if(s.size() > 0) // skip empty strings
    {
      // if it has a * as the last char, then search for matching goals,
      // otherwise just push it into the list, if it exists.
      std::string::size_type starPos = s.find('*');
      if(starPos == s.npos)
      {
        myMap->lock();
        if(myMap->findMapObject(s.c_str(), "Goal") || myMap->findMapObject(s.c_str(), "GoalWithHeading"))
        {
          ArLog::log(ArLog::Normal, "Tour goals: adding \"%s\" to tour list.", s.c_str());
          goals.push_back(s);
        }
        else
        {
          ArLog::log(ArLog::Terse, "Tour goals: Warning: not adding \"%s\" to tour list; no goal by that name found in the map.", s.c_str());
        }
        myMap->unlock();
      }
      else if(starPos == s.size()-1)
      {
        // Find matching goals
        std::string prefix = s.substr(0, starPos);
        ArLog::log(ArLog::Normal, "Tour goals: searching for goals with prefix \"%s\"...", prefix.c_str());
        myMap->lock();
        for(std::list<ArMapObject*>::const_iterator i = myMap->getMapObjects()->begin();
            i != myMap->getMapObjects()->end(); i++)
        {
          if(!(*i)) continue;
          if( ! (strcasecmp((*i)->getType(), "GoalWithHeading") == 0 ||
                 strcasecmp((*i)->getType(), "Goal") == 0) )
            continue;
          const char *goalName = (*i)->getName();
          if(strncmp(goalName, prefix.c_str(), prefix.size()) == 0)
          {
            ArLog::log(ArLog::Normal, "\t...Adding matching goal \"%s\" to tour.", goalName);
            goals.push_back(goalName);
          }
        }
        myMap->unlock();
      }
      else
      {
        ArLog::log(ArLog::Terse, "Tour goals: Error in goal list; the \'*\' wildcard must be the last character in the goal name (in \"%s\"). starPos=%d, npos=%d, size=%d", s.c_str(), starPos, s.npos, s.size());
        free(str);
        return;
      }
    }

    // Find next token
    tok = strtok_r(NULL, ",", &strtokpriv);
  }
  free(str);
  tourGoalsInList(goals);
}


size_t ArServerModeGoto2::numGoalsTouring()
{
  if(!myTouringGoals) return 0;
  if(myAmTouringGoalsInList)
  {
    return myTouringGoalsList.size();
  }
  else
  {
    if(!myMap) return 0;
    size_t count = 0;
    myMap->lock();
    for (std::list<ArMapObject*>::const_iterator i = myMap->getMapObjects()->begin(); 
         i != myMap->getMapObjects()->end(); 
         i++)
    {
      ArMapObject *obj = (*i);
      if ((strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
         strcasecmp(obj->getType(), "Goal") == 0))
      {
        ++count;
      }
    }
    myMap->unlock();
    return count;
  }
}

void ArServerModeGoto2::findNextTourGoal(void)
{
  if (myMap == NULL)
  {
    myGoalName = "";
    return;
  }


  if(myAmTouringGoalsInList)
  {
    // If we are selecting goals from a list, return the head
    // and move it to the back.
    myGoalName = myTouringGoalsList.front();
    myTouringGoalsList.pop_front();
    myTouringGoalsList.push_back(myGoalName);
    ArLog::log(ArLog::Verbose, "Tour goals: popped next goal \"%s\" from user's list.", myGoalName.c_str());
  }
  else
  {
    // Otherwise, search the map's goals for the current goal,
    // and return the next one.
    std::list<ArMapObject *>::iterator objIt;
    ArMapObject* obj;
    bool nextGoalIt = false;
    bool gotGoal = false;
    std::string firstGoal;
    myMap->lock();
    for (objIt = myMap->getMapObjects()->begin(); 
         objIt != myMap->getMapObjects()->end(); 
         objIt++)
    {
      obj = (*objIt);
      if ((strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
         strcasecmp(obj->getType(), "Goal") == 0))
      {
        if (nextGoalIt)
        {
          myGoalName = obj->getName();
          gotGoal = true;
          break;
        }
        if (strcasecmp(obj->getName(), myGoalName.c_str()) == 0)
        {
          nextGoalIt = true;
        }
        if (firstGoal.size() <= 0)
          firstGoal = obj->getName();
      }
    }
    myMap->unlock();
    if (!gotGoal)
      myGoalName = firstGoal;
  }

  myStatus = "Touring to ";
  myStatus += myGoalName;
  //myPathTask->unlock();
  //myRobot->unlock();

}


void ArServerModeGoto2::reset(void)
{
  myGoingHome = false;
  myTouringGoals = false;
  myGoalName = "";
  myUseHeading = true;
}

// keep trying to plan to goals in tour, until either one suceeds or all goals fail
void ArServerModeGoto2::planToNextTourGoal()
{
  size_t failedCount = 0;
  size_t numGoals = numGoalsTouring();
  while(failedCount < numGoals) 
  {
    findNextTourGoal();
    if(myPathTask->pathPlanToGoal(myGoalName.c_str()))
    {
      return;
    }
    else
    {
      ++failedCount;
      ArLog::log(ArLog::Terse, "Tour goals: Warning: failed to plan a path to \"%s\".", myGoalName.c_str());
    }
  }
  ArLog::log(ArLog::Terse, "Tour goals: Warning: failed to find a path to any goal.");
  myStatus = "Failed touring goals: All goals failed.";
}

// TODO move this to ArPathPlanningTask
ArMapObject* ArServerModeGoto2::getCurrentGoalObject()
{
  const char *goalName = myGoalName.c_str();
  ArMapObject *obj = myMap->findMapObject(goalName, "GoalWithHeading");
  if(!obj) obj = myMap->findMapObject(goalName, "Goal");
  return obj;
}

void ArServerModeGoto2::goalDone(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  if (myGoingHome)
  {
    myDone = true;
    myStatus = "Returned home";
  }
  else if (myTouringGoals)
  {
    ArMapObject *obj = getCurrentGoalObject();
    if(obj) myTourCallbacks.invoke(obj);
    planToNextTourGoal();
  }
  else if (myGoalName.size() > 0)
  {
    myDone = true;
    myStatus = "Arrived at ";
    myStatus += myGoalName;
  }
  else
  {
    myDone = false;
    myStatus = "Arrived at point";
  }
}

void ArServerModeGoto2::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  if (myPathTask->getAriaMap() == NULL || 
      strlen(myPathTask->getAriaMap()->getFileName()) <= 0)
  {
    myDone = true;
    myStatus = "Failed driving because map empty";
    ArLog::log(ArLog::Normal, "Failed driving because map empty");
    return;
  }
  if (myTouringGoals)
  {
    if (ArUtil::strcasecmp(myStatus, "Robot lost") == 0)
    {
      myStatus = "Failed touring because robot lost";
    }
    else
    {
      planToNextTourGoal();
    }
  }
  else
  {
    myDone = true;
    std::string oldStatus;
    oldStatus = myStatus;
    if (myGoingHome)
      myStatus = "Failed to get home";
    else if (myGoalName.size() > 0)
    {
      myStatus = "Failed to get to ";
      myStatus += myGoalName;
    }
    else
    {
      myStatus = "Failed to get to point";
    }

    
    if (ArUtil::strcasecmp(oldStatus, "Robot lost") == 0)
    {
      myStatus += " because robot lost";
    }
    else 
    {
      char failureStr[512];
      myPathTask->getFailureString(failureStr, sizeof(failureStr));
      myStatus += " (" + std::string(failureStr) + ")";
    }
  }
}

AREXPORT void ArServerModeGoto2::serverGotoGoal(ArServerClient * /*client*/, 
					       ArNetPacket *packet)
{
  char buf[512];
  packet->bufToStr(buf, sizeof(buf)-1);
  ArLog::log(ArLog::Normal, "Going to goal %s", buf);
  //myRobot->lock();
  gotoGoal(buf);
  //myRobot->unlock();
}

AREXPORT void ArServerModeGoto2::serverGotoPose(ArServerClient * /*client*/, 
					       ArNetPacket *packet)
{
  ArPose pose;
  bool useHeading = false;

  pose.setX(packet->bufToByte4());
  pose.setY(packet->bufToByte4());
  if (packet->getDataLength() > packet->getDataReadLength())
  {
    useHeading = true;
    pose.setTh(packet->bufToByte4());
  }
  //myRobot->lock();
  ArLog::log(ArLog::Normal, "Going to point");
  gotoPose(pose, useHeading);
  //myRobot->unlock();
}


AREXPORT void ArServerModeGoto2::serverHome(ArServerClient * /*client*/, 
					   ArNetPacket * /*packet*/)
{
  ArLog::log(ArLog::Normal, "Going home");
  //myRobot->lock();
  home();
  //myRobot->unlock();
}

AREXPORT void ArServerModeGoto2::serverTourGoals(ArServerClient * /*client*/,
						ArNetPacket * /*packet*/ )
{
  ArLog::log(ArLog::Normal, "Touring goals");
  //myRobot->lock();
  tourGoals();
  //myRobot->unlock();
}


AREXPORT void ArServerModeGoto2::serverGetGoals(ArServerClient *client, 
					       ArNetPacket * /*packet*/ )
{
  ArNetPacket sendPacket;
  ArLog::log(ArLog::Verbose, "getGoals requested");

  if (myMap == NULL)
  {
    client->sendPacketTcp(&sendPacket);
    myPathTask->unlock();
    return;
  }

  myMap->lock();
  std::list<ArMapObject *>::iterator objIt;
  ArMapObject* obj;
  for (objIt = myMap->getMapObjects()->begin(); 
       objIt != myMap->getMapObjects()->end(); 
       objIt++)
  {
    obj = (*objIt);
    if (strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
	strcasecmp(obj->getType(), "Goal") == 0)
    {
      sendPacket.strToBuf(obj->getName());
    }
  }
  myMap->unlock();
  client->sendPacketTcp(&sendPacket);
}







void ArServerModeGoto2::serverGoalName(ArServerClient *client, ArNetPacket * /*pkt*/)
{
    ArNetPacket retPkt;
    retPkt.strToBuf(myGoalName.c_str());
    client->sendPacketTcp(&retPkt);
}


AREXPORT void ArServerModeGoto2::addTourGoalCallback(ArFunctor1<ArMapObject*> *func)
{
  // TODO add a callback type to ARNL which provides the ArMapObject*.
}
