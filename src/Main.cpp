#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>

SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;

int main()
{
  std::string FolderPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/";
  std::string EnviName = "Envi1.xml";

  RobotWorld world;
  SimGUIBackend Backend(&world);
  WorldSimulation& Sim = Backend.sim;

  string XMLFileStr = FolderPath + EnviName;
  const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
  if(!Backend.LoadAndInitSim(XMLFile))
  {
    std::cerr<< EnviName<<" file does not exist in that path!"<<endl;
    return -1;
  }
  Robot SimRobot = *world.robots[0];

  /* 1. Load the Contact Link file */
  const std::string UserFilePath = FolderPath + "user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  NonlinearOptimizerInfo::RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);

  /* 2. Load the Contact Status file */
  const std::string ContactStatusPath ="/home/motion/Desktop/Multi-Contact-Config-Sampler/build/ContactStatus.txt";
  std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  /* 3. Environment Geometry and Reachability Map*/
  const int GridsNo = 251;
  // NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(GridsNo);

  std::vector<ContactStatusInfo> InitRobotContactInfo = RobotContactInfo;
  Robot SimRobotObj = SimRobot;
  std::vector<double> InitRobotConfig;
  std::vector<double> InitRobotVelocity(SimRobotObj.q.size(), 0.0);
  std::vector<double> RobotConfigRef = InitRobotVelocity;
  bool SelfCollisionTest = true;
  while(SelfCollisionTest == true)
  {
    for (int i = 6; i < SimRobotObj.q.size(); i++)
    {
      RobotConfigRef[i] = RandomValue(1.0);
    }
    RobotConfigRef[2] = 0.65;
    RobotConfigRef[23] = -1.0;
    RobotConfigRef[30] = 1.0;
    InitRobotConfig = InitialConfigurationOptimization(SimRobotObj, InitRobotContactInfo, RobotConfigRef);
    RobotConfigWriter(InitRobotConfig, "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/", "InitConfig.config");
    SimRobotObj.UpdateConfig(Config(InitRobotConfig));
    SimRobotObj.UpdateGeometry();
    SimRobotObj.dq = InitRobotVelocity;
    SelfCollisionTest = SimRobotObj.SelfCollision();
  }

  //  Given the optimized result to be the initial state
  Sim.world->robots[0]->UpdateConfig(Config(InitRobotConfig));
  Sim.world->robots[0]->dq = InitRobotVelocity;

  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitRobotConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitRobotVelocity));

  string FailureStateTrajStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

  double InitDuration = 2.0;
  double TimeStep = 0.025;

  string str = "cd /home/motion/Desktop/Multi-Contact-Config-Sampler/build/";
  str+="&& rm -f *StateTraj.path";
  const char *command = str.c_str();
  system(command);

  while(Sim.time <= InitDuration)
  {
    std::printf("Initial Simulation Time: %f\n", Sim.time);
    if((Sim.world->robots[0]->q[2])<0.35)
    {
      return -1;
    }
    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);

    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  std::printf("Initial Simulation Done!\n");

  return 1;
}
