#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/World.h"
#include <ode/ode.h>
#include <math.h>

SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;

// The main purpose of this function is to create a number of initial configurations such that the experiemntation can be easily conducted.
int main_inner()
{
  std::string FolderPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/";
  std::string EnviName = "Envi1.xml";

  string FailureStateTrajStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

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

  const int NumberOfTerrains = world.terrains.size();
  std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*world.terrains[0]);
  Meshing::TriMesh EnviTriMesh  = Terrain_ptr->geometry->AsTriangleMesh();
  for (int i = 0; i < NumberOfTerrains-1; i++)
  {
    std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*world.terrains[i+1]);
    Meshing::TriMesh EnviTriMesh_i  = Terrain_ptr->geometry->AsTriangleMesh();
    EnviTriMesh.MergeWith(EnviTriMesh_i);
  }
  CollisionMesh EnviTriMeshTopology(EnviTriMesh);
  EnviTriMeshTopology.InitCollisions();
  EnviTriMeshTopology.CalcTriNeighbors();

  Robot SimRobotObj = SimRobot;
  std::vector<double> InitRobotConfig(SimRobotObj.q.size(), 0.0);
  std::vector<double> InitRobotVelocity(SimRobotObj.q.size(), 0.0);

  RegionInfo LeftFoot, RightFoot, LeftHand, RightHand;
  RegionInfo COM;

  LeftFoot.xUpdate(0.0, 3.75);
  LeftFoot.yUpdate(-0.5, 0.5);
  LeftFoot.zUpdate(-0.25, 0.25);
  RightFoot = LeftFoot;
  LeftHand.xUpdate(3.95, 4.05);
  LeftHand.yUpdate(-1.0, 0.0);
  LeftHand.zUpdate(0.75, 1.15);
  RightHand = LeftHand;
  COM.xUpdate(0.0, 3.75);
  COM.yUpdate(-1.5, 0.5);
  COM.zUpdate(0.5, 0.8);

  std::vector<RegionInfo> RegionInfoObj = {LeftFoot, RightFoot, LeftHand, RightHand};

  bool SelfCollisionTest = true;
  bool EnviCollisionTest = true;
  int OptFlag = 0;
  while((SelfCollisionTest == true)||(EnviCollisionTest == true)||(OptFlag == 0))
  {
    Vector3 InitCOM = COM.Sample();
    InitRobotConfig = ConfigSampler(SimRobotObj);
    InitRobotConfig[0] = InitCOM[0];
    InitRobotConfig[1] = InitCOM[1];
    InitRobotConfig[2] = InitCOM[2];

    InitRobotConfig[3] = RandomValue(M_PI);
    InitRobotConfig[4] = RandomValue(M_PI/2.0);
    InitRobotConfig[5] = RandomValue(M_PI/2.0);

    int OptFlag = 0;
    InitRobotConfig = InitialConfigurationOptimization(SimRobotObj, RobotContactInfo, InitRobotConfig, RegionInfoObj, OptFlag);
    switch (OptFlag)
    {
      case 0:
      continue;
      break;
      default:
      break;
    }

    RobotConfigWriter(InitRobotConfig, "/home/motion/Desktop/Multi-Contact-Config-Sampler/build/", "InitConfig.config");

    // Then SelfCollision Test!
    SimRobotObj.UpdateConfig(Config(InitRobotConfig));
    SimRobotObj.UpdateGeometry();
    SimRobotObj.dq = InitRobotVelocity;
    SelfCollisionTest = SimRobotObj.SelfCollision();
    if(SelfCollisionTest)
    {
      continue;
    }
    // Then EnvironmentCollision Test!
    for (int i = 6; i < SimRobot.q.size(); i++)
    {
      CollisionMesh RobotLinkMeshTopology(RobotLinkMesh);
      RobotLinkMeshTopology.InitCollisions();
      // RobotLinkMeshTopology.CalcTriNeighbors();

      AnyCollisionQuery EnviCollision= new AnyCollisionQuery(*geometry[i],*SimRobotObj.geometry[i]);


      AnyCollisionQuery
      CollisionMeshQuery CollisionChecker(EnviTriMeshTopology, RobotLinkMeshTopology);

      std::printf("Link %d's Penetration Depth %f\n", i, CollisionChecker.PenetrationDepth());

      std::vector<Vector3> p1, p2;

      CollisionChecker.TolerancePoints(p1, p2);
      int a = 1;

      // if(CollisionChecker.Collide())
      // {
      //   EnviCollisionTest = true;
      //   break;
      // }
      // else
      // {
      //   EnviCollisionTest = false;
      // }
    }
    if(EnviCollisionTest)
    {
      continue;
    }
    int a = 1;
  }

  SimRobotObj.UpdateConfig(Config(InitRobotConfig));
  SimRobotObj.UpdateGeometry();

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);

  //  Given the optimized result to be the initial state
  Sim.world->robots[0]->UpdateConfig(Config(InitRobotConfig));
  Sim.world->robots[0]->dq = InitRobotVelocity;

  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitRobotConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitRobotVelocity));

  double InitDuration = 2.0;
  double TimeStep = 0.025;

  string str = "cd /home/motion/Desktop/Multi-Contact-Config-Sampler/build/";
  str+="&& rm -f *StateTraj.path";
  const char *command = str.c_str();
  system(command);

  double InitCOMDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(COMPos);
  while(Sim.time <= InitDuration)
  {
    CentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    double CurCOMDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(COMPos);

    double COMDistDiff = CurCOMDist - InitCOMDist;
    COMDistDiff = COMDistDiff * COMDistDiff;
    std::printf("Initial Simulation Time: %f, InitCOMDist: %f, CurCOMDist: %f, and COMDistDiff2: %f\n", Sim.time, InitCOMDist, CurCOMDist, COMDistDiff);

    if(COMDistDiff>0.01)
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

int main()
{
  int res = -1;
  while(res == -1)
  {
    res = main_inner();
  }
  return 0;
}
