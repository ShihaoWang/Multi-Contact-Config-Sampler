#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/World.h"
#include <ode/ode.h>
#include <math.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>


SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;

// The main purpose of this function is to create a number of initial configurations such that the experiemntation can be easily conducted.
int main_inner()
{
  std::ifstream FolderPathFile("./Specs/FolderPath.txt");
  std::string FolderPath;
  std::getline(FolderPathFile, FolderPath);
  FolderPathFile.close();

  std::ifstream EnviNameFile("./Specs/EnviName.txt");
  std::string EnviName;
  std::getline(EnviNameFile, EnviName);
  EnviNameFile.close();

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
  const std::string ContactStatusPath ="./Specs/ContactStatus.txt";
  std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  /* 3. Environment Geometry and Reachability Map*/
  const int GridsNo = 251;
  struct stat buffer;   // This is used to check whether "SDFSpecs.bin" exists or not.
  const string SDFSpecsName = "SDFSpecs.bin";
  if(stat (SDFSpecsName.c_str(), &buffer) == 0)
  {
    NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(GridsNo);
  }
  else
  {
    NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  }

  const int NumberOfTerrains = world.terrains.size();
  std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*world.terrains[0]);
  Meshing::TriMesh EnviTriMesh  = Terrain_ptr->geometry->AsTriangleMesh();
  for (int i = 0; i < NumberOfTerrains-1; i++)
  {
    std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*world.terrains[i+1]);
    Meshing::TriMesh EnviTriMesh_i  = Terrain_ptr->geometry->AsTriangleMesh();
    EnviTriMesh.MergeWith(EnviTriMesh_i);
  }
  AnyCollisionGeometry3D TerrColGeom(EnviTriMesh);

  Robot SimRobotObj = SimRobot;
  std::vector<double> InitRobotConfig(SimRobotObj.q.size(), 0.0);
  std::vector<double> InitRobotVelocity(SimRobotObj.q.size(), 0.0);

  RegionInfo LeftFoot, RightFoot, LeftHand, RightHand;
  RegionInfo COM;
  RegionInfoLoader(LeftFoot, RightFoot, LeftHand, RightHand, COM);

  std::vector<RegionInfo> RegionInfoObj = { LeftFoot, RightFoot, LeftHand, RightHand };

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

    InitRobotConfig = InitialConfigurationOptimization(SimRobotObj, RobotContactInfo, InitRobotConfig, RegionInfoObj, OptFlag);
    if(!OptFlag) continue;

    // Then SelfCollision Test!
    SimRobotObj.UpdateConfig(Config(InitRobotConfig));
    SimRobotObj.UpdateGeometry();
    SimRobotObj.dq = InitRobotVelocity;
    SelfCollisionTest = SimRobotObj.SelfCollision();
    if(SelfCollisionTest) continue;

    // Then EnvironmentCollision Test!
    std::vector<double> LinkTerrDistVec(SimRobot.q.size() - 6);
    for (int i = 6; i < SimRobot.q.size(); i++)
    {
      double LinkTerrDist = SimRobotObj.geometry[i]->Distance(TerrColGeom);
      AnyCollisionQuery CollisionObj(TerrColGeom, *SimRobotObj.geometry[i]);
      LinkTerrDistVec[i-6] = CollisionObj.PenetrationDepth();
    }
    if(*std::max_element(LinkTerrDistVec.begin(), LinkTerrDistVec.end())>0)
    {
      EnviCollisionTest = true;
      continue;
    }
    else
    {
      EnviCollisionTest = false;
    }
  }
  //  Given the optimized result to be the initial state
  Sim.world->robots[0]->UpdateConfig(Config(InitRobotConfig));
  Sim.world->robots[0]->dq = InitRobotVelocity;

  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitRobotConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitRobotVelocity));

  double InitDuration = 2.0;
  double TimeStep = 0.1;

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(*Sim.world->robots[0], COMPos, COMVel);

  double InitCOMDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(COMPos);
  while(Sim.time <= InitDuration)
  {
    CentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    double CurCOMDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(COMPos);

    double COMDistDiff = CurCOMDist - InitCOMDist;
    COMDistDiff = COMDistDiff * COMDistDiff;

    if(COMDistDiff>0.0001)
    {
      return -1;
    }

    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  int FileIndex = FileIndexFinder(false);
  const string FileName = "./res/" + to_string(FileIndex) + "/";
  RobotConfigWriter(InitRobotConfig, FileName, "InitConfig.config");
  string str = "cp " + ContactStatusPath + " " + FileName;
  const char *command = str.c_str();
  system(command);

  FileIndex = FileIndexFinder(true);
  std::printf("Initial Simulation Done!\n");
  return 1;
}

int main()
{
  int ConfigNo = FileIndexFinder(false);
  struct stat buffer;   // This is used to check whether "SDFSpecs.bin" exists or not.
  const string SDFSpecsName = "./res/" + to_string(ConfigNo);
  if(stat (SDFSpecsName.c_str(), &buffer) == 0)
  {
    // There exists this folder, we remove all the files within this folder
    string str = "cd ";
    str+=SDFSpecsName + " && rm -r *.*";
    const char *command = str.c_str();
    system(command);
  }
  else
  {
    // We then create this folder.
    string str = "mkdir " + SDFSpecsName;
    const char *command = str.c_str();
    system(command);
  }
  int res = -1;
  while(res == -1)
  {
    res = main_inner();
  }
  return 0;
}
