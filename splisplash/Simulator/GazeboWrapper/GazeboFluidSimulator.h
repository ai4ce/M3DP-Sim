/* Desc: Fluid World plugin
 * Author: Manos Angelidis
 * Email: angelidis@fortiss.org
 * Date: 15 Dec 2018
 */

#ifndef FLUID_SIMULATOR_HH
#define FLUID_SIMULATOR_HH

#include <vector>
#include <map>
#include <string>
#include <list>
#include <algorithm>
#include <fstream>
#include <iostream>

#include "extern/pystring/pystring.h"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "SPlisHSPlasH/TriangleMesh.h"
#include "GazeboSimulatorBase.h"
#include "SPlisHSPlasH/StaticRigidBody.h"
#include <memory>

enum CollisionGeometry
{
  plane,
  sphere,
  box,
  cylinder,
  mesh,
  trimesh,
  polyline,
  heightmap,
  multiray,
  ray
};

namespace gazebo
{



/// \brief FluidSimulator class
class GAZEBO_VISIBLE FluidSimulator : public ModelPlugin
{

  /// \brief Constructor
public:
  FluidSimulator();
  std::string name1, name2;

  /// \brief Destructor
public:
  ~FluidSimulator();

  /// \brief Initialize the fluid simulator
public:
  void Init();

  /// \brief Run a timestep in the fluid world
public:
  void RunStep();
  void ImportModel(std::string name, float size);
  void RemoveModel(std::string name);

public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  /// \brief Load plugin
protected:
  void initBoundaryData();
  void reset();
  void loadObj(const std::string &filename, SPH::TriangleMesh &geo, const Vector3r &scale);
  void ParseSDF();
  void RegisterMesh(physics::CollisionPtr collision, std::string extension, std::string path);

  int activeParticles;
  int maxParticles;
  int particles_number;


private:
  std::vector<event::ConnectionPtr> connections;
  std::unique_ptr<SPH::GazeboSimulatorBase> base;

  // maps from gazebo to the fluid simulation
  std::map<std::string, physics::CollisionPtr> filenamesToCollisions;
  std::map<SPH::StaticRigidBody *, physics::CollisionPtr> boundariesToCollisions;

  // sdf pointers
  physics::WorldPtr world;
  //physics::ModelPtr model2;
  sdf::ElementPtr fluidPluginSdf;

  /// \brief Publisher for fluid object visual messages.
  transport::PublisherPtr fluidObjPub;
  transport::PublisherPtr rigidObjPub;
  transport::NodePtr node;

  event::ConnectionPtr updateConnection;
  // counters
  unsigned int simulationSteps;
  unsigned int currentFluidModel = 0;
};
} // namespace gazebo
#endif
