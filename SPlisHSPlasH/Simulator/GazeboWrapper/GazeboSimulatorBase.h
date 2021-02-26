#ifndef __GazeboSimulatorBase_h__
#define __GazeboSimulatorBase_h__

#include "SPlisHSPlasH/Common.h"
#include "GazeboSceneLoader.h"
#include "SPlisHSPlasH/TimeStep.h"
#include "SPlisHSPlasH/FluidModel.h"
#include "ParameterObject.h"
#include "SPlisHSPlasH/BoundaryModel_Akinci2012.h"
#include "SPlisHSPlasH/BoundaryModel_Koschier2017.h"
#include "SPlisHSPlasH/BoundaryModel_Bender2019.h"
#include "SPlisHSPlasH/TriangleMesh.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "SPlisHSPlasH/StaticRigidBody.h"
#include <memory>

using namespace gazebo;
//using namespace Utilities;
namespace SPH
{
class GazeboSimulatorBase : public GenParam::ParameterObject
{
public:
	struct SimulationMethod
	{
		short simulationMethod = 0;
		TimeStep *simulation = NULL;
		FluidModel model;
	};

protected:
	Utilities::GazeboSceneLoader::Scene m_scene;
	virtual void initParameters();

	void initFluidData();
	void createFluidBlocks(std::map<std::string, unsigned int> &fluidIDs, std::vector<std::vector<Vector3r>> &fluidParticles, std::vector<std::vector<Vector3r>> &fluidVelocities);

public:
	GazeboSimulatorBase();
	virtual ~GazeboSimulatorBase();

	void readParameters();

	void createEmitters();
	void init(const sdf::ElementPtr &fluidSdf);
	void buildModel();
	void cleanup();
	void initDensityMap(std::vector<Vector3r> &x, std::vector<unsigned int> &faces, const Utilities::GazeboSceneLoader::GazeboBoundaryData *boundaryData, const bool md5, const bool isDynamic, BoundaryModel_Koschier2017 *boundaryModel);
	void initVolumeMap(std::vector<Vector3r> &x, std::vector<unsigned int> &faces, const Utilities::GazeboSceneLoader::GazeboBoundaryData *boundaryData, const bool md5, const bool isDynamic, BoundaryModel_Bender2019 *boundaryModel);
	void reset();
	void updateBoundaryParticles(const std::map<SPH::StaticRigidBody *, physics::CollisionPtr> &boundariesToCollisions, const bool forceUpdate);
	void updateDMVelocity();
	void updateVMVelocity();
	void updateBoundaryForces(const std::map<SPH::StaticRigidBody *, physics::CollisionPtr> &boundariesToCollisions);
	Utilities::GazeboSceneLoader *getSceneLoader() { return m_sceneLoader.get(); }
	std::string real2String(const Real r);
	static void loadObj(const std::string &filename, TriangleMesh &mesh, const Vector3r &scale);
	Utilities::GazeboSceneLoader::Scene &getScene() { return m_scene; }
	std::unique_ptr<Utilities::GazeboSceneLoader> m_sceneLoader;
	void processBoundary(const gazebo::physics::CollisionPtr &collision, const std::string &objFilePath);
};
} // namespace SPH

#endif