#include "GazeboSimulatorBase.h"
#include "GazeboFluidSimulator.h"
#include "SPlisHSPlasH/TimeManager.h"
#include "SPlisHSPlasH/Emitter.h"
#include "SPlisHSPlasH/EmitterSystem.h"
#include "SPlisHSPlasH/Simulation.h"
#include "SPlisHSPlasH/Vorticity/MicropolarModel_Bender2017.h"
#include "NumericParameter.h"
//#include "extern/partio/src/lib/Partio.h"
#include "SPlisHSPlasH/Utilities/GaussQuadrature.h"
#include "SPlisHSPlasH/Utilities/SimpleQuadrature.h"
#include "SPlisHSPlasH/Utilities/VolumeSampling.h"
#include "Utilities/OBJLoader.h"
#include "Utilities/BinaryFileReaderWriter.h"
#include "Utilities/Logger.h"
#include "Utilities/Timing.h"
#include "Utilities/Counting.h"
#include "Utilities/FileSystem.h"

using namespace SPH;
using namespace std;
using namespace GenParam;
using namespace Utilities;
using namespace gazebo;

//virtual void initParameters();

INIT_LOGGING
INIT_TIMING
INIT_COUNTING

void GazeboSimulatorBase::initParameters()
{
	ParameterObject::initParameters();
}
GazeboSimulatorBase::GazeboSimulatorBase()
{
	Utilities::logger.addSink(unique_ptr<Utilities::ConsoleSink>(new Utilities::ConsoleSink(Utilities::LogLevel::INFO)));
}

GazeboSimulatorBase::~GazeboSimulatorBase()
{
}

void GazeboSimulatorBase::readParameters()
{
	m_sceneLoader->readParameterObject("fluidConfiguration", this);
	m_sceneLoader->readParameterObject("fluidConfiguration", Simulation::getCurrent());
	m_sceneLoader->readParameterObject("fluidConfiguration", Simulation::getCurrent()->getTimeStep());

	Simulation *sim = Simulation::getCurrent();
	for (unsigned int i = 0; i < sim->numberOfFluidModels(); i++)
	{
		FluidModel *model = sim->getFluidModel(i);
		const std::string &key = model->getId();
		m_sceneLoader->readParameterObject(model->getId(), model);
		m_sceneLoader->readParameterObject(key, (ParameterObject *)model->getDragBase());
		m_sceneLoader->readParameterObject(key, (ParameterObject *)model->getSurfaceTensionBase());
		m_sceneLoader->readParameterObject(key, (ParameterObject *)model->getViscosityBase());
		m_sceneLoader->readParameterObject(key, (ParameterObject *)model->getVorticityBase());
		m_sceneLoader->readParameterObject(key, (ParameterObject *)model->getElasticityBase());
	}
}

void GazeboSimulatorBase::init(const sdf::ElementPtr &fluidSdf)
{
	initParameters();
	m_sceneLoader = std::make_unique<GazeboSceneLoader>();
	m_sceneLoader->readScene(fluidSdf, m_scene);
}

void GazeboSimulatorBase::processBoundary(const gazebo::physics::CollisionPtr &collision, const std::string &objFilePath)
{
	m_sceneLoader->processBoundary(m_scene, collision, objFilePath);
}

void GazeboSimulatorBase::cleanup()
{
	for (unsigned int i = 0; i < m_scene.boundaryModels.size(); i++)
		delete m_scene.boundaryModels[i];
	m_scene.boundaryModels.clear();

	for (unsigned int i = 0; i < m_scene.fluidModels.size(); i++)
		delete m_scene.fluidModels[i];
	m_scene.fluidModels.clear();

	for (unsigned int i = 0; i < m_scene.fluidBlocks.size(); i++)
		delete m_scene.fluidBlocks[i];
	m_scene.fluidBlocks.clear();

	for (unsigned int i = 0; i < m_scene.emitters.size(); i++)
		delete m_scene.emitters[i];
	m_scene.emitters.clear();
}

void GazeboSimulatorBase::buildModel()
{
	TimeManager::getCurrent()->setTimeStepSize(m_scene.timeStepSize);

	initFluidData();

	createEmitters();

	Simulation *sim = Simulation::getCurrent();

	if (sim->getTimeStep())
		sim->getTimeStep()->resize();

	sim->setValue(Simulation::KERNEL_METHOD, Simulation::ENUM_KERNEL_CUBIC_2D);
	sim->setValue(Simulation::GRAD_KERNEL_METHOD, Simulation::ENUM_GRADKERNEL_CUBIC_2D);
}

void GazeboSimulatorBase::loadObj(const std::string &filename, TriangleMesh &mesh, const Vector3r &scale)
{
	std::vector<OBJLoader::Vec3f> x;
	std::vector<OBJLoader::Vec3f> normals;
	std::vector<MeshFaceIndices> faces;
	OBJLoader::Vec3f s = {(float)scale[0], (float)scale[1], (float)scale[2]};
	OBJLoader::loadObj(filename, &x, &faces, &normals, nullptr, s);

	mesh.release();
	const unsigned int nPoints = (unsigned int)x.size();
	const unsigned int nFaces = (unsigned int)faces.size();
	mesh.initMesh(nPoints, nFaces);
	for (unsigned int i = 0; i < nPoints; i++)
	{
		mesh.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
	}
	for (unsigned int i = 0; i < nFaces; i++)
	{
		// Reduce the indices by one
		int posIndices[3];
		for (int j = 0; j < 3; j++)
		{
			posIndices[j] = faces[i].posIndices[j] - 1;
		}

		mesh.addFace(&posIndices[0]);
	}

	LOG_INFO << "Number of triangles: " << nFaces;
	LOG_INFO << "Number of vertices: " << nPoints;
}

void GazeboSimulatorBase::initFluidData()
{
	LOG_INFO << "Initialize fluid particles";

	Simulation *sim = Simulation::getCurrent();

	//////////////////////////////////////////////////////////////////////////
	// Determine number of different fluid IDs
	//////////////////////////////////////////////////////////////////////////
	std::map<std::string, unsigned int> fluidIDs;
	unsigned int index = 0;
	for (unsigned int i = 0; i < m_scene.fluidBlocks.size(); i++)
	{
		if (fluidIDs.find(m_scene.fluidBlocks[i]->id) == fluidIDs.end())
			fluidIDs[m_scene.fluidBlocks[i]->id] = index++;
	}
	for (unsigned int i = 0; i < m_scene.fluidModels.size(); i++)
	{
		if (fluidIDs.find(m_scene.fluidModels[i]->id) == fluidIDs.end())
			fluidIDs[m_scene.fluidModels[i]->id] = index++;
	}
	for (unsigned int i = 0; i < m_scene.emitters.size(); i++)
	{
		if (fluidIDs.find(m_scene.emitters[i]->id) == fluidIDs.end())
			fluidIDs[m_scene.emitters[i]->id] = index++;
	}
	const unsigned int numberOfFluidModels = static_cast<unsigned int>(fluidIDs.size());

	std::vector<std::vector<Vector3r>> fluidParticles;
	std::vector<std::vector<Vector3r>> fluidVelocities;
	fluidParticles.resize(numberOfFluidModels);
	fluidVelocities.resize(numberOfFluidModels);

	createFluidBlocks(fluidIDs, fluidParticles, fluidVelocities);

	for (unsigned int i = 0; i < m_scene.fluidModels.size(); i++)
	{
		const unsigned int fluidIndex = fluidIDs[m_scene.fluidModels[i]->id];
		fluidVelocities[fluidIndex].resize(fluidParticles[fluidIndex].size(), m_scene.fluidModels[i]->initialVelocity);

		// transform particles
		for (unsigned int j = 0; j < (unsigned int)fluidParticles[fluidIndex].size(); j++)
			fluidParticles[fluidIndex][j] = m_scene.fluidModels[i]->rotation * fluidParticles[fluidIndex][j] + m_scene.fluidModels[i]->translation;
		Simulation::getCurrent()->setValue(Simulation::PARTICLE_RADIUS, m_scene.particleRadius);
	}

	unsigned int nParticles = 0;
	for (auto it = fluidIDs.begin(); it != fluidIDs.end(); it++)
	{
		const unsigned int index = it->second;

		unsigned int maxEmitterParticles = 10000000;
		//m_sceneLoader->readValue(it->first, "maxEmitterParticles", maxEmitterParticles);
		sim->addFluidModel(it->first, (unsigned int)fluidParticles[index].size(), fluidParticles[index].data(), fluidVelocities[index].data(), maxEmitterParticles);
		nParticles += (unsigned int)fluidParticles[index].size();
	}

	LOG_INFO << "Number of fluid particles: " << nParticles;
}

void GazeboSimulatorBase::createEmitters()
{
	Simulation *sim = Simulation::getCurrent();

	//////////////////////////////////////////////////////////////////////////
	// emitters
	//////////////////////////////////////////////////////////////////////////
	for (unsigned int i = 0; i < m_scene.emitters.size(); i++)
	{
		//GazeboSceneLoader::EmitterData *ed = m_scene.emitters[i];
		SceneLoader::EmitterData *ed = m_scene.emitters[i]; // TODO

		FluidModel *model = nullptr;
		unsigned int j;
		for (j = 0; j < sim->numberOfFluidModels(); j++)
		{
			model = sim->getFluidModel(j);
			if (model->getId() == ed->id)
				break;
		}

		if (j < sim->numberOfFluidModels())
		{
			model->getEmitterSystem()->addEmitter(
				ed->width, ed->height,
				ed->x, ed->rotation,
				ed->velocity,
				ed->type);

			// Generate boundary geometry around emitters
			Emitter *emitter = model->getEmitterSystem()->getEmitters().back();
			SceneLoader::BoundaryData *emitterBoundary = new SceneLoader::BoundaryData();
			emitterBoundary->dynamic = false;
			emitterBoundary->isWall = false;
			emitterBoundary->rotation = ed->rotation;
			const Real supportRadius = sim->getSupportRadius();
			const Vector3r &emitDir = ed->rotation.col(0);
			emitterBoundary->scale = Emitter::getSize(ed->width, ed->height, ed->type);
			const Vector3r pos = ed->x;
			emitterBoundary->translation = pos;
			emitterBoundary->samplesFile = "";
			emitterBoundary->mapInvert = false;
			emitterBoundary->mapResolution = Eigen::Matrix<unsigned int, 3, 1>(20, 20, 20);
			emitterBoundary->mapThickness = 0.0;
			emitter->setEmitStartTime(ed->emitStartTime);

			/* 			if (ed->type == 0)
				emitterBoundary->meshFile = FileSystem::normalizePath(getDataPath() + "/models/EmitterBox.obj");
			else if (ed->type == 1)
				emitterBoundary->meshFile = FileSystem::normalizePath(getDataPath() + "/models/EmitterCylinder.obj");
			m_scene.boundaryModels.push_back(emitterBoundary);

			// reuse particles if they are outside of a bounding box
			bool emitterReuseParticles = false;
			m_sceneLoader->getSDFParameter(model->getId(), "emitterReuseParticles", emitterReuseParticles);
			// TODO whats the point of this

			if (emitterReuseParticles)
			{
				// boxMin
				Vector3r emitterBoxMin(-1.0, -1.0, -1.0);
				m_sceneLoader->readVector(model->getId(), "emitterBoxMin", emitterBoxMin);

				// boxMax
				Vector3r emitterBoxMax(1.0, 1.0, 1.0);
				m_sceneLoader->readVector(model->getId(), "emitterBoxMax", emitterBoxMax);

				model->getEmitterSystem()->enableReuseParticles(emitterBoxMin, emitterBoxMax);
			}
			emitter->setEmitStartTime(ed->emitStartTime);
			emitter->setEmitEndTime(ed->emitEndTime); */
		}
	}
}

void GazeboSimulatorBase::createFluidBlocks(std::map<std::string, unsigned int> &fluidIDs, std::vector<std::vector<Vector3r>> &fluidParticles, std::vector<std::vector<Vector3r>> &fluidVelocities)
{
	for (unsigned int i = 0; i < m_scene.fluidBlocks.size(); i++)
	{
		const unsigned int fluidIndex = fluidIDs[m_scene.fluidBlocks[i]->id];
		const Real diam = static_cast<Real>(2.0) * m_scene.particleRadius;

		Real xshift = diam;
		Real yshift = diam;
		const Real eps = static_cast<Real>(1.0e-9);
		if (m_scene.fluidBlocks[i]->mode == 1)
			yshift = sqrt(static_cast<Real>(3.0)) * m_scene.particleRadius + eps;
		else if (m_scene.fluidBlocks[i]->mode == 2)
		{
			xshift = sqrt(static_cast<Real>(6.0)) * diam / static_cast<Real>(3.0) + eps;
			yshift = sqrt(static_cast<Real>(3.0)) * m_scene.particleRadius + eps;
		}

		Vector3r diff = m_scene.fluidBlocks[i]->box.m_maxX - m_scene.fluidBlocks[i]->box.m_minX;
		if (m_scene.fluidBlocks[i]->mode == 1)
		{
			diff[0] -= diam;
			diff[2] -= diam;
		}
		else if (m_scene.fluidBlocks[i]->mode == 2)
		{
			diff[0] -= xshift;
			diff[2] -= diam;
		}

		const int stepsX = (int)round(diff[0] / xshift) - 1;
		const int stepsY = (int)round(diff[1] / yshift) - 1;
		int stepsZ = (int)round(diff[2] / diam) - 1;

		Vector3r start = m_scene.fluidBlocks[i]->box.m_minX + static_cast<Real>(2.0) * m_scene.particleRadius * Vector3r::Ones();
		fluidParticles[fluidIndex].reserve(fluidParticles[fluidIndex].size() + stepsX * stepsY * stepsZ);
		fluidVelocities[fluidIndex].resize(fluidVelocities[fluidIndex].size() + stepsX * stepsY * stepsZ, m_scene.fluidBlocks[i]->initialVelocity);

		for (int j = 0; j < stepsX; j++)
		{
			for (int k = 0; k < stepsY; k++)
			{
				for (int l = 0; l < stepsZ; l++)
				{
					Vector3r currPos = Vector3r(j * xshift, k * yshift, l * diam) + start;
					if (m_scene.fluidBlocks[i]->mode == 1)
					{
						if (k % 2 == 0)
							currPos += Vector3r(0, 0, m_scene.particleRadius);
						else
							currPos += Vector3r(m_scene.particleRadius, 0, 0);
					}
					else if (m_scene.fluidBlocks[i]->mode == 2)
					{
						currPos += Vector3r(0, 0, m_scene.particleRadius);

						Vector3r shift_vec(0, 0, 0);
						if (k % 2 == 0)
						{
							shift_vec[0] += xshift / static_cast<Real>(2.0);
						}
						currPos += shift_vec;
					}
					fluidParticles[fluidIndex].push_back(currPos);
				}
			}
		}
	}
}

void GazeboSimulatorBase::reset()
{
	if (Simulation::getCurrent()->getValue<int>(Simulation::CFL_METHOD) != Simulation::ENUM_CFL_NONE)
		TimeManager::getCurrent()->setTimeStepSize(m_scene.timeStepSize);
#ifdef DL_OUTPUT
	m_nextTiming = 1.0;
#endif
}

void GazeboSimulatorBase::updateBoundaryParticles(const std::map<SPH::StaticRigidBody *, physics::CollisionPtr> &boundariesToCollisions, const bool forceUpdate = false)
{
	Simulation *sim = Simulation::getCurrent();
	GazeboSceneLoader::Scene &scene = getScene();

	const unsigned int nObjects = sim->numberOfBoundaryModels();

	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel_Akinci2012 *bm = static_cast<BoundaryModel_Akinci2012 *>(sim->getBoundaryModel(i));
		StaticRigidBody *rbo = dynamic_cast<StaticRigidBody *>(bm->getRigidBodyObject());
		if (rbo->isDynamic() || forceUpdate)
		{
			//#pragma omp parallel default(shared)
			{
				//#pragma omp for schedule(static)
				physics::CollisionPtr currentRigidBody = scene.boundaryModels[i]->rigidBody;
				physics::LinkPtr gazeboBodyLink = currentRigidBody->GetLink();

				// Position of rigid body
				ignition::math::Pose3d gazeboRigidBodyPose = currentRigidBody->WorldPose();
				Vector3r fluidObjectPosition = Vector3r(gazeboRigidBodyPose.Pos().X(), gazeboRigidBodyPose.Pos().Y(), gazeboRigidBodyPose.Pos().Z());

				// Rotation of rigid body
				ignition::math::Matrix3d rigidBodyRotation = ignition::math::Matrix3d(gazeboRigidBodyPose.Rot());
				Matrix3r fluidObjectRotation;
				fluidObjectRotation << rigidBodyRotation(0, 0), rigidBodyRotation(0, 1), rigidBodyRotation(0, 2),
					rigidBodyRotation(1, 0), rigidBodyRotation(1, 1), rigidBodyRotation(1, 2),
					rigidBodyRotation(2, 0), rigidBodyRotation(2, 1), rigidBodyRotation(2, 2);

				// Linear velocity of rigid body
				ignition::math::Vector3d linearVelocityGazebo = currentRigidBody->WorldLinearVel();
				Vector3r fluidObjectLinearVel = Vector3r(linearVelocityGazebo.X(), linearVelocityGazebo.Y(), linearVelocityGazebo.Z());

				// Angular velocity of rigid body
				ignition::math::Vector3d angularVelocityGazebo = currentRigidBody->WorldAngularVel();
				Vector3r fluidObjectAngularVel = Vector3r(angularVelocityGazebo.X(), angularVelocityGazebo.Y(), angularVelocityGazebo.Z());

				for (int j = 0; j < (int)bm->numberOfParticles(); j++)
				{
					bm->getPosition(j) = fluidObjectRotation * bm->getPosition0(j) + fluidObjectPosition;
					if (rbo->isDynamic())
						bm->getVelocity(j) = fluidObjectAngularVel.cross(bm->getPosition(j) - fluidObjectPosition) + fluidObjectLinearVel;
					else
						bm->getVelocity(j).setZero();
				}
			}
#ifdef GPU_NEIGHBORHOOD_SEARCH
			// copy the particle data to the GPU
			if (forceUpdate)
				sim->getNeighborhoodSearch()->update_point_sets();
#endif
		}
	}
}

void SPH::GazeboSimulatorBase::updateDMVelocity()
{
	Simulation *sim = Simulation::getCurrent();
	GazeboSceneLoader::Scene &scene = getScene();
	const unsigned int nObjects = sim->numberOfBoundaryModels();
	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel_Koschier2017 *bm = static_cast<BoundaryModel_Koschier2017 *>(sim->getBoundaryModel(i));
		StaticRigidBody *rbo = dynamic_cast<StaticRigidBody *>(bm->getRigidBodyObject());
		if (rbo->isDynamic())
		{

			physics::CollisionPtr currentRigidBody = scene.boundaryModels[i]->rigidBody;
			physics::LinkPtr gazeboBodyLink = currentRigidBody->GetLink();

			// Linear velocity of rigid body
			ignition::math::Vector3d linearVelocityGazebo = gazeboBodyLink->WorldLinearVel();
			Vector3r fluidObjectLinearVel = Vector3r(linearVelocityGazebo.X(), linearVelocityGazebo.Y(), linearVelocityGazebo.Z());

			// Angular velocity of rigid body
			ignition::math::Vector3d angularVelocityGazebo = gazeboBodyLink->WorldAngularVel();
			Vector3r fluidObjectAngularVel = Vector3r(angularVelocityGazebo.X(), angularVelocityGazebo.Y(), angularVelocityGazebo.Z());

			const Real maxDist = bm->getMaxDist();
			const Vector3r x(maxDist, 0.0, 0.0);
			const Vector3r vel = fluidObjectAngularVel.cross(x) + fluidObjectLinearVel;
			bm->setMaxVel(vel.norm());
		}
	}
}

void SPH::GazeboSimulatorBase::updateVMVelocity()
{
	Simulation *sim = Simulation::getCurrent();
	GazeboSceneLoader::Scene &scene = getScene();
	const unsigned int nObjects = sim->numberOfBoundaryModels();
	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel_Bender2019 *bm = static_cast<BoundaryModel_Bender2019 *>(sim->getBoundaryModel(i));
		StaticRigidBody *rbo = dynamic_cast<StaticRigidBody *>(bm->getRigidBodyObject());
		if (rbo->isDynamic())
		{
			physics::CollisionPtr currentRigidBody = scene.boundaryModels[i]->rigidBody;
			physics::LinkPtr gazeboBodyLink = currentRigidBody->GetLink();

			// Linear velocity of rigid body
			ignition::math::Vector3d linearVelocityGazebo = gazeboBodyLink->WorldLinearVel();
			Vector3r fluidObjectLinearVel = Vector3r(linearVelocityGazebo.X(), linearVelocityGazebo.Y(), linearVelocityGazebo.Z());

			// Angular velocity of rigid body
			ignition::math::Vector3d angularVelocityGazebo = gazeboBodyLink->WorldAngularVel();
			Vector3r fluidObjectAngularVel = Vector3r(angularVelocityGazebo.X(), angularVelocityGazebo.Y(), angularVelocityGazebo.Z());

			const Real maxDist = bm->getMaxDist();
			const Vector3r x(maxDist, 0.0, 0.0);
			const Vector3r vel = fluidObjectAngularVel.cross(x) + fluidObjectLinearVel;
			bm->setMaxVel(vel.norm());
		}
	}
}

void GazeboSimulatorBase::updateBoundaryForces(const std::map<SPH::StaticRigidBody *, physics::CollisionPtr> &boundariesToCollisions)
{
	Real h = TimeManager::getCurrent()->getTimeStepSize();
	Simulation *sim = Simulation::getCurrent();
	GazeboSceneLoader::Scene &scene = getScene();
	const unsigned int nObjects = sim->numberOfBoundaryModels();
	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel *bm = sim->getBoundaryModel(i);
		StaticRigidBody *rbo = dynamic_cast<StaticRigidBody *>(bm->getRigidBodyObject());
		if (rbo->isDynamic())
		{
			gazebo::physics::CollisionPtr gazeboRigidBody = scene.boundaryModels[i]->rigidBody;

			Vector3r force, torque;
			// change rigid body fluid position?
			bm->getForceAndTorque(force, torque);
			const ignition::math::Vector3d gazeboForce = ignition::math::Vector3d(force[0], force[1], force[2]);
			const ignition::math::Vector3d gazeboTorque = ignition::math::Vector3d(torque[0], torque[1], torque[2]);
			gazeboRigidBody->GetLink()->AddForceAtRelativePosition(gazeboForce, ignition::math::Vector3d(0.0, 0.0, 0.0));
			//gazeboRigidBody->GetLink()->AddForce(gazeboForce);
			//gazeboRigidBody->GetLink()->AddTorque(gazeboTorque);
			bm->clearForceAndTorque();
		}
	}
}

std::string GazeboSimulatorBase::real2String(const Real r)
{
	string str = to_string(r);
	str.erase(str.find_last_not_of('0') + 1, std::string::npos);
	str.erase(str.find_last_not_of('.') + 1, std::string::npos);
	return str;
}

void GazeboSimulatorBase::initDensityMap(std::vector<Vector3r> &x, std::vector<unsigned int> &faces, const Utilities::GazeboSceneLoader::GazeboBoundaryData *boundaryData, const bool md5, const bool isDynamic, BoundaryModel_Koschier2017 *boundaryModel)
{
	Simulation *sim = Simulation::getCurrent();
	const Real supportRadius = sim->getSupportRadius();
	GazeboSceneLoader::Scene &scene = getScene();
	Discregrid::CubicLagrangeDiscreteGrid *densityMap;
	Eigen::Matrix<unsigned int, 3, 1> resolutionSDF = boundaryData->mapResolution;

#ifdef USE_DOUBLE
	Discregrid::TriangleMesh sdfMesh(&x[0][0], faces.data(), x.size(), faces.size() / 3);
#else
	// if type is float, copy vector to double vector
	std::vector<double> doubleVec;
	doubleVec.resize(3 * x.size());
	for (unsigned int i = 0; i < x.size(); i++)
		for (unsigned int j = 0; j < 3; j++)
			doubleVec[3 * i + j] = x[i][j];
	Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), x.size(), faces.size() / 3);
#endif

	Discregrid::MeshDistance md(sdfMesh);
	Eigen::AlignedBox3d domain;
	for (auto const &x_ : x)
	{
		domain.extend(x_.cast<double>());
	}
	const Real tolerance = boundaryData->mapThickness;
	domain.max() += (4.0 * supportRadius + tolerance) * Eigen::Vector3d::Ones();
	domain.min() -= (4.0 * supportRadius + tolerance) * Eigen::Vector3d::Ones();

	LOG_INFO << "Domain - min: " << domain.min()[0] << ", " << domain.min()[1] << ", " << domain.min()[2];
	LOG_INFO << "Domain - max: " << domain.max()[0] << ", " << domain.max()[1] << ", " << domain.max()[2];

	LOG_INFO << "Set SDF resolution: " << resolutionSDF[0] << ", " << resolutionSDF[1] << ", " << resolutionSDF[2];
	densityMap = new Discregrid::CubicLagrangeDiscreteGrid(domain, std::array<unsigned int, 3>({resolutionSDF[0], resolutionSDF[1], resolutionSDF[2]}));
	auto func = Discregrid::DiscreteGrid::ContinuousFunction{};

	Real sign = 1.0;
	if (boundaryData->mapInvert)
		sign = -1.0;
	func = [&md, &sign, &tolerance](Eigen::Vector3d const &xi) { return sign * (md.signedDistanceCached(xi) - tolerance); };

	LOG_INFO << "Generate SDF";
	START_TIMING("SDF Construction");
	densityMap->addFunction(func, false);
	STOP_TIMING_AVG

	auto int_domain = Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-supportRadius), Eigen::Vector3d::Constant(supportRadius));
	Real factor = 5.0;
	auto density_func = [&](Eigen::Vector3d const &x) {
		auto d = densityMap->interpolate(0u, x);
		if (d > (1.0 + 1.0 / factor) * supportRadius)
		{
			return 0.0;
		}

		auto integrand = [&](Eigen::Vector3d const &xi) {
			if (xi.squaredNorm() > supportRadius * supportRadius)
				return 0.0;

			auto dist = densityMap->interpolate(0u, x + xi);

			// Linear function gamma
			if (dist > 1.0 / factor * supportRadius)
				return 0.0;
			return static_cast<double>((1.0 - factor * dist / supportRadius) * sim->W(xi.cast<Real>()));
		};

		return 0.8 * GaussQuadrature::integrate(integrand, int_domain, 50);
	};

	auto cell_diag = densityMap->cellSize().norm();
	std::cout << "Generate density map..." << std::endl;
	const bool no_reduction = true;
	START_TIMING("Density Map Construction");
	densityMap->addFunction(density_func, false, [&](Eigen::Vector3d const &x_) {
		if (no_reduction)
		{
			return true;
		}
		auto x = x_.cwiseMax(densityMap->domain().min()).cwiseMin(densityMap->domain().max());
		auto dist = densityMap->interpolate(0u, x);
		if (dist == std::numeric_limits<double>::max())
		{
			return false;
		}

		return -6.0 * supportRadius < dist + cell_diag && dist - cell_diag < 2.0 * supportRadius;
	});
	STOP_TIMING_PRINT;

	// reduction
	if (!no_reduction)
	{
		std::cout << "Reduce discrete fields...";
		densityMap->reduceField(0u, [&](Eigen::Vector3d const &, double v) -> double {
			return 0.0 <= v && v <= 3.0;
		});
		std::cout << "DONE" << std::endl;
	}

	boundaryModel->setMap(densityMap);
}

void GazeboSimulatorBase::initVolumeMap(std::vector<Vector3r> &x, std::vector<unsigned int> &faces, const Utilities::GazeboSceneLoader::GazeboBoundaryData *boundaryData, const bool md5, const bool isDynamic, BoundaryModel_Bender2019 *boundaryModel)
{
	Simulation *sim = Simulation::getCurrent();
	const Real supportRadius = sim->getSupportRadius();
	GazeboSceneLoader::Scene &scene = getScene();
	Discregrid::CubicLagrangeDiscreteGrid *volumeMap;

	Eigen::Matrix<unsigned int, 3, 1> resolutionSDF = boundaryData->mapResolution;

	//////////////////////////////////////////////////////////////////////////
	// Generate distance field of object using Discregrid
	//////////////////////////////////////////////////////////////////////////
#ifdef USE_DOUBLE
	Discregrid::TriangleMesh sdfMesh(&x[0][0], faces.data(), x.size(), faces.size() / 3);
#else
	// if type is float, copy vector to double vector
	std::vector<double> doubleVec;
	doubleVec.resize(3 * x.size());
	for (unsigned int i = 0; i < x.size(); i++)
		for (unsigned int j = 0; j < 3; j++)
			doubleVec[3 * i + j] = x[i][j];
	Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), x.size(), faces.size() / 3);
#endif

	Discregrid::MeshDistance md(sdfMesh);
	Eigen::AlignedBox3d domain;
	for (auto const &x_ : x)
	{
		domain.extend(x_.cast<double>());
	}
	const Real tolerance = boundaryData->mapThickness;
	domain.max() += (4.0 * supportRadius + tolerance) * Eigen::Vector3d::Ones();
	domain.min() -= (4.0 * supportRadius + tolerance) * Eigen::Vector3d::Ones();

	LOG_INFO << "Domain - min: " << domain.min()[0] << ", " << domain.min()[1] << ", " << domain.min()[2];
	LOG_INFO << "Domain - max: " << domain.max()[0] << ", " << domain.max()[1] << ", " << domain.max()[2];

	LOG_INFO << "Set SDF resolution: " << resolutionSDF[0] << ", " << resolutionSDF[1] << ", " << resolutionSDF[2];
	volumeMap = new Discregrid::CubicLagrangeDiscreteGrid(domain, std::array<unsigned int, 3>({resolutionSDF[0], resolutionSDF[1], resolutionSDF[2]}));
	auto func = Discregrid::DiscreteGrid::ContinuousFunction{};

	//volumeMap->setErrorTolerance(0.001);

	Real sign = 1.0;
	if (boundaryData->mapInvert)
		sign = -1.0;
	const Real particleRadius = sim->getParticleRadius();
	// subtract 0.5 * particle radius to prevent penetration of particles and the boundary
	func = [&md, &sign, &tolerance, &particleRadius](Eigen::Vector3d const &xi) { return sign * (md.signedDistanceCached(xi) - tolerance - 0.5 * particleRadius); };

	LOG_INFO << "Generate SDF";
	START_TIMING("SDF Construction");
	volumeMap->addFunction(func, false);
	STOP_TIMING_PRINT

	//////////////////////////////////////////////////////////////////////////
	// Generate volume map of object using Discregrid
	//////////////////////////////////////////////////////////////////////////

	auto int_domain = Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-supportRadius), Eigen::Vector3d::Constant(supportRadius));
	Real factor = 1.0;
	auto volume_func = [&](Eigen::Vector3d const &x) {
		auto dist = volumeMap->interpolate(0u, x);
		if (dist > (1.0 + 1.0 / factor) * supportRadius)
		{
			return 0.0;
		}

		auto integrand = [&volumeMap, &x, &supportRadius, &factor, &sim](Eigen::Vector3d const &xi) -> double {
			if (xi.squaredNorm() > supportRadius * supportRadius)
				return 0.0;

			auto dist = volumeMap->interpolate(0u, x + xi);

			if (dist <= 0.0)
				return 1.0 - 0.1 * dist / supportRadius;
			if (dist < 1.0 / factor * supportRadius)
				return static_cast<double>(CubicKernel::W(factor * static_cast<Real>(dist)) / CubicKernel::W_zero());
			return 0.0;
		};

		return 0.8 * GaussQuadrature::integrate(integrand, int_domain, 30);
	};

	auto cell_diag = volumeMap->cellSize().norm();
	std::cout << "Generate volume map..." << std::endl;
	const bool no_reduction = true;
	START_TIMING("Volume Map Construction");
	volumeMap->addFunction(volume_func, false, [&](Eigen::Vector3d const &x_) {
		if (no_reduction)
		{
			return true;
		}
		auto x = x_.cwiseMax(volumeMap->domain().min()).cwiseMin(volumeMap->domain().max());
		auto dist = volumeMap->interpolate(0u, x);
		if (dist == std::numeric_limits<double>::max())
		{
			return false;
		}

		return -6.0 * supportRadius < dist + cell_diag && dist - cell_diag < 2.0 * supportRadius;
	});
	STOP_TIMING_PRINT;

	// reduction
	if (!no_reduction)
	{
		std::cout << "Reduce discrete fields...";
		volumeMap->reduceField(0u, [&](Eigen::Vector3d const &, double v) -> double {
			return 0.0 <= v && v <= 3.0;
		});
		std::cout << "DONE" << std::endl;
	}

	boundaryModel->setMap(volumeMap);

	// store maximal distance of a point to center of mass for CFL
	if (boundaryData->dynamic)
	{
		// determine center of mass
		Vector3r com;
		com.setZero();
		for (unsigned int i = 0; i < x.size(); i++)
		{
			com += x[i];
		}
		com /= static_cast<Real>(x.size());

		// determine point with maximal distance to center of mass
		Real maxDist = 0.0;
		for (unsigned int i = 0; i < x.size(); i++)
		{
			const Vector3r diff = x[i] - com;
			const Real dist = diff.norm();
			if (dist > maxDist)
			{
				maxDist = dist;
			}
		}
		boundaryModel->setMaxDist(maxDist);
	}
}
