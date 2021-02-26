#include "SPlisHSPlasH/Common.h"
#include "SPlisHSPlasH/TimeManager.h"
#include <Eigen/Dense>
#include <iostream>
#include "Utilities/OBJLoader.h"
#include "SPlisHSPlasH/Utilities/SurfaceSampling.h"
#include <fstream>
#include "SPlisHSPlasH/Simulation.h"
#include "SPlisHSPlasH/BoundaryModel_Koschier2017.h"
#include "SPlisHSPlasH/BoundaryModel_Bender2019.h"
#include "SPlisHSPlasH/BoundaryModel_Akinci2012.h"
#include "GazeboFluidSimulator.h"
#include "Utilities/Timing.h"
#include "Utilities/Counting.h"
#include "Utilities/FileSystem.h"
#include <ignition/math/Vector3.hh>
#include <memory>
#include <gazebo/physics/physics.hh>
#include "SPlisHSPlasH/Emitter.h"
#include "SPlisHSPlasH/EmitterSystem.h"
#include <functional>
#include <gazebo/common/common.hh>
#include <chrono>

// Enable memory leak detection
#ifdef _DEBUG
#ifndef EIGEN_ALIGN
#define new DEBUG_NEW
#endif
#endif

using namespace SPH;
using namespace Eigen;
using namespace std;
using namespace Utilities;
using namespace GenParam;
using namespace gazebo;


const std::string objFilePath("/tmp/");

FluidSimulator::FluidSimulator()
{
	REPORT_MEMORY_LEAKS;
	std::cout << "Plugin loaded" << std::endl;
}


void FluidSimulator::RunStep()
{
        auto start = std::chrono::high_resolution_clock::now();
	simulationSteps++;
	Simulation *sim = Simulation::getCurrent();
	if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Akinci2012)
		base->updateBoundaryParticles(boundariesToCollisions, false);
	else if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Koschier2017)
		base->updateDMVelocity();
	else if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Bender2019)
		base->updateVMVelocity();
	START_TIMING("SimStep");
	Simulation::getCurrent()->getTimeStep()->step();
	STOP_TIMING_AVG;
	this->world->Physics()->SetMaxStepSize(TimeManager::getCurrent()->getTimeStepSize());
	INCREASE_COUNTER("Time step size", TimeManager::getCurrent()->getTimeStepSize());
	base->updateBoundaryForces(boundariesToCollisions);

	//get model pos
	TimeManager *get= TimeManager::getCurrent();
	Real t = get->getTime();
	double xi,yj,zk;
	physics::ModelPtr model2 = this->world->ModelByName("robot");
	//physics::ModelPtr model2 = this->world->ModelByName("turtlebot3_burger");
	//physics::LinkPtr targetLink = model2->GetLink("gripper_link");
	physics::LinkPtr targetLink = model2->GetLink("wrist_3_link");
	//physics::ModelPtr model2 = this->world->ModelByName("modelName");
	ignition::math::Pose3d pose;     
	//pose = model2->WorldPose();
	pose = targetLink->WorldPose();
	ignition::math::Vector3<double> position = pose.Pos();
	xi = position.X(); // x coordinate
	yj = position.Y(); // y coordinate
	zk = position.Z(); // z coordinate	
	//std::cout << ' ' << xi << ' ' << yj <<  ' ' << zk << std::endl;

	FluidModel *model = sim->getFluidModel(0);
	Emitter *emitter = model->getEmitterSystem()->getEmitters().back();
	emitter->move_emitter(Vector3r(xi,yj,zk-0.01));

	//this->model = physics::ModelPtr parent;

	//this->model2->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

	//get particles position
	/*if (simulationSteps % 2000 == 0) {
		std::ofstream file("frame_" + std::to_string(simulationSteps) + ".xyz");
		if (file) {
			for (unsigned int i = 0; i < model->numActiveParticles(); i++){
				Vector3r &particlepos = model->getPosition(i);
				//file << particlepos;
				file << particlepos[0] << " " << particlepos[1] << " " << particlepos[2]<< "\n" ;
			}
			file.close();
		}
	}*/

	/*for (unsigned int i = 0; i < model->numActiveParticles(); ++i) {
		model->setParticleState(i, ParticleState::AnimatedByEmitter);
	}*/

	if (simulationSteps % 5 == 0)
	{
		msgs::Fluid fluid_positions_msg;
		fluid_positions_msg.set_name("fluid_positions");
		FluidModel *model = Simulation::getCurrent()->getFluidModel(0);
		for (unsigned int i = 0; i < model->numActiveParticles(); ++i)
		{
			gazebo::msgs::Set(fluid_positions_msg.add_position(),
							  ignition::math::Vector3d(
								  model->getPosition(i)[0],
								  model->getPosition(i)[1],
								  model->getPosition(i)[2]));
		}
		this->fluidObjPub->Publish(fluid_positions_msg);
	}

	// publish all the boundary particles positions
	msgs::Fluid boundary_particles_msg;
	boundary_particles_msg.set_name("boundary_particles");
	for (int i = 0; i < SPH::Simulation::getCurrent()->numberOfBoundaryModels(); ++i)
	{
		BoundaryModel_Akinci2012 *bm = static_cast<BoundaryModel_Akinci2012 *>(SPH::Simulation::getCurrent()->getBoundaryModel(i));
		for (int j = 0; j < (int)bm->numberOfParticles(); j++)
		{
			ignition::math::Vector3d boundary_particles = ignition::math::Vector3d(
				bm->getPosition(j)[0],
				bm->getPosition(j)[1],
				bm->getPosition(j)[2]);
			gazebo::msgs::Set(boundary_particles_msg.add_position(), boundary_particles);
		}
	}
	this->rigidObjPub->Publish(boundary_particles_msg);

/*if (simulationSteps % 1000 == 0) {
     	const string fileName = "0frame_" + std::to_string(simulationSteps) + ".xyz";
	 	const string path="/home/jason/fluid-engine-dev/build/bin/hybrid_liquid_sim_output/SPlisHSPlasH_out/"+fileName;   //determine where you are saving the files in

	 	std::ofstream outFile;
	 	outFile.open(path, ios::out);

	 	if (outFile.fail()){

	 		cerr << "Error opening the output file, check path  \n";
	 		exit(6);
	 	}

	 	else{

	 		for (unsigned int i=0; i< model -> numActiveParticles(); i++){
	 			Vector3r &particlepos = model->getPosition(i);
	 			outFile << particlepos[0] << " " << particlepos[1] << " " << particlepos[2]<< " "<<"\n";
	 		}
	 	}

 	string command="'/home/jason/fluid-engine-dev/build/bin/particles2obj' -i '"+path+"' -r 200,200,100 -k 0.05 -m spherical -o '/home/jason/fluid-engine-dev/build/bin/hybrid_liquid_sim_output/louis_test_"+std::to_string(simulationSteps)+".obj'";

	 	outFile.close();
	 	system((command).c_str());
*/
		/*#pragma region keep two models
		RemoveModel(name2);
		name2 = name1;
		name1 = to_string(simulationSteps);
		ImportModel(name1, 0.42); // 0.42 is scale size
		#pragma endregion
		*/
		
/*		#pragma region keep one model
		RemoveModel(name1);
		name1 = to_string(simulationSteps);
		ImportModel(name1,1);
		#pragma endregion
		

	 }

    auto finish = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start); 
	if (simulationSteps % 100 == 0) {
		std::ofstream outfile;
		outfile.open("/home/jason/splisplash/timeDuration.txt", std::ios::out | std::ios::app);
		outfile << duration.count() << std::endl;
	}
 */       
}

void FluidSimulator::ImportModel(string name, float size)
{

	double xi,yj,zk;
	physics::ModelPtr model2 = this->world->ModelByName("robot");

	physics::LinkPtr targetLink = model2->GetLink("gripper_link");

	ignition::math::Pose3d pose;     

	ignition::math::Vector3<double> position = pose.Pos();
    
	pose = targetLink->WorldPose();

	xi = position.X(); // x coordinate
	yj = position.Y(); // y coordinate
	zk = position.Z(); // z coordinate	
	string x=to_string(xi);
	string y=to_string(yj);
	string z=to_string(zk);

	// string x="0";
	// string y="0";
	// string z="0";
	string yaw="0";
	string roll="0";
	string pitch = "0";

	//string mesh_pos = "<pose>"+x+y+'0'+yaw+pitch+roll+"</pose>\n";
	string mesh_pos = "<pose>" + x + " " + y + " " + z + " "+ yaw + " " + pitch + " " + roll + "</pose>\n";
	string size_str = "<scale>" + to_string(size) + " " + to_string(size) + " " + to_string(size) + "</scale>\n";

		std::ofstream myfile;

		myfile.open ("/home/jason/fluid-engine-dev/build/bin/hybrid_liquid_sim_output/louis_test_1.sdf");
		myfile << "<?xml version='1.0'?>\n";
		myfile << "<sdf version ='1.6'>\n";
		myfile << "<model name='my_mesh'>\n";
		myfile << "<static>true</static>\n";
		myfile << "<link name='body'>\n";
		myfile << mesh_pos;
		myfile << "<visual name='visual'>\n";
		myfile << "<geometry>\n";
		myfile << "<mesh>\n";
		myfile << "<uri>/home/jason/fluid-engine-dev/build/bin/hybrid_liquid_sim_output/louis_test_" + std::to_string(simulationSteps) + ".obj</uri>\n";
		myfile << size_str;
		myfile << "</mesh>\n";
		myfile << "</geometry>\n";
		myfile << "</visual>\n";
		myfile << "</link>\n";
		myfile << "</model>\n";
		myfile << "</sdf>";
		myfile.close();

	string command = "gz model -m " + name + " -f '/home/jason/fluid-engine-dev/build/bin/hybrid_liquid_sim_output/louis_test_1.sdf'";

	system(command.c_str());
} 

void FluidSimulator::RemoveModel(string name)
{

	string command = "gz model -m " + name + " -d";
	system(command.c_str());
}

FluidSimulator::~FluidSimulator()
{
	this->connections.clear();
	base->cleanup();
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	Utilities::Counting::printAverageCounts();
	Utilities::Counting::printCounterSums();

	delete Simulation::getCurrent();
}

void FluidSimulator::Init()
{
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->world->Name());

	this->fluidObjPub = this->node->Advertise<msgs::Fluid>("~/fluid_pos", 10);
	this->rigidObjPub = this->node->Advertise<msgs::Fluid>("~/rigids_pos", 10);

	base = std::make_unique<GazeboSimulatorBase>();
	base->init(this->fluidPluginSdf);

	this->ParseSDF();

	Simulation *sim = Simulation::getCurrent();
	sim->init(base->getScene().particleRadius, false);

	base->buildModel();

	if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Akinci2012)
	{
		unsigned int nBoundaryParticles = 0;
		for (unsigned int i = 0; i < sim->numberOfBoundaryModels(); i++)
			nBoundaryParticles += static_cast<BoundaryModel_Akinci2012 *>(sim->getBoundaryModel(i))->numberOfParticles();

		LOG_INFO << "Number of boundary particles: " << nBoundaryParticles;
	}
	base->readParameters();
	initBoundaryData();
}

void FluidSimulator::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	this->world = parent->GetWorld();
	//this->model2 = world->ModelByName("modelName");
	this->fluidPluginSdf = sdf;
	this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
		boost::bind(&FluidSimulator::RunStep, this)));


}

void FluidSimulator::RegisterMesh(physics::CollisionPtr collision, std::string extension, std::string path)
{
	// Get collision mesh by name
	const gazebo::common::Mesh *mesh = common::MeshManager::Instance()->GetMesh(collision->GetName());

	// Export the mesh to a temp file in the selected format
	std::string objFilePath = path + collision->GetModel()->GetName() + "_" + collision->GetName() + ".obj";
	common::MeshManager::Instance()->Export(mesh, FileSystem::normalizePath(objFilePath), extension);
	base->processBoundary(collision, objFilePath);
	// Map the mesh filename to the collision shape
	filenamesToCollisions.insert(std::pair<std::string, physics::CollisionPtr>(objFilePath, collision));
}

void FluidSimulator::ParseSDF()
{

	//only prase ground
	physics::ModelPtr models = this->world->ModelByName("ground_plane");

	// get all models from the world
	//physics::Model_V models = world->Models();

	// iterate through all models
	//for (physics::Model_V::iterator currentModel = models.begin(); currentModel != models.end(); ++currentModel)
	//{

		// get all links from the model
		//physics::Link_V model_links = currentModel->get()->GetLinks();
		physics::Link_V model_links = models->GetLinks();

		//std::cout << "Model: " << currentModel->get()->GetName() << std::endl;
		std::cout << "Model: " << models->GetName() << std::endl;

		// iterate through all the links
		for (physics::Link_V::iterator link_it = model_links.begin(); link_it != model_links.end(); ++link_it)
		{
			// get all collisions of the link
			physics::Collision_V collisions = link_it->get()->GetCollisions();

			std::cout << "\t Link: " << link_it->get()->GetName() << std::endl;

			// iterate through all the collisions
			for (physics::Collision_V::iterator collision_it = collisions.begin(); collision_it != collisions.end(); ++collision_it)
			{
				std::cout << "\t\t Collision: " << (*collision_it)->GetName() << std::endl;

				physics::CollisionPtr coll_ptr = boost::static_pointer_cast<physics::Collision>(*collision_it);

				// check the geometry type of the given collision
				sdf::ElementPtr geometry_elem = coll_ptr->GetSDF()->GetElement("geometry");

				// get the name of the geometry
				std::string geometry_type = geometry_elem->GetFirstElement()->GetName();

				// check type of the geometry
				if (geometry_type == "box")
				{
					// Get the size of the box
					ignition::math::Vector3d size = geometry_elem->GetElement(geometry_type)->GetElement("size")->Get<ignition::math::Vector3d>();

					// Create box shape
					common::MeshManager::Instance()->CreateBox((*collision_it)->GetName(), ignition::math::Vector3d(size.X(), size.Y(), size.Z()),
															   ignition::math::Vector2d(1, 1));
					// Generate an obj file in the temporary directory containing the mesh of the box
					RegisterMesh(*collision_it, "obj", objFilePath);
				}

				else if (geometry_type == "cylinder")
				{
					// Cylinder dimensions
					double radius = geometry_elem->GetElement(geometry_type)->GetElement("radius")->Get<double>();
					double length = geometry_elem->GetElement(geometry_type)->GetElement("length")->Get<double>();

					// Create cylinder mesh
					common::MeshManager::Instance()->CreateCylinder((*collision_it)->GetName(), radius, length, 32, 32);

					//Generate an obj file in the temporary directory containing the mesh of the cylinder
					RegisterMesh(*collision_it, "obj", objFilePath);
				}

				else if (geometry_type == "sphere")
				{
					// Sphere radius
					double radius = geometry_elem->GetElement(geometry_type)->GetElement("radius")->Get<double>();

					// Create a sphere mesh
					common::MeshManager::Instance()->CreateSphere((*collision_it)->GetName(), radius, 32, 32);

					// Generate an obj file in the temporary directory containing the mesh of the sphere
					RegisterMesh(*collision_it, "obj", objFilePath);
				}

				else if (geometry_type == "plane")
				{
					ignition::math::Vector3d normal;
					ignition::math::Vector2d size;
					// Plane dimensions. To prevent a huge plane which causes problems when
					// sampling it, for now it is harcoded
					if ((*collision_it)->GetName() == "collision_ground_plane")
					{
						normal = ignition::math::Vector3d(0, 0, 1); // = geom_elem->GetElement(geom_type)->GetElement("normal")->Get<ignition::math::Vector3d>();
						size = ignition::math::Vector2d(4.0, 4.0);  //= geom_elem->GetElement(geom_type)->GetElement("size")->Get<ignition::math::Vector2d>();
					}
					else
					{
						//normal = geometry_elem->GetElement(geometry_type)->GetElement("normal")->Get<ignition::math::Vector3d>();
						//size = geometry_elem->GetElement(geometry_type)->GetElement("size")->Get<ignition::math::Vector2d>();
						normal = ignition::math::Vector3d(0, 0, 1);
						size = ignition::math::Vector2d(4.0, 4.0);
					}

					// Generate the plane mesh
					common::MeshManager::Instance()->CreatePlane((*collision_it)->GetName(), ignition::math::Vector3d(0.0, 0.0, 1.0), 0.0, size, ignition::math::Vector2d(4.0, 4.0), ignition::math::Vector2d());

					//Generate an obj file in the temporary directory containing the mesh of the plane
					RegisterMesh(*collision_it, "obj", objFilePath);
				}

				else if (geometry_type == "mesh")
				{
					// get the uri element value
					const std::string uri = geometry_elem->GetElement(geometry_type)->GetElement("uri")->Get<std::string>();

					// get the filepath from the uri
					const std::string filepath = common::SystemPaths::Instance()->FindFileURI(uri);
					const gazebo::common::Mesh *mesh = common::MeshManager::Instance()->GetMesh(filepath);

					std::string fullMeshPath = objFilePath + (*collision_it)->GetModel()->GetName() + "_" + (*collision_it)->GetName() + ".obj";
					// Export the mesh to a temp file in the selected format
					common::MeshManager::Instance()->Export(mesh, FileSystem::normalizePath(fullMeshPath), "obj");
					base->processBoundary(*collision_it, fullMeshPath);
					// Map the mesh filename to the collision shape
					filenamesToCollisions.insert(std::pair<std::string, physics::CollisionPtr>(fullMeshPath, (*collision_it)));
				}
				else
				{
					// Error for other possible weird types
					gzerr << "Collision type [" << geometry_type << "] unimplemented\n";
				}
			}
		}
	//} 
}

void FluidSimulator::reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Utilities::Counting::printAverageCounts();
	Utilities::Counting::reset();

	Simulation::getCurrent()->reset();
	base->reset();
}

void FluidSimulator::loadObj(const std::string &filename, SPH::TriangleMesh &mesh, const Vector3r &scale)
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

void FluidSimulator::initBoundaryData()
{
	// TODO: initializations ok?
	Simulation *sim = Simulation::getCurrent();
	GazeboSceneLoader::Scene &scene = base->getScene(); // scene

	unsigned int currentModelIndex = 0;
	for (unsigned int i = 0; i < scene.boundaryModels.size(); ++i)
	{
		StaticRigidBody *rigidBody = new StaticRigidBody();
		if (scene.boundaryModels[i]->rigidBody->GetModel()->GetSDF()->HasElement("static"))
		{
			rigidBody->setDynamic(scene.boundaryModels[i]->dynamic);
			std::cout << scene.boundaryModels[i]->rigidBody->GetModel()->GetName() << "is dynamic " << scene.boundaryModels[i]->dynamic << std::endl;
		}
		TriangleMesh &geo = rigidBody->getGeometry();
		loadObj(scene.boundaryModels[i]->objFilePath, geo, scene.boundaryModels[i]->scale);
		std::vector<Vector3r> boundaryParticles;
		if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Akinci2012)
		{
			// LOG_INFO << "Poisson disk surface sampling of " << meshFileName;
			START_TIMING("Poisson disk sampling");
			PoissonDiskSampling sampling;
			sampling.sampleMesh(geo.numVertices(), geo.getVertices().data(), geo.numFaces(), geo.getFaces().data(), scene.particleRadius, 10, 1, boundaryParticles);
			STOP_TIMING_AVG;
			/* 	for (unsigned int j = 0; j < (unsigned int)boundaryParticles.size(); j++)
		{
		//	boundaryParticles[j] = scene.boundaryModels[i]->rotation * boundaryParticles[j] + scene.boundaryModels[i]->translation;
			std::cout << boundaryParticles[j][2] << std::endl;
		} */
			//boundaryParticles[j] = fluidObjectOrientation * boundaryParticles[j] + fluidObjectPosition;
		}
		boundariesToCollisions.insert(std::pair<StaticRigidBody *, physics::CollisionPtr>(rigidBody, scene.boundaryModels[i]->rigidBody));
		rigidBody->setWorldSpacePosition(scene.boundaryModels[i]->translation);
		rigidBody->setWorldSpaceRotation(scene.boundaryModels[i]->rotation);
		rigidBody->setPosition(scene.boundaryModels[i]->translation);
		rigidBody->setRotation(scene.boundaryModels[i]->rotation);

		if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Akinci2012)
		{
			BoundaryModel_Akinci2012 *bm = new BoundaryModel_Akinci2012();
			bm->initModel(rigidBody, static_cast<unsigned int>(boundaryParticles.size()), &boundaryParticles[0]);
			sim->addBoundaryModel(bm);
		}
		else if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Koschier2017)
		{
			BoundaryModel_Koschier2017 *bm = new BoundaryModel_Koschier2017();
			bm->initModel(rigidBody);
			sim->addBoundaryModel(bm);
			SPH::TriangleMesh &mesh = rigidBody->getGeometry();
			base->initDensityMap(mesh.getVertices(), mesh.getFaces(), scene.boundaryModels[i], false, false, bm);
		}
		else if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Bender2019)
		{
			BoundaryModel_Bender2019 *bm = new BoundaryModel_Bender2019();
			bm->initModel(rigidBody);
			sim->addBoundaryModel(bm);
			SPH::TriangleMesh &mesh = rigidBody->getGeometry();
			base->initVolumeMap(mesh.getVertices(), mesh.getFaces(), scene.boundaryModels[i], false, false, bm);
		}
		for (unsigned int j = 0; j < geo.numVertices(); j++)
			geo.getVertices()[j] = scene.boundaryModels[i]->rotation * geo.getVertices()[j] + scene.boundaryModels[i]->translation;

		geo.updateNormals();
		geo.updateVertexNormals();
	}

	if (sim->getBoundaryHandlingMethod() == BoundaryHandlingMethods::Akinci2012)
	{
		base->updateBoundaryParticles(boundariesToCollisions, true);
		sim->updateBoundaryVolume();
	}

	sim->performNeighborhoodSearchSort();

#ifdef GPU_NEIGHBORHOOD_SEARCH
	// copy the particle data to the GPU
	sim->getNeighborhoodSearch()->update_point_sets();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(FluidSimulator)
