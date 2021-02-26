/* Desc: System plugin for rendering the particles from Fluidix
 * Author: Andrei Haidu
 */

#include "GazeboFluidVisualizer.h"

using namespace gazebo;

//////////////////////////////////////////////////
FluidVisPlugin::FluidVisPlugin()
{
}

//////////////////////////////////////////////////
FluidVisPlugin::~FluidVisPlugin()
{
}

//////////////////////////////////////////////////
void FluidVisPlugin::Load(int _argc, char **_argv)
{
	// check for the given arguments
	std::cout << "Visualization plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void FluidVisPlugin::Init()
{
	this->node = transport::NodePtr(new transport::Node());

	this->newFluidMsgReceived = false;

	// Event to check that the rendering engine is loaded
	this->updateConnection = event::Events::ConnectPreRender(
		boost::bind(&FluidVisPlugin::InitAtRenderEvent, this));
}

//////////////////////////////////////////////////
void FluidVisPlugin::InitAtRenderEvent()
{
	// Initialize node only after the rendering engine has been loaded
	this->node->Init();

	// subscribe to the fluid topic
	this->fluidSub = this->node->Subscribe("~/fluid_pos",
										   &FluidVisPlugin::OnFluidMsg, this);

	//this->rigidsSub = this->node->Subscribe("~/rigids_pos",
											//&FluidVisPlugin::OnRigidMsg, this);

	if (!this->userCam)
	{
		// Get a pointer to the active user camera
		this->userCam = gui::get_active_camera();

		this->userCam->OgreViewport()->setVisibilityMask(GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_SELECTABLE);

		// Enable saving frames
		/* 	this->userCam->EnableSaveFrame(true);
		this->userCam->SetRenderRate(1.0/60);

		// Specify the path to save frames into
		this->userCam->SetSaveFramePathname("path_to_save"); */
	}

	this->manager = this->userCam->GetScene()->OgreSceneManager();

	this->updateConnection = event::Events::ConnectPreRender(
		boost::bind(&FluidVisPlugin::RenderAsPointsUpdate, this));
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderAsPointsUpdate()
{
	// render fluid if new message received
	if (this->newFluidMsgReceived)
	{
		// render fluid particles
		//FluidVisPlugin::RenderParticlesAsEntities(this->fluidParticlePositions, "fluid1");
		FluidVisPlugin::RenderParticles(this->fluidParticlePositions, "fluid1");
		//FluidVisPlugin::RenderParticlesAsEntities(this->rigidsParticlePositions, "rigid1");

		//FluidVisPlugin::RenderParticles(this->rigidsParticlePositions, "rigid1");
		//	FluidVisPlugin::RenderParticlesAsEntities(this->rigidsParticlePositions, "rigid1");
		this->newFluidMsgReceived = false;
	}

}
/////////////////////////////////////////////////
void FluidVisPlugin::OnFluidMsg(
	const boost::shared_ptr<msgs::Fluid const> &_msg)
{
	auto start2 = std::chrono::high_resolution_clock::now();
	this->newFluidMsgReceived = true;
	this->fluidParticlePositions.resize(_msg->position_size());

	for (int i = 0; i < _msg->position_size(); ++i)
	{
		this->fluidParticlePositions[i] = Ogre::Vector3(
			_msg->position(i).x(), _msg->position(i).y(), _msg->position(i).z());
	}
	auto finish2 = std::chrono::high_resolution_clock::now();
	auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(finish2 - start2);
	std::ofstream outfile;
	outfile.open("/home/jason/splisplash/timeDurationRender.txt", std::ios::out | std::ios::app);
	outfile << duration2.count() << std::endl;
}

/*void FluidVisPlugin::OnRigidMsg(
	const boost::shared_ptr<msgs::Fluid const> &_msg)
{

	this->newFluidMsgReceived = true;
	this->rigidsParticlePositions.resize(_msg->position_size());

	for (int i = 0; i < _msg->position_size(); ++i)
	{
		this->rigidsParticlePositions[i] = Ogre::Vector3(
			_msg->position(i).x(), _msg->position(i).y(), _msg->position(i).z());
	}
}*/
/////////////////////////////////////////////////
void FluidVisPlugin::RenderParticles(std::vector<Ogre::Vector3> &_particles, std::string _name)
{
	Ogre::SceneNode *sceneNode = NULL;
	Ogre::ManualObject *obj = NULL;
	bool attached = false;

	if (this->manager->hasManualObject(_name))
	{
		sceneNode = this->manager->getSceneNode(_name);
		obj = this->manager->getManualObject(_name);
		attached = true;
	}
	else
	{
		sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(_name);
		obj = this->manager->createManualObject(_name);
	}

	sceneNode->setVisible(true);
	obj->setVisible(true);

	obj->clear();

	// OT_POINT_LIST = 1, A list of points, 1 vertex per point
	// OT_LINE_LIST = 2, A list of lines, 2 vertices per line
	// OT_LINE_STRIP = 3, A strip of connected lines, 1 vertex per line plus 1 start vertex
	// OT_TRIANGLE_LIST = 4, A list of triangles, 3 vertices per triangle
	// OT_TRIANGLE_STRIP = 5, A strip of triangles, 3 vertices for the first triangle, and 1 per triangle after that
	// OT_TRIANGLE_FAN = 6, A fan of triangles, 3 vertices for the first triangle, and 1 per triangle after that
	obj->begin("Gazebo/Black", Ogre::RenderOperation::OT_POINT_LIST);	
	
	for (unsigned int i = 0; i < _particles.size(); ++i)
	{	
		obj->position(_particles[i]);	

	}

	obj->end();


	if (!attached)
		sceneNode->attachObject(obj);
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderParticlesAsEntities(std::vector<Ogre::Vector3> &_particles, std::string _name)
{
	std::cout << "OnUpdate: Rendering new positions.." << std::endl;
	for (unsigned int i = 0; i < _particles.size(); ++i)
	{
		Ogre::SceneNode *sceneNode = NULL;
		Ogre::Entity *entity = NULL;
		bool attached = false;
		std::ostringstream name_ss;
		std::string name;

		name_ss << _name << "_" << i;
		name = name_ss.str();

		if (this->manager->hasEntity(name))
		{
			sceneNode = this->manager->getSceneNode(name);
			entity = this->manager->getEntity(name);
			attached = true;
		}
		else
		{
			sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(name);
			entity = this->manager->createEntity(name,
												 Ogre::SceneManager::PT_SPHERE);//PT_SPHERE,PT_CUBE 
			entity->setMaterialName("Gazebo/Red");
		}

		sceneNode->setVisible(true);
		entity->setVisible(true);

		double particleSize = 0.08;

		sceneNode->setScale(0.005 * particleSize, 0.005 * particleSize, 0.005 * particleSize);

		sceneNode->setPosition(_particles[i]);

		if (!attached)
		{
			sceneNode->attachObject(entity);
		}
	}

}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(FluidVisPlugin)
