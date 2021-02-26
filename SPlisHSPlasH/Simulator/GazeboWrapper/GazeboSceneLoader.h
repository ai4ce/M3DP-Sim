#ifndef __GazeboSceneLoader_h__
#define __GazeboSceneLoader_h__

#include "SPlisHSPlasH/Common.h"
#include "gazebo/gazebo.hh"
#include <vector>
#include "ParameterObject.h"
#include "SPlisHSPlasH/Utilities/SceneLoader.h"

namespace Utilities
{
/** \brief Importer of Gazebo SPlisHSPlasH scene files. 
	*/
class GazeboSceneLoader
{

public:
	struct GazeboBoundaryData
	{
		std::string collisionName;
		std::string objFilePath;
		Vector3r translation;
		Matrix3r rotation;
		Vector3r scale;
		bool dynamic;
		bool isWall;
		Real density = 1000.0;
		gazebo::physics::CollisionPtr rigidBody;
		unsigned int samplingMode;

		bool mapInvert;
		Real mapThickness;
		Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign> mapResolution;
	};

	/** \brief Struct to store scene information */
	struct Scene
	{
		std::vector<GazeboBoundaryData *> boundaryModels;
		std::vector<SceneLoader::FluidData *> fluidModels;
		std::vector<SceneLoader::FluidBlock *> fluidBlocks;
		std::vector<SceneLoader::EmitterData *> emitters;
		Real particleRadius;
		Real timeStepSize;
	};

	template <typename T>
	bool getSDFParameter(const sdf::ElementPtr sdf, T &parameter, const std::string &parameterName, const T &defaultValue);
	void readScene(const sdf::ElementPtr &fluidSceneSDF, Scene &scene);
	void readParameterObject(const std::string &key, GenParam::ParameterObject *paramObj);
	bool getVector3rParameter(const sdf::ElementPtr sdf, Vector3r &parameter, const ::std::string &parameterName, const Vector3r &defaultValue);
	void processBoundary(Scene &scene, const gazebo::physics::CollisionPtr &collision, std::string objFilePath);
	//void getVector3iParameter(const sdf::ElementPtr sdf, Eigen::Matrix<unsigned int, 3, 1> &parameter, const ::std::string &parameterName, const Eigen::Matrix<unsigned int, 3, 1> &defaultValue)
	//void getSDFParameter(Scene &scene, sdf::ElementPtr sdf, const std::string &parameterName);

private:
	sdf::ElementPtr fluidSceneSDF;
	void processFluidModels(Scene &scene, const sdf::ElementPtr &sdf);
	void processFluidBlocks(Scene &scene, const sdf::ElementPtr &sdf);
	void processFluidEmmiters(Scene &scene, const sdf::ElementPtr &sdf);
};

/* template <>
	bool GazeboSceneLoader::readValue<bool>(const nlohmann::json &j, bool &v); */

} // namespace Utilities

#endif
