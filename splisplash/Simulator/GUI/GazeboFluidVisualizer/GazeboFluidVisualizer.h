/* Desc: System plugin for rendering the particles from Fluidix
 * Author: Andrei Haidu
 */

#ifndef FLUID_VIS_PLUGIN_HH
#define FLUID_VIS_PLUGIN_HH

#include "gazebo/gazebo.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
/// \brief FluidVisPlugin class
class GAZEBO_VISIBLE FluidVisPlugin : public SystemPlugin
{

    /// \brief Constructor
  public:
    FluidVisPlugin();

    /// \brief Destructor
  public:
    ~FluidVisPlugin();

    /// \brief Load plugin
  protected:
    void Load(int _argc, char **_argv);

    /// \brief Init plugin (Load called first, then Init)
  protected:
    void Init();

    /// \brief Initialize rendering, scene, cameras,
    /// called at a rendering event when the rendering is initialized
  protected:
    void InitAtRenderEvent();

    /// \brief Render liquid as points event update
  protected:
    void RenderAsPointsUpdate();

    /// \brief Render liquid as spheres event update
    //protected: void RenderAsSpheresUpdate();

    /// \brief Fluid message callback.
    /// \param[in] _msg The message data.
  private:
    void OnFluidMsg(
        const boost::shared_ptr<msgs::Fluid const> &_msg);

  private:
    void OnRigidMsg(
        const boost::shared_ptr<msgs::Fluid const> &_msg);

    /// \brief Fluid Obj message callback.
    /// \param[in] _msg The message data.
    /*  private: void OnFluidObjMsg(
    		const boost::shared_ptr<msgs::Fluid const> &_msg); */

    /// \brief Render particles
  private:
    void RenderParticles(std::vector<Ogre::Vector3> &_particles, std::string _name);

   /// \brief Render particles
  private:
    void RenderParticlesAsEntities(std::vector<Ogre::Vector3> &_particles, std::string _name);

    /// \brief Communication Node
  private:
    transport::NodePtr node;

    /// \brief Event connection
  private:
    event::ConnectionPtr updateConnection;

    /// \brief Subscribe to fluid pose updates
  private:
    transport::SubscriberPtr fluidSub;

  private:
    transport::SubscriberPtr rigidsSub;

    /// \brief The ogre scene manager.
  private:
    Ogre::SceneManager *manager;

    /// \brief Flag if new fluid message received
  private:
    bool newFluidMsgReceived;

    /// \brief Vector of Ogre::Vector3 representing all fluid particle positions
  private:
    std::vector<Ogre::Vector3> fluidParticlePositions;

  private:
    std::vector<Ogre::Vector3> rigidsParticlePositions;

  private:
    std::string visualizationType;

    /// Pointer the user camera.
  private: rendering::UserCameraPtr userCam;
};
} // namespace gazebo
#endif
