#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Link.hh"

#include "diff_system.hh"


using namespace ignition;
using namespace gazebo;
using namespace diff_system;

class diff_system::DiffSystemPrivate
{

public : Model model{kNullEntity};
public : Link canonicalLink{kNullEntity};

public : std::vector<Entity> leftJoints;
public : std::vector<Entity> rightJoints;
public : std::vector<std::string> leftJointNames;
public : std::vector<std::string> rightJointsNames;

public : std::string tmsg="TEST PRIVATE";
};

DiffSystem::DiffSystem()
        : dataPtr(std::make_unique<DiffSystemPrivate>())
{
}
DiffSystem::~DiffSystem()=default;

void DiffSystem::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
    ignmsg << "DiffSystem::Configure " << this->dataPtr->tmsg << std::endl;
    this->dataPtr->model = Model(_entity);
    std::vector<Entity> links = _ecm.ChildrenByComponents(
            this->dataPtr->model.Entity(), components::CanonicalLink());

    if (!links.empty())
        this->dataPtr->canonicalLink = Link(links[0]);

    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "DiffDrive plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }

}

void DiffSystem::PostUpdate(const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
{
    ignmsg << "DiffSystem::PostUpdate" << std::endl;
}

IGNITION_ADD_PLUGIN(
        diff_system::DiffSystem,
        ignition::gazebo::System,
        diff_system::DiffSystem::ISystemPostUpdate,
        diff_system::DiffSystem::ISystemConfigure
        )