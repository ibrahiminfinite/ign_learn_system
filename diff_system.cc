#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/Model.hh"
//#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"

#include "diff_system.hh"


using namespace ignition;
using namespace gazebo;
using namespace diff_system;

class diff_system::DiffSystemPrivate
{

public :
    Model model{kNullEntity};
    Link canonicalLink{kNullEntity};

    std::vector<Entity> leftJoints;
    std::vector<Entity> rightJoints;
    std::vector<std::string> leftJointNames;
    std::vector<std::string> rightJointsNames;
    double leftJointSpeed{0};

    std::string tmsg="TEST PRIVATE";
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

    //Get the joints
    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    // Get params from SDF
    sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
//    sdfElem = ptr->GetElement("right_joint");
    while (sdfElem)
    {
        this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
        ignmsg << "NAME : "<<sdfElem->Get<std::string>()<<std::endl;
        sdfElem = sdfElem->GetNextElement("left_joint");
    }
}


void DiffSystem::PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
    ignmsg << "DiffSystem::PreUpdate" << std::endl;

    // Setting wheel velocities
    for (Entity joint : this->dataPtr->leftJoints)
    {
        //get pointer to component via ECM
        auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
        if (vel == nullptr)
        {
            ignmsg << "Creating component JointVelocityCmd"<<std::endl;
            _ecm.CreateComponent(
                    joint, components::JointVelocityCmd({this->dataPtr->leftJointSpeed}));

        }
    }
}

IGNITION_ADD_PLUGIN(
        diff_system::DiffSystem,
        ignition::gazebo::System,
        diff_system::DiffSystem::ISystemPreUpdate,
        diff_system::DiffSystem::ISystemConfigure
        )