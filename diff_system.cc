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
    std::vector<std::string> rightJointNames;
    double leftJointSpeed{0.2};
    double rightJointSpeed{0.2};

    std::string tmsg="TEST PRIVATE";
};


/////////////////////////////////////////////////////
DiffSystem::DiffSystem()
        : dataPtr(std::make_unique<DiffSystemPrivate>())
{
}

/////////////////////////////////////////////////////
DiffSystem::~DiffSystem()=default;

//////////////////////////////////////////////////////
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

    while (sdfElem)
    {
        this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
        ignmsg << "NAME : "<<sdfElem->Get<std::string>()<<std::endl;
        sdfElem = sdfElem->GetNextElement("left_joint");
    }
    sdfElem = ptr->GetElement("right_joint");
    while (sdfElem)
    {
        this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
        ignmsg << "NAME : "<<sdfElem->Get<std::string>()<<std::endl;
        sdfElem = sdfElem->GetNextElement("right_joint");
    }
}

/////////////////////////////////////////////////////
void DiffSystem::PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
    ignmsg << "DiffSystem::PreUpdate" << std::endl;
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
                << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                << "s]. System may not work properly." << std::endl;
    }

    //////////////
    // If the joints haven't been identified yet, look for them
    static std::set<std::string> warnedModels;
    auto modelName = this->dataPtr->model.Name(_ecm);
    if (this->dataPtr->leftJoints.empty() ||
        this->dataPtr->rightJoints.empty())
    {
        bool warned{false};
        for (const std::string &name : this->dataPtr->leftJointNames)
        {
            Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != kNullEntity)
                this->dataPtr->leftJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find left joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }

        for (const std::string &name : this->dataPtr->rightJointNames)
        {
            Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != kNullEntity)
                this->dataPtr->rightJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find right joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }
        if (warned)
        {
            warnedModels.insert(modelName);
        }
    }
    if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
        return;

    if (warnedModels.find(modelName) != warnedModels.end())
    {
        ignmsg << "Found joints for model [" << modelName
               << "], plugin will start working." << std::endl;
        warnedModels.erase(modelName);
    }
    /////////////

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
        else
        {
            *vel = components::JointVelocityCmd({this->dataPtr->leftJointSpeed});
        }
    }
    for (Entity joint : this->dataPtr->rightJoints)
    {
        //get pointer to component via ECM
        auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
        if (vel == nullptr)
        {
            ignmsg << "Creating component JointVelocityCmd"<<std::endl;
            _ecm.CreateComponent(
                    joint, components::JointVelocityCmd({this->dataPtr->leftJointSpeed}));

        }
        else
        {
            *vel = components::JointVelocityCmd({this->dataPtr->leftJointSpeed});
        }
    }
}

////////////////////////////////////////////////////
void DiffSystem::PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
{

}

////////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(
        diff_system::DiffSystem,
        ignition::gazebo::System,
        diff_system::DiffSystem::ISystemPreUpdate,
        diff_system::DiffSystem::ISystemPostUpdate,
        diff_system::DiffSystem::ISystemConfigure
        )