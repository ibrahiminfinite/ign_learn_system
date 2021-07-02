#include <ignition/plugin/Register.hh>
#include "diff_system.hh"


using namespace diff_system;

DiffSystem::DiffSystem()=default;
DiffSystem::~DiffSystem()=default;

void DiffSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                              const ignition::gazebo::EntityComponentManager &_ecm)
{
    ignmsg << "DiffSystem::PostUpdate" << std::endl;
}

IGNITION_ADD_PLUGIN(
        diff_system::DiffSystem,
        ignition::gazebo::System,
        diff_system::DiffSystem::ISystemPostUpdate
        )