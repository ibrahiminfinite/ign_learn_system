#include <ignition/gazebo/System.hh>

namespace diff_system
{
    class DiffSystem:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemPostUpdate
    {
    public : DiffSystem();
    public : ~DiffSystem() override;
    public : void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                             const ignition::gazebo::EntityComponentManager &_ecm) override;
    };
}