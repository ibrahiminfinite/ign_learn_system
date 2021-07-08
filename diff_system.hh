#include <ignition/gazebo/System.hh>

namespace diff_system
{
    // Forward declaration of the implementation class
    class DiffSystemPrivate;

    class DiffSystem:
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
    {
    // Definition begin

    private: std::unique_ptr<DiffSystemPrivate> dataPtr;


    public : DiffSystem();
    public : ~DiffSystem() override;
    public : void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm) override;

    public : void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                             const ignition::gazebo::EntityComponentManager &_ecm) override;

    public : void Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr ) override;

    // Definition End
    };
}