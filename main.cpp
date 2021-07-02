#include <ignition/gazebo/Server.hh>

using namespace  ignition;
class DiffTest
{

public: void MoveCar()
    {
    common::Console::SetVerbosity(3);
    gazebo::ServerConfig serverConfig;
    auto _sdf_file = "/home/dark/CLionProjects/ign_learn_system/worlds/diff_drive.sdf";
    serverConfig.SetSdfFile(_sdf_file);

    gazebo::Server server(serverConfig);
    server.Run(true,0,false);
    }

};

int main() {
    std::cout << "Hello, World!" << std::endl;
    DiffTest d;
    d.MoveCar();
    return 0;
}
