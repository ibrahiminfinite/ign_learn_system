#include <ignition/gazebo/Server.hh>

using namespace  ignition;
class DiffTest
{

public: void MoveCar()
    {
    gazebo::ServerConfig serverConfig;
    auto _sdf_file = "/home/dark/CLionProjects/ign_learn_system/worlds/diff_drive.sdf";
    serverConfig.SetSdfFile(_sdf_file);

    gazebo::Server server(serverConfig);
    server.Run();
    }

};

int main() {
    std::cout << "Hello, World!" << std::endl;
    DiffTest d;
    d.MoveCar();
    return 0;
}
