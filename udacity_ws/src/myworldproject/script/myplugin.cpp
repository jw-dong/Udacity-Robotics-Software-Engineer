#include <gazebo/gazebo.hh>

namespace gazebo
{
	class myprojectplugin : public WorldPlugin
	{
		public: myprojectplugin() : WorldPlugin()
			{
				printf("Welcome to Jiawei's world! \n");
			}

		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
			{
			}
	};
	GZ_REGISTER_WORLD_PLUGIN(myprojectplugin)

}
