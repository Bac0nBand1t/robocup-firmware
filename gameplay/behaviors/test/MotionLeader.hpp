#pragma once

#include "../../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class MotionLeader: public Behavior
        {
            public:
                MotionLeader(GameplayModule *gameplay, Role *role);

                virtual void run();

            protected:
                ObstaclePtr centerObstacle[2];
                ObstaclePtr quadrantObstacle[4];
                ObstaclePtr goalObstacle[2];
        };
    }
}
