#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include "Entity.hpp"
#include <Geometry/Point2d.hpp>

class Robot : public Entity
{
    public:
        Robot(NxScene& scene);
        ~Robot();

	void step();

	/** set position of the robot */
	void position(float x, float y);

        /** Get the robot's position **/
        Geometry::Point2d* getPosition();

	//FIXME
	int8_t vels[4];

    private:
        void initRoller();
        void initKicker();
        void initWheels();

        static NxConvexMesh* cylinder(const float length, const float radius,
                const unsigned int sides);

    private:
        /** the roller actor */
        NxActor* _roller;

        /** the kicker actor */
        NxActor* _kicker;

        NxActor* _wheels[4];
	NxRevoluteJoint* _motors[4];

        /** center of roller from ground */
        const static float RollerHeight = .03;
        /** center of roller from center of robot */
        const static float RollerOffset = .065;
        /** roller length */
        const static float RollerLength = .07;
        /** radius of the roller */
        const static float RollerRadius = .01;

        /** width of the kicker face */
        const static float KickerFaceWidth = .05;
        /** height of the kicker face */
        const static float KickerFaceHeight = .005;
        /** depth of the kicker plate */
        const static float KickerLength = .03;
};

#endif /* _ROBOT_HPP */
