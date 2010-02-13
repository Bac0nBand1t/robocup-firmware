#include "Kick.hpp"

#include "../Window.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Utils;
using namespace Geometry2d;

//#define DEBUG

#ifdef DEBUG
#define debug(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug(...)
#endif

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay) :
	Behavior(gameplay), _ballHandlingScale(1.0), _ballHandlingRange(0.5)
{
	automatic = true;
	_win = 0;
	targetRobot = 0;
	_intercept = new Gameplay::Behaviors::Intercept(gameplay);
}

Gameplay::Behaviors::Kick::~Kick()
{
	if (_win)
	{
		delete _win;
	}
	if (_intercept)
	{
		delete _intercept;
	}
}

void Gameplay::Behaviors::Kick::assign(set<Robot *> &available)
{
	_robots.clear(); // clear existing robots
	_intercept->robots().clear();
	takeBest(available);
	
	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = false;
	}

	_state = Intercept;
	_intercept->assignOne(robot());
	_lastMargin = 0;
}

bool Gameplay::Behaviors::Kick::run()
{
	if (!allVisible() || !ball().valid)
	{
		// No ball
		return false;
	}

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	const Geometry2d::Point pos = robot()->pos();

	// find a target segment using window evaluators
	if (targetRobot) {
		_target = evaluatePass();
	} else {
		_target = evaluateShot();
	}
	drawLine(_target, 255, 0, 0); // show the target segment

	Geometry2d::Point targetCenter = _target.center();

	//Determine how hard to kick given
	int kickStrength = calcKickStrength(targetCenter);

	// Always face the ball
	robot()->face(ballPos);

	// Vector that we would shoot along if we shot now
	// assume that the ball goes where we were facing
	//Geometry2d::Point shootDir = -(pos - ballPos);
	Point shootDir = Point::direction(robot()->angle() * DegreesToRadians);

	//shot segment is from us in direction of shootDir
	//it is sufficiently large to handle the entire field shot
	Segment shotLine (pos, pos + shootDir * 7.0);

	//the point of intersection of the shotLine and target
	Geometry2d::Point shootPoint;

	//see if the shot line intersects where we indent to shoot
	bool intersectedTarget = shotLine.intersects(_target, &shootPoint);

	// robot intersection tests
	bool intersectsRobot = checkRobotIntersections(shotLine);

	// canKick is true if the robot is facing the target and the ball is between the robot and the target.
	bool canKick = intersectedTarget && robot()->haveBall() && !intersectsRobot && robot()->charged();

	//debug("%d ", canKick);

#if 0
	if (_state == Shoot && !canKick)
	{
		debug("Abort shoot, ball out of range\n");
		_state = Turn;
	}
#endif

	// keep track of state transitions
	State oldState = _state;
	QColor toggle = Qt::magenta, stable = Qt::black;
	Point textOffset(Constants::Robot::Radius*1.3, 0.0);

	// keep a set of thresholds for aiming and shooting
	const float aimThresh = Constants::Robot::Radius * 1.2;

	// STATE TRANSITION OVERRIDES
	//if we already have the ball, skip approach states
	if (_state == Intercept && robot()->haveBall())
	{
		//cout << "Intercept succeded - switching to aim" << endl;
		_state = Aim;
		_pivot = ballPos;
	} else if (_state == Aim && !robot()->haveBall() && pos.distTo(ballPos) > aimThresh)
	{
		//cout << "Lost ball - switching to intercept" << endl;
		_state = Intercept;
	}

	// HANDLE STATES (with debug text)
	switch (_state) {
	case Intercept:
		//approach the ball at high speed using Intercept
		_state = intercept(targetCenter);
		drawText("Intercept",robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Aim:
		_state = aim(targetCenter, canKick);
		drawText("Aim",robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Shoot:
		_state = shoot(targetCenter, kickStrength);
		drawText("Shoot",robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Done: // do nothing
		break;
	}
	
	return _state != Done;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::intercept(const Geometry2d::Point& targetCenter) {
	_intercept->target = targetCenter;
	if (!_intercept->run())
	{
		return Aim;
	}
	return Intercept;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::aim(const Geometry2d::Point& targetCenter, bool canKick) {
	debug("Aim: ");

	// scale velocity due to range
	if (targetCenter.distTo(robot()->pos()) <= _ballHandlingRange) {
		robot()->setVScale(_ballHandlingScale);
	}

	// prepare the robot for kicking
	robot()->willKick = true;
	robot()->dribble(50);

	const float clearance = Constants::Ball::Radius + Constants::Robot::Radius;

	// pull out current states
	Geometry2d::Point ballPos = ball().pos,
					  ballVel = ball().vel,
					  pos     = robot()->pos();

	Geometry2d::Line line(ballPos, ballPos + ballVel);
	Geometry2d::Point intercept = line.nearestPoint(pos);

	// Vectors from ball to extents of target
	Geometry2d::Point v0 = (_target.pt[0] - ballPos).normalized();
	Geometry2d::Point v1 = (_target.pt[1] - ballPos).normalized();
	float g0 = v0.angle() * RadiansToDegrees;
	float g1 = v1.angle() * RadiansToDegrees;

	if (g0 > g1)
	{
		swap(g0, g1);
	}

	float ra = robot()->angle();
	// 		float ba = (ballPos - pos).angle() * RadiansToDegrees;

	Geometry2d::Point m = ballPos + (pos - ballPos).normalized()
							* clearance;
	if (pos.nearPoint(m, Constants::Robot::Radius))
	{
		debug("pivot  ");
		// We're close to the ideal kicking location, so fine tune by moving back and forth
		// to avoid getting stuck with a very small move.

		// From the ball's point of view:
		//     t is towards the target.
		//     s is perpendicular to that.
		Geometry2d::Point t = (targetCenter - ballPos).normalized();
		Geometry2d::Point s = t.perpCCW();


		// How far the robot is from the ball in that direction
		float d = (pos - ballPos).dot(s);


		// Pivot towards the target-ball line
		robot()->pivot(_pivot, d < 0);
	}
	else
	{
		debug("move   ");
		robot()->move(m);
	}

	bool shotAvailable = false;
	if (canKick)
	{
		float margin = max(fixAngleDegrees(ra - g0), fixAngleDegrees(g1
				- ra));

		float threshold = 0.9f * (g1 - g0) / 2;
		debug(
				"goal %.1f, %.1f ball %.1f robot %.1f margin %.1f threshold %.1f\n",
				g0, g1, ba, ra, margin, threshold);
		if (margin > threshold || margin <= _lastMargin)
		{
			if (ballPos.nearPoint(pos, clearance + Constants::Robot::Radius))
			{
				debug("Shoot\n");
				_shootStart = pos;
				_shootMove = pos + (ballPos - pos).normalized() * 0.2f;
				_shootBallStart = ballPos;
				shotAvailable = true;
			}
		}

		_lastMargin = margin;
	}
	else
	{
		debug("wait %.1f %.1f %.1f %.1f", g0, g1, ba, ra);
		_lastMargin = 0;
	}
	debug("\n");

	// determine return state
	return shotAvailable ? Shoot : Aim;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::shoot(const Geometry2d::Point& targetCenter, int kickStrength) {
	debug("Shoot: %f", ball().vel.dot(robot()->pos() - ball().pos));
	//robot()->kick(strength_param.value());
	robot()->kick(kickStrength);
	robot()->face(targetCenter);
	robot()->dribble(0);
	robot()->move(_shootMove);

	// If the robot has moved more than half a meter since starting to shoot, we are probably
	// just pushing the ball around, so give up.
	if (!ball().pos.nearPoint(_shootBallStart, 0.2f) || !robot()->pos().nearPoint(_shootStart, 0.5f))
	{
		// Ball was kicked or we moved far enough to give up
		return Done;
		debug("  done");
	}
	debug("\n");
	return Shoot;
}


float Gameplay::Behaviors::Kick::score(Robot* robot)
{
	return (robot->pos() - ball().pos).magsq();
}

Geometry2d::Segment Gameplay::Behaviors::Kick::evaluatePass() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	// Kick towards a robot
	Geometry2d::Point t = targetRobot->pos();
	_win->run(ball().pos, t);
	_win->exclude.push_back(t);

	Segment target;
	if (_win->best)
	{
		target = _win->best->segment;
	} else {
		// Can't reach the target
		target = _win->target();
	}
	return target;
}

Geometry2d::Segment Gameplay::Behaviors::Kick::evaluateShot() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	//goal line, for intersection detection
	Segment target(Point(-Constants::Field::GoalWidth / 2.0f, Constants::Field::Length),
			Point(Constants::Field::GoalWidth / 2.0f, Constants::Field::Length));

	// Try to kick to the goal.
	_win->run(ball().pos, target);

	if (!_win->best && automatic)
	{
		// Use the entire end line
		target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
		target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);

		_win->run(ball().pos, target);
		if (!_win->best)
		{
			// Use the last 1/3rd of a side
			if (ball().pos.x > 0)
			{
				target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
				target.pt[1] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
			} else {
				target.pt[0] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);
				target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
			}
			_win->run(ball().pos, target);
		}
	}

	Segment bestTarget;
	if (_win->best)
	{
		bestTarget = _win->best->segment;
	} else {
		// Can't reach the target
		bestTarget = _win->target();
	}
	return bestTarget;
}

int Gameplay::Behaviors::Kick::calcKickStrength(const Geometry2d::Point& targetCenter) {
	//if kicking to a target
	//calculate kick strength
	int kickStrength = 255;
	if (targetRobot)
	{
		const float dist = robot()->pos().distTo(targetCenter);

		const float m = robot()->packet()->config.kicker.m;
		const float b = robot()->packet()->config.kicker.b;

		kickStrength = int(m * dist + b);

		if (kickStrength < 0)
		{
			kickStrength = 0;
		}
		else if (kickStrength > 255)
		{
			kickStrength = 255;
		}
	}
	return kickStrength;
}

//FIXME - These robot equivalence checks are too complicated.
// Should be able to compare pointers.  Also don't use LogFrame from within Gameplay.
bool Gameplay::Behaviors::Kick::checkRobotIntersections(const Geometry2d::Segment& shotLine) {
	bool intersectsRobot = false;
	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->opp)
	{
		//don't check against if target
		bool noAdd = (targetRobot && !targetRobot->self() && targetRobot->packet()->shell == r.shell);
		if (r.valid && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}

	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->self)
	{
		//don't check against if target
		bool noAdd = (targetRobot && targetRobot->self() && targetRobot->packet()->shell == r.shell);
		if (r.valid && r.shell != robot()->packet()->shell && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}
	return intersectsRobot;
}

