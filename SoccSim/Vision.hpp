#ifndef VISION_HPP_
#define VISION_HPP_

#include "Physics/Env.hpp"

#include <Network/PacketReceiver.hpp>

#include <QThread>

class Vision : public QThread
{
    public:
	Vision(Env* env);
	~Vision();

	uint64_t timestamp();
	void radioHandler(const Packet::RadioTx* packet);

    protected:
	    void run();

    private:

	Env* _env;

	unsigned int _id, _fps;

	/** Reciever for radio commands. Temporary until something better comes along**/
	Network::PacketReceiver* _receiver;
};
#endif /* VISION_HPP_ */
