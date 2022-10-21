#ifndef TYPES_HPP
#define TYPES_HPP

struct LockCommandStruct{
	enum LockCommand {LOCK, UNLOCK} lc;
	interval_t ts;
};

struct OpenEventStruct{
	enum OpenEvent {OPEN, CLOSE} oe;
	interval_t ts;
};

struct LockEventStruct{
	enum LockEvent {LOCKED, UNLOCKED} le;
	interval_t ts;
};

#endif