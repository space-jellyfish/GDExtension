#ifndef TEST
#define TEST

#include <godot_cpp/classes/object.hpp>

using namespace godot;

class Test : public Object {
	GDCLASS(Test, Object);
	
public:
	void _ready();
	
protected:
	static void _bind_methods() {};
};

#endif
