#ifndef TEST
#define TEST

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/node.hpp>

using namespace godot;

class Test : public Node {
	GDCLASS(Test, Node);
	
public:
	void _ready();
	
protected:
	static void _bind_methods() {};
};
Test* t;

#endif
