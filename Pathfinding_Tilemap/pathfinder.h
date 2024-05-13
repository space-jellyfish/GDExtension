#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/classes/node.hpp>

namespace godot {

class Pathfinder : public Node {
    GDCLASS(Pathfinder, Node);

private:
    double time_passed;

protected:
    static void _bind_methods();

public:
    Pathfinder();
    ~Pathfinder();
};

}

#endif