/* godot-cpp integration testing project.
 *
 * This is free and unencumbered software released into the public domain.
 */

#ifndef PATHFINDER_REGISTER_TYPES_H
#define PATHFINDER_REGISTER_TYPES_H

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void initialize_pathfinder_module(ModuleInitializationLevel p_level);
void uninitialize_pathfinder_module(ModuleInitializationLevel p_level);

#endif // PATHFINDER_REGISTER_TYPES_H
