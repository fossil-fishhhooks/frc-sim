#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <string>


// converts a mesh file into a Jolt collision shape.
JPH::Ref<JPH::Shape> LoadShape(const std::string& mesh_path, bool is_static);
