# C++ Quaternion Library

This library implements 2 types: `quat` (quanterion) and `dquat` (dual-quaternion). Both types can be templated with different number types, or one of the typedefs (`quatf_t`, `quatd_t`, `dquatf_t`, `dquatd_t`) can be used.

Will attempt to find GLM headers. If <glm/gtc/constants.hpp>, <glm/mat4x4.hpp>, <glm/vec4.hpp> and <glm/vec3.hpp> are available, cql will generate functions for conversion to and from GLM vectors and matrices
