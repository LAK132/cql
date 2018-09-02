# C++ Quaternion Library

This library implements 2 types: `quat` (quanterion) and `dquat` (dual-quaternion). Both types can be templated with different number types, or one of the typedefs (`quatf_t`, `quatd_t`, `dquatf_t`, `dquatd_t`) can be used.

By default this library attempts to use GLM to allow for conversion to and from matrices and various other transforms. This can be disabled by defining `CQL_NO_GLM` befor inclusion.