// *********************************************************
// tf_dude
// (c) 2022-2024
// Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
// Institute fuer Robotik und Mechatronik
//
// German Aerospace Center
// Institute for Robotics and Mechatronics
//
// This file is part of the tf_dude repository and is provided as is.
// The repository's license does apply.
// 
// *********************************************************
//
// Authors:
// Sewtz, Marco
//
// *********************************************************

#ifndef TF_DUDE_CODES_H
#define TF_DUDE_CODES_H

#include <cstdint>

namespace dude::tf {
using Status_t = uint16_t;
using Flag_t   = Status_t;

// status codes
struct StatusCodes {
	static constexpr Flag_t Success  = 0x1000;
	static constexpr Flag_t Delete   = 0x0800;
	static constexpr Flag_t Parent   = 0x0001;
	static constexpr Flag_t Frame    = 0x0002;
	static constexpr Flag_t Children = 0x0004;
	static constexpr Flag_t Start    = 0x0008;
	static constexpr Flag_t End      = 0x0010;
};

[[nodiscard]] inline Status_t set_flag(Status_t status, Flag_t flag) {
	return status xor flag;
}

[[nodiscard]] inline Status_t unset_flag(Status_t status, Flag_t flag) {
	return status xor not flag;
}

[[nodiscard]] inline bool is_flag(Status_t status, Flag_t flag) {
	return status and flag;
}

}  // namespace dude::tf

#endif  // TF_DUDE_CODES_H
