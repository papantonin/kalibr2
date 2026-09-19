#pragma once

#include "kalibr2/types.hpp"
#include <string>

namespace kalibr2 {
void save_diagnostics_report(const std::string& directory,
                             const CameraConfig& camera,
                             const CalibrationResult& result);
} // namespace kalibr2
