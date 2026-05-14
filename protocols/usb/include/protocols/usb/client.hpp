#pragma once

#include <helix/ipc.hpp>
#include "api.hpp"

namespace protocols::usb {

Device connect(helix::UniqueLane lane);

std::unique_ptr<DeviceController> connectDeviceController(helix::UniqueLane lane);

} // namespace protocols::usb
