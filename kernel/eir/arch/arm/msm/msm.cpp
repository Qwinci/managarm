#include <frg/manual_box.hpp>
#include <eir-internal/generic.hpp>

namespace eir {

void debugPrintChar(char) {}

extern "C" [[noreturn]] void eirMsmMain(uintptr_t deviceTreePtr) {
	eirRelocate();
	uintptr_t address = 0xA0000000;
	unsigned int width = 1080;
	unsigned int height = 2400;
	unsigned int pitch = width * 4;
	setFbInfo(reinterpret_cast<void *>(address), (int)width, (int)height, pitch);
	GenericInfo info{
		.deviceTreePtr = deviceTreePtr,
		.cmdline = nullptr,
		.fb {
			.fbAddress = address,
			.fbEarlyWindow = 0,
			.fbPitch = pitch,
			.fbWidth = width,
			.fbHeight = height,
			.fbBpp = 32,
			.fbType = 0
		},
		.debugFlags {},
		.hasFb = true
	};
	eirGenericMain(info);
}

} // namespace eir
