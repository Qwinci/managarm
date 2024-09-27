#pragma once

#include <qacpi/context.hpp>
#include <qacpi/event.hpp>

#include <uacpi/acpi.h>

#include <frg/manual_box.hpp>
#include <frg/optional.hpp>

namespace thor::acpi {

extern frg::manual_box<qacpi::Context> globalCtx;
extern frg::manual_box<qacpi::events::Context> globalEventCtx;
extern acpi_fadt *globalFadt;

frg::optional<uint64_t> intFromPackageQacpi(qacpi::ObjectRef &pkg, size_t index);

} // namespace thor::acpi
