#include <limits.h>

#include <frg/allocation.hpp>
#include <frg/manual_box.hpp>
#include <frg/spinlock.hpp>
#include <frg/string.hpp>
#include <thor-internal/cpu-data.hpp>
#include <thor-internal/fiber.hpp>
#include <thor-internal/arch/paging.hpp>
#include <thor-internal/irq.hpp>
#include <thor-internal/kernel_heap.hpp>
#include <thor-internal/pci/pci.hpp>
#include <thor-internal/acpi/acpi.hpp>
#include <async/queue.hpp>
#include <async/recurring-event.hpp>
#include <async/mutex.hpp>
#include <stdlib.h>

#include <qacpi/os.hpp>
#include <qacpi/ns.hpp>

#include "madt.hpp"

using namespace thor;

bool qacpi_os_mutex_create(void **handle) {
	*handle = frg::construct<async::mutex>(*kernelAlloc);
	return true;
}

void qacpi_os_mutex_destroy(void *handle) {
	frg::destruct(*kernelAlloc, static_cast<async::mutex *>(handle));
}

qacpi::Status qacpi_os_mutex_lock(void *handle, uint16_t timeout_ms) {
	auto *mutex = static_cast<async::mutex *>(handle);

	if(timeout_ms == 0xFFFF) {
		KernelFiber::asyncBlockCurrent([mutex]() -> coroutine<void> {
			co_await mutex->async_lock();
		}());
		return qacpi::Status::Success;
	}

	uint16_t sleepTime;
	do {
		if(mutex->try_lock())
			return qacpi::Status::Success;

		sleepTime = frg::min<uint16_t>(timeout_ms, 10);
		timeout_ms -= sleepTime;

		if(sleepTime)
			qacpi_os_sleep(sleepTime);
	} while(timeout_ms);

	return qacpi::Status::TimeOut;
}

qacpi::Status qacpi_os_mutex_unlock(void *handle) {
	auto *mutex = static_cast<async::mutex *>(handle);
	mutex->unlock();
	return qacpi::Status::Success;
}

struct AcpiEvent {
	std::atomic<uint64_t> counter;

	bool tryDecrement() {
		for(;;) {
			auto value = counter.load(std::memory_order::acquire);
			if(value == 0)
				return false;

			if(counter.compare_exchange_strong(value, value - 1,
				std::memory_order::acq_rel, std::memory_order::acquire))
				return true;
		}
	}
};

bool qacpi_os_event_create(void **handle) {
	*handle = frg::construct<AcpiEvent>(*kernelAlloc);
	return true;
}

void qacpi_os_event_destroy(void *handle) {
	frg::destruct(*kernelAlloc, static_cast<AcpiEvent *>(handle));
}

qacpi::Status qacpi_os_event_wait(void *handle, uint16_t timeout_ms) {
	auto *event = static_cast<AcpiEvent *>(handle);

	uint16_t sleepTime;
	do {
		if(event->tryDecrement())
			return qacpi::Status::Success;

		sleepTime = frg::min<uint16_t>(timeout_ms, 10);

		if(timeout_ms != 0xFFFF)
			timeout_ms -= sleepTime;

		if(sleepTime)
			qacpi_os_sleep(sleepTime);
	} while(timeout_ms);

	return qacpi::Status::TimeOut;
}

qacpi::Status qacpi_os_event_signal(void *handle) {
	auto *event = static_cast<AcpiEvent *>(handle);
	event->counter.fetch_add(1, std::memory_order_acq_rel);
	return qacpi::Status::Success;
}

qacpi::Status qacpi_os_event_reset(void *handle) {
	auto *event = static_cast<AcpiEvent *>(handle);
	event->counter.store(0, std::memory_order_release);
	return qacpi::Status::Success;
}

void qacpi_os_trace(const char *str, size_t size) {
	auto msgView = frg::string_view(str, size);
	if (msgView.ends_with("\n"))
		msgView = msgView.sub_string(0, msgView.size() - 1);

	infoLogger() << "thor: " << msgView << frg::endlog;
}

void *qacpi_os_get_tid() {
	return thisFiber();
}

void *qacpi_os_malloc(size_t size) {
	return kernelAlloc->allocate(size);
}

void qacpi_os_free(void *ptr, size_t) {
	kernelAlloc->free(ptr);
}

void qacpi_os_stall(uint64_t us) {
	auto now = systemClockSource()->currentNanos();
	auto deadline = now + us * 1000;

	while (systemClockSource()->currentNanos() < deadline);
}

void qacpi_os_sleep(uint64_t ms) {
	KernelFiber::asyncBlockCurrent(
		generalTimerEngine()->sleepFor(ms * 1000 * 1000)
	);
}

void qacpi_os_fatal(uint8_t type, uint16_t code, uint64_t arg) {
	infoLogger() << "thor: fatal firmware error:"
			<< " type: " << (int)type
			<< " code: " << code
			<< " arg: " << arg << frg::endlog;
}

uint64_t qacpi_os_timer() {
	return systemClockSource()->currentNanos() / 100;
}

void qacpi_os_breakpoint() {
	infoLogger() << "thor: ignoring AML breakpoint" << frg::endlog;
}

static void *acpiMap(uintptr_t phys, size_t size) {
	auto misalign = phys & 0xfff;
	phys &= ~0xfff;
	size = (size + misalign + 0xfff) & ~0xfff;
	auto ptr = KernelVirtualMemory::global().allocate(size);
	for(size_t pg = 0; pg < size; pg += kPageSize)
		KernelPageSpace::global().mapSingle4k((VirtualAddr)ptr + pg, phys + pg,
				page_access::write, CachingMode::null);

	return static_cast<char *>(ptr) + misalign;
}

static void acpiUnmap(void *ptr, size_t size) {
	auto misalign = reinterpret_cast<uintptr_t>(ptr) & 0xfff;
	ptr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(ptr) & ~0xfff);
	size = (size + misalign + 0xfff) & ~0xfff;
	for(size_t pg = 0; pg < size; pg += kPageSize)
		KernelPageSpace::global().unmapSingle4k(reinterpret_cast<VirtualAddr>(ptr) + pg);

	KernelVirtualMemory::global().deallocate(ptr, size);
}

qacpi::Status qacpi_os_mmio_read(uint64_t phys, uint8_t size, uint64_t &res) {
	void *ptr = acpiMap(phys, size);

	switch (size) {
	case 1:
		res = *static_cast<volatile uint8_t *>(ptr);
		break;
	case 2:
		res = *static_cast<volatile uint16_t *>(ptr);
		break;
	case 4:
		res = *static_cast<volatile uint32_t *>(ptr);
		break;
	case 8:
		res = *static_cast<volatile uint64_t *>(ptr);
		break;
	default:
		acpiUnmap(ptr, size);
		return qacpi::Status::InvalidArgs;
	}

	acpiUnmap(ptr, size);

	return qacpi::Status::Success;
}

qacpi::Status qacpi_os_mmio_write(uint64_t phys, uint8_t size, uint64_t value) {
	void *ptr = acpiMap(phys, size);

	switch (size) {
	case 1:
		*static_cast<volatile uint8_t *>(ptr) = value;
		break;
	case 2:
		*static_cast<volatile uint16_t *>(ptr) = value;
		break;
	case 4:
		*static_cast<volatile uint32_t *>(ptr) = value;
		break;
	case 8:
		*static_cast<volatile uint64_t *>(ptr) = value;
		break;
	default:
		acpiUnmap(ptr, size);
		return qacpi::Status::InvalidArgs;
	}

	acpiUnmap(ptr, size);

	return qacpi::Status::Success;
}

#ifdef THOR_ARCH_SUPPORTS_PIO
qacpi::Status qacpi_os_io_read(uint32_t port, uint8_t size, uint64_t &res) {
	uint16_t p = port;

	switch (size) {
	case 1: {
		uint8_t v;
		asm volatile ("inb %1, %0" : "=a"(v) : "d"(p));
		res = v;
		break;
	}
	case 2: {
		uint16_t v;
		asm volatile ("inw %1, %0" : "=a"(v) : "d"(p));
		res = v;
		break;
	}
	case 4: {
		uint32_t v;
		asm volatile ("inl %1, %0" : "=a"(v) : "d"(p));
		res = v;
		break;
	}
	default:
		return qacpi::Status::InvalidArgs;
	}

	return qacpi::Status::Success;
}

qacpi::Status qacpi_os_io_write(uint32_t port, uint8_t size, uint64_t value) {
	uint16_t p = port;

	switch (size) {
	case 1: {
		uint8_t v = value;
		asm volatile ("outb %0, %1" : : "a"(v), "d"(p));
		break;
	}
	case 2: {
		uint16_t v = value;
		asm volatile ("outw %0, %1" : : "a"(v), "d"(p));
		break;
	}
	case 4: {
		uint32_t v = value;
		asm volatile ("outl %0, %1" : : "a"(v), "d"(p));
		break;
	}
	default:
		return qacpi::Status::InvalidArgs;
	}

	return qacpi::Status::Success;
}

#else

qacpi::Status qacpi_os_io_read(uint32_t port, uint8_t size, uint64_t &res) {
	return qacpi::Status::Unsupported;
}

qacpi::Status qacpi_os_io_write(uint32_t port, uint8_t size, uint64_t value) {
	return qacpi::Status::Unsupported;
}

#endif

qacpi::Status qacpi_os_pci_read(
	qacpi::PciAddress address, uint64_t offset,
	uint8_t size, uint64_t &res
) {
	switch (size) {
	case 1: {
		res = pci::readConfigByte(
			address.segment, address.bus, address.device,
			address.function, offset
		);
		break;
	}
	case 2: {
		res = pci::readConfigHalf(
			address.segment, address.bus, address.device,
			address.function, offset
		);
		break;
	}
	case 4: {
		res = pci::readConfigWord(
			address.segment, address.bus, address.device,
			address.function, offset
		);
		break;
	}
	default:
		return qacpi::Status::InvalidArgs;
	}

	return qacpi::Status::Success;
}

qacpi::Status qacpi_os_pci_write(
	qacpi::PciAddress address, uint64_t offset,
	uint8_t size, uint64_t value
) {
	switch (size) {
	case 1: {
		pci::writeConfigByte(
			address.segment, address.bus, address.device,
			address.function, offset, value
		);
		break;
	}
	case 2: {
		pci::writeConfigHalf(
			address.segment, address.bus, address.device,
			address.function, offset, value
		);
		break;
	}
	case 4: {
		pci::writeConfigWord(
			address.segment, address.bus, address.device,
			address.function, offset, value
		);
		break;
	}
	default:
		return qacpi::Status::InvalidArgs;
	}

	return qacpi::Status::Success;
}

void qacpi_os_notify(void *, qacpi::NamespaceNode *node, uint64_t value) {
	acpi::globalEventCtx->on_notify(node, value);
}

struct AcpiWork {
	qacpi::Status (*handler_)(void *);
	void *ctx_;
};
static async::queue<AcpiWork, KernelAlloc> acpiWorkQueue { *kernelAlloc };
static async::recurring_event acpiWorkEvent;
static std::atomic<uint64_t> acpiWorkCounter;

static void workExec(AcpiWork &work) {
	work.handler_(work.ctx_);
	acpiWorkCounter.fetch_sub(1, std::memory_order_acq_rel);
	acpiWorkEvent.raise();
}

void thor::acpi::initGlue() {
	KernelFiber::run([] {
		while (true) {
			auto work = KernelFiber::asyncBlockCurrent(
				acpiWorkQueue.async_get());
			workExec(*work);
		}
		}, &getCpuData(0)->scheduler);
}

struct SciDevice final : IrqSink {
	bool (*handler)(void *);
	void *ctx;

	SciDevice()
	: IrqSink{frg::string<KernelAlloc>{*kernelAlloc, "acpi-sci"}} { }

	IrqStatus raise() override {
		return handler(ctx) ?
			IrqStatus::acked : IrqStatus::nacked;
	}
};

frg::manual_box<SciDevice> sciDevice;

qacpi::Status qacpi_os_install_sci_handler(uint32_t irq, bool (*handler)(void *arg), void *arg, void **handle) {
	auto sciOverride = resolveIsaIrq(irq);
	configureIrq(sciOverride);

	sciDevice.initialize();
	sciDevice->handler = handler;
	sciDevice->ctx = arg;

#ifdef __x86_64__
	IrqPin::attachSink(getGlobalSystemIrq(sciOverride.gsi), sciDevice.get());
#endif

	*handle = &sciDevice;
	return qacpi::Status::Success;
}

void qacpi_os_uninstall_sci_handler(uint32_t, void *) {}

qacpi::Status qacpi_os_queue_work(qacpi::Status (*fn)(void *arg), void *arg) {
	acpiWorkCounter.fetch_add(1, std::memory_order_acq_rel);

	acpiWorkQueue.put({ fn, arg });

	return qacpi::Status::Success;
}
