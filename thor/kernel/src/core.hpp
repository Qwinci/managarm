
namespace thor {

int64_t allocAsyncId();

// --------------------------------------------------------
// Debugging and logging
// --------------------------------------------------------

class BochsSink {
public:
	void print(char c);
	void print(const char *str);
};

extern BochsSink infoSink;
extern frigg::LazyInitializer<frigg::DefaultLogger<BochsSink>> infoLogger;

// --------------------------------------------------------
// Memory management
// --------------------------------------------------------

class KernelVirtualAlloc {
public:
	KernelVirtualAlloc();

	uintptr_t map(size_t length);
	void unmap(uintptr_t address, size_t length);

private:
	uintptr_t p_nextPage;
};

typedef frigg::SlabAllocator<KernelVirtualAlloc, frigg::TicketLock> KernelAlloc;
extern frigg::LazyInitializer<PhysicalChunkAllocator> physicalAllocator;
extern frigg::LazyInitializer<KernelVirtualAlloc> kernelVirtualAlloc;
extern frigg::LazyInitializer<KernelAlloc> kernelAlloc;

template<typename T>
using KernelSharedPtr = frigg::SharedPtr<T>;

template<typename T>
using KernelWeakPtr = frigg::WeakPtr<T>;

template<typename T>
using KernelUnsafePtr = frigg::UnsafePtr<T>;

// --------------------------------------------------------
// Kernel data types
// --------------------------------------------------------

enum Error {
	kErrSuccess,
	kErrBufferTooSmall,
	kErrPipeClosed
};

typedef uint64_t Handle;

class Universe;
class Memory;
class AddressSpace;
class Thread;
class Signal;
class EventHub;
class Channel;
class BiDirectionPipe;
class Endpoint;
class Server;
class RdFolder;
class IrqLine;
class IoSpace;

struct CpuContext {
	CpuContext();

	KernelUnsafePtr<Thread> currentThread;
	KernelUnsafePtr<Thread> idleThread;
};

struct Timer {
	Timer(uint64_t deadline)
	: deadline(deadline) { }

	bool operator< (const Timer &other) {
		return deadline < other.deadline;
	}

	uint64_t deadline;

	KernelWeakPtr<Thread> thread;
};

struct SubmitInfo {
	SubmitInfo(int64_t async_id, uintptr_t submit_function,
			uintptr_t submit_object);
	
	int64_t asyncId;
	uintptr_t submitFunction;
	uintptr_t submitObject;
};

// this is a base class for async request classes
struct BaseRequest {
	BaseRequest(KernelSharedPtr<EventHub> event_hub, SubmitInfo submit_info);
	
	KernelSharedPtr<EventHub> eventHub;
	SubmitInfo submitInfo;
};

} // namespace thor

#include "descriptor.hpp"
#include "usermem.hpp"
#include "thread.hpp"
#include "event.hpp"
#include "ipc.hpp"
#include "rd.hpp"
#include "io.hpp"

namespace thor {

// --------------------------------------------------------
// Process related classes
// --------------------------------------------------------

class Universe {
public:
	typedef frigg::TicketLock Lock;
	typedef frigg::LockGuard<frigg::TicketLock> Guard;

	Universe();
	
	Handle attachDescriptor(Guard &guard, AnyDescriptor &&descriptor);

	AnyDescriptor *getDescriptor(Guard &guard, Handle handle);
	
	frigg::Optional<AnyDescriptor> detachDescriptor(Guard &guard, Handle handle);

	Lock lock;

private:
	frigg::Hashmap<Handle, AnyDescriptor,
			frigg::DefaultHasher<Handle>, KernelAlloc> p_descriptorMap;
	Handle p_nextHandle;
};

} // namespace thor

