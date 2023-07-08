/* -----------------------------------------------------------------------------
 *
 * Atomic Swapper
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2021 - 2022 Giovanni A. Zuliani | Monocasual Laboratories
 *
 * This file is part of Atomic Swapper.
 *
 * Atomic Swapper is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Atomic Swapper is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Atomic Swapper. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------- */

#ifndef MONOCASUAL_ATOMIC_SWAPPER_H
#define MONOCASUAL_ATOMIC_SWAPPER_H

#include <atomic>
#include <cassert>
#include <cstdint>
#include <mutex>
#include <vector>

namespace mcl
{
template <typename T, std::size_t Size>
class AtomicSwapper
{
public:
	using ThreadIndex = uint8_t;
	using Revision    = uint32_t;
	using Bitfield    = uint32_t;

	/* Thread
	A struct containing information on the current thread. */

	struct Thread
	{
		ThreadIndex index{0};
		Revision    revision{0};
		bool        registered{false};
		bool        realtime{false};
		std::string name;
	};

	/* RtLock
	A class for scoped-locking the real-time thread. */

	class RtLock
	{
		friend AtomicSwapper;

	public:
		RtLock(const AtomicSwapper& s)
		: m_swapper(s)
		{
			m_swapper.rt_lock();
		}

		~RtLock()
		{
			m_swapper.rt_unlock();
		}

		const T& get() const
		{
			return m_swapper.rt_get();
		}

	private:
		const AtomicSwapper& m_swapper;
	};

	/* thread
	The current thread info. Each thread has its own copy (thread_local). */

	inline static thread_local Thread thread;

	/* AtomicSwapper (ctor)
	The m_bits field is initialized as 0x0000[m_data.size() - 1] so that the
	real-time thread index will be the last slot in the m_data array. This trick
	allows you to register other threads easily starting from 0, without stepping
	on the real-time one's toes. */

	AtomicSwapper()
	: m_data(Size, T())
	, m_bits(m_data.size() - 1)
	{
		static_assert(std::is_assignable_v<T, T>);
		static_assert(std::atomic<Bitfield>::is_always_lock_free);
		static_assert(std::atomic<ThreadIndex>::is_always_lock_free);
		static_assert(Size < RT_MAX_SIZE);
	}

	/* isRtLocked
	Returns true if the busy bit is currently set, that is if the realtime 
	thread is reading its copy of data. */

	bool isRtLocked() const
	{
		return m_bits.load() & RT_BUSY_BIT;
	}

	/* registerThread
	Registers the thread invoking this function as a new one. Does nothing if
	already registered. Returns false if there's no space left. */

	bool registerThread(const std::string& name, bool realtime) const
	{
		if (thread.registered)
			return true;

		/* The critical section starts here. Only one thread at a time can be
		registered. */

		std::scoped_lock lock(m_mutex);

		if (m_lastThreadIndex >= m_data.size() - 1) // Not enough space
			return false;

		/* Set a new thread index. This is not super thread-safe, because an
		existing thread might change the real-time index while the new one performs 
		the two instructions below. However, new threads should be registered on 
		startup, when index swapping is less likely to occur. */

		const ThreadIndex rtIndex  = rt_getIndex(m_bits.load());
		const ThreadIndex newIndex = realtime ? rtIndex : getNewNonRtThreadIndex(rtIndex);

		thread.index      = newIndex;
		thread.revision   = 0;
		thread.registered = true;
		thread.realtime   = realtime;
		thread.name       = name;

		return true;
	}

	/* get
	Returns local data for a non-realtime thread. */

	const T& get() const
	{
		assert(thread.registered);
		assert(!thread.realtime);

		const Bitfield bits       = m_bits.load();
		const Revision rtRevision = rt_getRevision(bits);

		updateLocalDataToLatest(bits, rtRevision);

		return m_data[thread.index];
	}

	T& get()
	{
		return const_cast<T&>(std::as_const(*this).get());
	}

	/* swap
	Core function: sets the current thread index as the new realtime one. Waits 
	for the realtime thread until it has finished reading its own copy of data. 
	Only then the indexes are swapped atomically. */

	void swap()
	{
		assert(thread.registered);
		assert(!thread.realtime);

		const Bitfield    bits        = m_bits.load();
		const ThreadIndex oldRtIndex  = rt_getIndex(bits);
		const ThreadIndex newRtIndex  = thread.index;
		const Revision    oldRevision = rt_getRevision(bits);
		const Revision    newRevision = (oldRevision + 1) % RT_REVISION_SIZE; // make it wrap around when > RT_REVISION_SIZE

		/* Make sure the caller has the most up-to-date data, otherwise it would
		give the realtime thread an old version of it. */
		updateLocalDataToLatest(bits, oldRevision);

		/* Wait for the realtime thread to finish, i.e. until the BUSY bit 
		becomes zero. Only then, swap indexes. This will let the realtime thread 
		to pick the updated data on its next cycle. Concretely, the loop will 
		spin until the value of m_bits is equal to the expected one, that is
		the current m_bits value WITHOUT the busy bit (which means the realtime
		data is finally unlocked). When it happens, the compare_exchange_weak
		instruction sets m_bits to a new value: m_bits WITHOUT the busy bit AND
		the index changed (which is the actual swap). */
		Bitfield expected;
		Bitfield desired;
		do
		{
			expected = rt_setBusyBitOff(bits);                  // Current value without busy bit set
			desired  = rt_setRevision(newRtIndex, newRevision); // The new real-time index merged with the new revision number
		} while (!m_bits.compare_exchange_weak(expected, desired));

		/* The current thread that requested the swap now gets the old index,
		since the new one has been given to the real time thread. */
		thread.index = oldRtIndex;

		/* Give the current thread the old revision: it points to old data that 
		needs to be updated during the next get() call. */
		thread.revision = oldRevision;
	}

	/* debug */

	void debug() const
	{
		const Bitfield bits = m_bits.load();

		printf("[AtomicSwapper] data size = %lu\n", m_data.size());
		printf("[AtomicSwapper] bits = 0x%X\n", bits);
		printf("[AtomicSwapper] realtime index = %d (0x%X)\n", rt_getIndex(bits), rt_getIndex(bits));
		printf("[AtomicSwapper] realtime revision = %d (0x%X)\n", rt_getRevision(bits), rt_getRevision(bits));
		printf("[AtomicSwapper] thread name = %s\n", thread.name.c_str());
		printf("[AtomicSwapper] thread index = %d\n", thread.index);
		printf("[AtomicSwapper] thread revision = %d\n", thread.revision);
		printf("[AtomicSwapper] thread is registered = %d\n", thread.registered);
		printf("[AtomicSwapper] thread is realtime = %d\n", thread.realtime);
	}

private:
	static constexpr std::size_t RT_REVISION_SIZE   = 0xFFFFF;                                // 20 bits (dec = 1048576)
	static constexpr std::size_t RT_REVISION_OFFSET = 12;                                     //                        |---------------
	static constexpr Bitfield    RT_REVISION_MASK   = RT_REVISION_SIZE << RT_REVISION_OFFSET; // 1111 1111 1111 1111 1111 0000 0000 0000
	static constexpr Bitfield    RT_INDEX_MASK      = 0xFF;                                   // 0000 0000 0000 0000 0000 0000 1111 1111
	static constexpr Bitfield    RT_BUSY_BIT        = 0x100;                                  // 0000 0000 0000 0000 0000 0001 0000 0000
	static constexpr Bitfield    RT_MAX_SIZE        = RT_INDEX_MASK;

	/* getNewNonRtThreadIndex
	Returns a new non-realtime thread index, making sure the new index is different
	than the real-time one. */

	ThreadIndex getNewNonRtThreadIndex(ThreadIndex rtIndex) const
	{
		ThreadIndex newIndex = m_lastThreadIndex++;
		if (newIndex == rtIndex)
			newIndex = m_lastThreadIndex++;

		assert(m_lastThreadIndex <= m_data.size() - 1);

		return newIndex;
	}

	/* [realtime] get
	Get data currently being ready by the realtime thread. Can't call this 
	directly (it's private), use the scoped lock RtLock class above.*/

	const T& rt_get() const
	{
		assert(thread.registered);
		assert(thread.realtime);

		return m_data[rt_getIndex(m_bits.load())];
	}

	/* [realtime] getIndex
	Returns the current real-time thread index. */

	ThreadIndex rt_getIndex(Bitfield bits) const
	{
		return bits & RT_INDEX_MASK;
	}

	Revision rt_getRevision(Bitfield bits) const
	{
		return (bits & RT_REVISION_MASK) >> RT_REVISION_OFFSET;
	};

	Bitfield rt_setRevision(Bitfield bits, Revision r) const
	{
		return (bits & ~RT_REVISION_MASK) | (r << RT_REVISION_OFFSET); // Clear and set
	};

	Bitfield rt_setBusyBitOff(Bitfield bits) const
	{
		return bits & ~RT_BUSY_BIT;
	}

	/* [realtime] lock
	Marks the data as busy. Used when the realtime thread starts reading its own 
	copy of data. Can't call this directly (it's private), use the scoped lock
	RtLock class above. */

	void rt_lock() const
	{
		assert(thread.registered);
		assert(thread.realtime);

		/* Set the busy bit by or-ing the current m_bits value with BIT_BUSY.
		This is done with the instruction m_bits.fetch_or(BIT_BUSY), which 
		returns m_bits with the busy bit on. Then get that value in return and 
		save it to the temp varible m_temp. */
		m_temp = m_bits.fetch_or(RT_BUSY_BIT);
	}

	/* [realtime] unlock
	Marks the data as free. Used when the realtime thread is done with reading 
	its own copy of data. Can't call this directly (it's private), use the 
	scoped lock	RtLock class above. */

	void rt_unlock() const
	{
		assert(thread.registered);
		assert(thread.realtime);

		/* Store m_temp into m_bits by and-ing it with ~RT_BUSY_BIT. This effectively
		strips away the busy bit an leaves only the index part as before. */
		m_bits.store(rt_setBusyBitOff(m_temp));
	}

	/* updateLocalDataToLatest
	Updates the local data and the local revision number if the local data is
	older than the realtime one. Call this whenever you want the caller thread
	to get the freshest copy of the data currently in use by the realtime thread. */

	void updateLocalDataToLatest(Bitfield bits, Revision rtRevision) const
	{
		assert(thread.registered);
		assert(!thread.realtime);

		if (thread.revision == rtRevision)
			return;
		m_data[thread.index] = m_data[rt_getIndex(bits)];
		thread.revision      = rtRevision;
	}

	/* m_data
	Vector containing 'Size' copies of data to be swapped. */

	mutable std::vector<T> m_data;

	/* m_bits
	A bitfield that groups the busy bit, the revision number and the real-time 
	index in a single atomic variable. Actually initialized in the constructor 
	(see note there). 
	
	rrrr rrrr rrrr rrrr rrrr 000b iiii iiii

	r = revision (5 bytes)
	b = busy bit (1 byte)
	i = index    (2 bytes)
	*/

	mutable std::atomic<Bitfield> m_bits{0};

	/* m_temp
	Temporary copy of m_bits with the busy bit set on, used when locking and
	unlocking the realtime thread. */

	mutable Bitfield m_temp{0};

	/* m_lastThreadIndex
	The last index generated when registering a thread. Must not be greater
	than m_data.size(). */

	mutable ThreadIndex m_lastThreadIndex{0};

	/* m_mutex
	A mutex used when registering new threads. */

	mutable std::mutex m_mutex;
};
} // namespace mcl

#endif
