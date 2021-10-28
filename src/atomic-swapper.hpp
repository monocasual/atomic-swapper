/* -----------------------------------------------------------------------------
 *
 * Atomic Swapper
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2021 Giovanni A. Zuliani | Monocasual Laboratories
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

#include <array>
#include <atomic>

namespace mcl
{
template <typename T>
class AtomicSwapper
{
public:
	class RtLock
	{
		friend AtomicSwapper;

	public:
		RtLock(AtomicSwapper& s)
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
		AtomicSwapper& m_swapper;
	};

	AtomicSwapper()
	{
		static_assert(std::is_assignable_v<T, T>);
	}

	/* isLocked
	Returns true if the busy bit is currently set, that is if the realtime 
	thread is reading its copy of data. */

	bool isLocked() const
	{
		return m_bits.load() & BIT_BUSY;
	}

	/* get (1)
	Returns local data for non-realtime thread. */

	const T& get() const
	{
		return m_data[(m_bits.load() & BIT_INDEX) ^ 1];
	}

	/* get (2)
	As above, non-const version. */

	T& get()
	{
		return const_cast<T&>(static_cast<const AtomicSwapper&>(*this).get());
	}

	/* swap
	Core function: swaps the realtime data with the non-realtime one. Waits for
	the realtime thread until it has finished reading its own copy of data. Only
	then the indexes are swapped atomically. */

	void swap()
	{
		int bits = m_bits.load();

		/* Wait for the realtime thread to finish, i.e. until the BUSY bit 
		becomes zero. Only then, swap indexes. This will let the realtime thread 
		to pick the updated data on its next cycle. */
		int desired;
		do
		{
			bits    = bits & ~BIT_BUSY;               // Expected: current value without busy bit set
			desired = (bits ^ BIT_INDEX) & BIT_INDEX; // Desired: flipped (xor) index
		} while (!m_bits.compare_exchange_weak(bits, desired));

		bits = desired;

		/* After the swap above, m_data[(bits & BIT_INDEX) ^ 1] has become the 
		non-realtime slot and it points to the data previously read by the
		realtime thread. That data is old, so update it: overwrite it with the 
		realtime data in the realtime slot (m_data[bits & BIT_INDEX]) that is 
		currently being read by the realtime thread. */
		m_data[(bits & BIT_INDEX) ^ 1] = m_data[bits & BIT_INDEX];
	}

private:
	static constexpr int BIT_INDEX = (1 << 0); // 0001
	static constexpr int BIT_BUSY  = (1 << 1); // 0010

	/* [realtime] lock
	Marks the data as busy. Used when the realtime thread starts reading its own 
	copy of data. Can't call this directly (it's private), use the scoped lock
	RtLock class above. */

	void rt_lock()
	{
		/* Set the busy bit and also get the current index. */
		m_index = m_bits.fetch_or(BIT_BUSY) & BIT_INDEX;
	}

	/* [realtime] unlock
	Marks the data as free. Used when the realtime thread is done with reading 
	its own copy of data. Can't call this directly (it's private), use the 
	scoped lock	RtLock class above. */

	void rt_unlock()
	{
		m_bits.store(m_index & BIT_INDEX);
	}

	/* [realtime] get
	Get data currently being ready by the realtime thread. Can't call this 
	directly (it's private), use the scoped lock RtLock class above.*/

	const T& rt_get() const
	{
		return m_data[m_bits.load() & BIT_INDEX];
	}

	std::array<T, 2> m_data;
	std::atomic<int> m_bits{0};
	int              m_index{0};
};
} // namespace mcl

#endif
