/*
 * Copyright 2001-2020, Axel Dörfler, axeld@pinc-software.de.
 * This file may be used under the terms of the MIT License.
 */


//! Block bitmap handling and allocation policies


#include "BlockAllocator.h"

#include "Debug.h"
#include "Inode.h"
#include "Volume.h"


// Things the BlockAllocator should do:

// - find a range of blocks of a certain size nearby a specific position
// - allocating an unsharp range of blocks for pre-allocation
// - free blocks
// - know how to deal with each allocation, special handling for directories,
//   files, symlinks, etc. (type sensitive allocation policies)

// What makes the code complicated is the fact that we are not just reading
// in the whole bitmap and operate on that in memory - e.g. a 13 GB partition
// with a block size of 2048 bytes already has a 800kB bitmap, and the size
// of partitions will grow even more - so that's not an option.
// Instead we are reading in every block when it's used - since an allocation
// group can span several blocks in the block bitmap, the AllocationBlock
// class is there to make handling those easier.

// The current implementation is only slightly optimized and could probably
// be improved a lot. Furthermore, the allocation policies used here should
// have some real world tests.

#if BFS_TRACING && !defined(FS_SHELL)
namespace BFSBlockTracing {


class Allocate : public AbstractTraceEntry {
public:
	Allocate(block_run run)
		:
		fRun(run)
	{
		Initialized();
	}

	virtual void AddDump(TraceOutput& out)
	{
		out.Print("bfs:alloc %" B_PRId32 ".%" B_PRIu16 ".%" B_PRIu16,
			fRun.AllocationGroup(), fRun.Start(), fRun.Length());
	}

	const block_run& Run() const { return fRun; }

private:
			block_run			fRun;
};


class Free : public AbstractTraceEntry {
public:
	Free(block_run run)
		:
		fRun(run)
	{
		Initialized();
	}

	virtual void AddDump(TraceOutput& out)
	{
		out.Print("bfs:free %" B_PRId32 ".%" B_PRIu16 ".%" B_PRIu16,
			fRun.AllocationGroup(), fRun.Start(), fRun.Length());
	}

	const block_run& Run() const { return fRun; }

private:
			block_run			fRun;
};


static uint32
checksum(const uint8* data, size_t size)
{
	const uint32* data4 = (const uint32*)data;
	uint32 sum = 0;
	while (size >= 4) {
		sum += *data4;
		data4++;
		size -= 4;
	}
	return sum;
}


class Block : public AbstractTraceEntry {
public:
	Block(const char* label, off_t blockNumber, const uint8* data,
			size_t size, uint32 start = 0, uint32 length = 0)
		:
		fBlock(blockNumber),
		fData(data),
		fStart(start),
		fLength(length),
		fLabel(label)
	{
		fSum = checksum(data, size);
		Initialized();
	}

	virtual void AddDump(TraceOutput& out)
	{
		out.Print("bfs:%s: block %lld (%p), sum %lu, s/l %lu/%lu", fLabel,
			fBlock, fData, fSum, fStart, fLength);
	}

private:
			off_t				fBlock;
			const uint8*		fData;
			uint32				fStart;
			uint32				fLength;
			uint32				fSum;
			const char*			fLabel;
};


class BlockChange : public AbstractTraceEntry {
public:
	BlockChange(const char* label, int32 block, uint32 oldData, uint32 newData)
		:
		fBlock(block),
		fOldData(oldData),
		fNewData(newData),
		fLabel(label)
	{
		Initialized();
	}

	virtual void AddDump(TraceOutput& out)
	{
		out.Print("bfs:%s: block %ld, %08lx -> %08lx", fLabel,
			fBlock, fOldData, fNewData);
	}

private:
			int32				fBlock;
			uint32				fOldData;
			uint32				fNewData;
			const char*			fLabel;
};


}	// namespace BFSBlockTracing

#	define T(x) new(std::nothrow) BFSBlockTracing::x;
#else
#	define T(x) ;
#endif

#ifdef DEBUG_ALLOCATION_GROUPS
#	define CHECK_ALLOCATION_GROUP(group) _CheckGroup(group)
#else
#	define CHECK_ALLOCATION_GROUP(group) ;
#endif


class AllocationBlock : public CachedBlock {
public:
	AllocationBlock(Volume* volume);

	inline void Allocate(uint16 start, uint16 numBlocks);
	inline void Free(uint16 start, uint16 numBlocks);
	inline bool IsUsed(uint16 block);

	status_t SetTo(AllocationGroup& group, uint16 block);
	status_t SetToWritable(Transaction& transaction, AllocationGroup& group,
		uint16 block);

	uint32 NumBlockBits() const { return fNumBits; }
	uint32& Chunk(int32 index) { return ((uint32*)fBlock)[index]; }
	uint8* Block() const { return (uint8*)fBlock; }

private:
	uint32	fNumBits;
#ifdef DEBUG
	bool	fWritable;
#endif
};


class AllocationGroup {
public:
	AllocationGroup();

	void AddFreeRange(int32 start, int32 blocks);
	bool IsFull() const { return fFreeBits == 0; }

	status_t Allocate(Transaction& transaction, uint16 start, int32 length);
	status_t Free(Transaction& transaction, uint16 start, int32 length);

	uint32 NumBits() const { return fNumBits; }
	uint32 NumBitmapBlocks() const { return fNumBitmapBlocks; }
	int32 Start() const { return fStart; }

private:
	friend class BlockAllocator;

	uint32	fNumBits;
	uint32	fNumBitmapBlocks;
	int32	fStart;
	int32	fFirstFree;
	int32	fFreeBits;

	int32	fLargestStart;
	int32	fLargestLength;
	bool	fLargestValid;
};


AllocationBlock::AllocationBlock(Volume* volume)
	: CachedBlock(volume)
{
}


status_t
AllocationBlock::SetTo(AllocationGroup& group, uint16 block)
{
	// 8 blocks per byte
	fNumBits = fVolume->BlockSize() << 3;
	// the last group may have less bits than the others
	if ((block + 1) * fNumBits > group.NumBits())
		fNumBits = group.NumBits() % fNumBits;

#ifdef DEBUG
	fWritable = false;
#endif
	return CachedBlock::SetTo(group.Start() + block);
}


status_t
AllocationBlock::SetToWritable(Transaction& transaction, AllocationGroup& group,
	uint16 block)
{
	// 8 blocks per byte
	fNumBits = fVolume->BlockSize() << 3;
	// the last group may have less bits in the last block
	if ((block + 1) * fNumBits > group.NumBits())
		fNumBits = group.NumBits() % fNumBits;

#ifdef DEBUG
	fWritable = true;
#endif
	return CachedBlock::SetToWritable(transaction, group.Start() + block);
}


bool
AllocationBlock::IsUsed(uint16 block)
{
	if (block > fNumBits)
		return true;

	// the block bitmap is accessed in 32-bit chunks
	return Chunk(block >> 5) & HOST_ENDIAN_TO_BFS_INT32(1UL << (block % 32));
}


void
AllocationBlock::Allocate(uint16 start, uint16 numBlocks)
{
	ASSERT(start < fNumBits);
	ASSERT(uint32(start + numBlocks) <= fNumBits);
#ifdef DEBUG
	ASSERT(fWritable);
#endif

	T(Block("b-alloc-in", fBlockNumber, fBlock, fVolume->BlockSize(),
		start, numBlocks));

	int32 block = start >> 5;

	while (numBlocks > 0) {
		uint32 mask = 0;
		for (int32 i = start % 32; i < 32 && numBlocks; i++, numBlocks--)
			mask |= 1UL << i;

		T(BlockChange("b-alloc", block, Chunk(block),
			Block(block) | HOST_ENDIAN_TO_BFS_INT32(mask)));

#if KDEBUG
		// check for already set blocks
		if (HOST_ENDIAN_TO_BFS_INT32(mask) & ((uint32*)fBlock)[block]) {
			FATAL(("AllocationBlock::Allocate(): some blocks are already "
				"allocated, start = %u, numBlocks = %u\n", start, numBlocks));
			panic("blocks already set!");
		}
#endif

		Chunk(block++) |= HOST_ENDIAN_TO_BFS_INT32(mask);
		start = 0;
	}
	T(Block("b-alloc-out", fBlockNumber, fBlock, fVolume->BlockSize(),
		start, numBlocks));
}


void
AllocationBlock::Free(uint16 start, uint16 numBlocks)
{
	ASSERT(start < fNumBits);
	ASSERT(uint32(start + numBlocks) <= fNumBits);
#ifdef DEBUG
	ASSERT(fWritable);
#endif

	int32 block = start >> 5;

	while (numBlocks > 0) {
		uint32 mask = 0;
		for (int32 i = start % 32; i < 32 && numBlocks; i++, numBlocks--)
			mask |= 1UL << (i % 32);

		T(BlockChange("b-free", block, Chunk(block),
			Block(block) & HOST_ENDIAN_TO_BFS_INT32(~mask)));

		Chunk(block++) &= HOST_ENDIAN_TO_BFS_INT32(~mask);
		start = 0;
	}
}


//	#pragma mark -


/*!	The allocation groups are created and initialized in
	BlockAllocator::Initialize() and BlockAllocator::InitializeAndClearBitmap()
	respectively.
*/
AllocationGroup::AllocationGroup()
	:
	fFirstFree(-1),
	fFreeBits(0),
	fLargestValid(false)
{
}


void
AllocationGroup::AddFreeRange(int32 start, int32 blocks)
{
	//D(if (blocks > 512)
	//	PRINT(("range of %ld blocks starting at %ld\n",blocks,start)));

	if (fFirstFree == -1)
		fFirstFree = start;

	if (!fLargestValid || fLargestLength < blocks) {
		fLargestStart = start;
		fLargestLength = blocks;
		fLargestValid = true;
	}

	fFreeBits += blocks;
}


/*!	Allocates the specified run in the allocation group.
	Doesn't check if the run is valid or already allocated partially, nor
	does it maintain the free ranges hints or the volume's used blocks count.
	It only does the low-level work of allocating some bits in the block bitmap.
	Assumes that the block bitmap lock is hold.
*/
status_t
AllocationGroup::Allocate(Transaction& transaction, uint16 start, int32 length)
{
	ASSERT(start + length <= (int32)fNumBits);

	// Update the allocation group info
	// TODO: this info will be incorrect if something goes wrong later
	// Note, the fFirstFree block doesn't have to be really free
	if (start == fFirstFree)
		fFirstFree = start + length;
	fFreeBits -= length;

	if (fLargestValid) {
		bool cut = false;
		if (fLargestStart == start) {
			// cut from start
			fLargestStart += length;
			fLargestLength -= length;
			cut = true;
		} else if (start > fLargestStart
			&& start < fLargestStart + fLargestLength) {
			// cut from end
			fLargestLength = start - fLargestStart;
			cut = true;
		}
		if (cut && (fLargestLength < fLargestStart
				|| fLargestLength
						< (int32)fNumBits - (fLargestStart + fLargestLength))) {
			// might not be the largest block anymore
			fLargestValid = false;
		}
	}

	Volume* volume = transaction.GetVolume();

	// calculate block in the block bitmap and position within
	uint32 bitsPerBlock = volume->BlockSize() << 3;
	uint32 block = start / bitsPerBlock;
	start = start % bitsPerBlock;

	AllocationBlock cached(volume);

	while (length > 0) {
		if (cached.SetToWritable(transaction, *this, block) < B_OK) {
			fLargestValid = false;
			RETURN_ERROR(B_IO_ERROR);
		}

		uint32 numBlocks = length;
		if (start + numBlocks > cached.NumBlockBits())
			numBlocks = cached.NumBlockBits() - start;

		cached.Allocate(start, numBlocks);

		length -= numBlocks;
		start = 0;
		block++;
	}

	return B_OK;
}


/*!	Frees the specified run in the allocation group.
	Doesn't check if the run is valid or was not completely allocated, nor
	does it maintain the free ranges hints or the volume's used blocks count.
	It only does the low-level work of freeing some bits in the block bitmap.
	Assumes that the block bitmap lock is hold.
*/
status_t
AllocationGroup::Free(Transaction& transaction, uint16 start, int32 length)
{
	ASSERT(start + length <= (int32)fNumBits);

	// Update the allocation group info
	// TODO: this info will be incorrect if something goes wrong later
	if (fFirstFree > start)
		fFirstFree = start;
	fFreeBits += length;

	// The range to be freed cannot be part of the valid largest range
	ASSERT(!fLargestValid || start + length <= fLargestStart
		|| start > fLargestStart);

	if (fLargestValid
		&& (start + length == fLargestStart
			|| fLargestStart + fLargestLength == start
			|| (start < fLargestStart && fLargestStart > fLargestLength)
			|| (start > fLargestStart
				&& (int32)fNumBits - (fLargestStart + fLargestLength)
						> fLargestLength))) {
		fLargestValid = false;
	}

	Volume* volume = transaction.GetVolume();

	// calculate block in the block bitmap and position within
	uint32 bitsPerBlock = volume->BlockSize() << 3;
	uint32 block = start / bitsPerBlock;
	start = start % bitsPerBlock;

	AllocationBlock cached(volume);

	while (length > 0) {
		if (cached.SetToWritable(transaction, *this, block) < B_OK)
			RETURN_ERROR(B_IO_ERROR);

		T(Block("free-1", block, cached.Block(), volume->BlockSize()));
		uint16 freeLength = length;
		if (uint32(start + length) > cached.NumBlockBits())
			freeLength = cached.NumBlockBits() - start;

		cached.Free(start, freeLength);

		length -= freeLength;
		start = 0;
		T(Block("free-2", block, cached.Block(), volume->BlockSize()));
		block++;
	}
	return B_OK;
}


//	#pragma mark -


BlockAllocator::BlockAllocator(Volume* volume)
	:
	fVolume(volume),
	fGroups(NULL)
	//fCheckBitmap(NULL),
	//fCheckCookie(NULL)
{
	recursive_lock_init(&fLock, "bfs allocator");
}


BlockAllocator::~BlockAllocator()
{
	recursive_lock_destroy(&fLock);
	delete[] fGroups;
}


status_t
BlockAllocator::Initialize(bool full)
{
	fNumGroups = fVolume->AllocationGroups();
	fBlocksPerGroup = fVolume->SuperBlock().BlocksPerAllocationGroup();
	fNumBitmapBlocks = fVolume->NumBitmapBlocks();

	fGroups = new(std::nothrow) AllocationGroup[fNumGroups];
	if (fGroups == NULL)
		return B_NO_MEMORY;

	if (!full)
		return B_OK;

	recursive_lock_lock(&fLock);
		// the lock will be released by the _Initialize() method

	thread_id id = spawn_kernel_thread((thread_func)BlockAllocator::_Initialize,
		"bfs block allocator", B_LOW_PRIORITY, this);
	if (id < B_OK)
		return _Initialize(this);

	recursive_lock_transfer_lock(&fLock, id);

	return resume_thread(id);
}


status_t
BlockAllocator::InitializeAndClearBitmap(Transaction& transaction)
{
	status_t status = Initialize(false);
	if (status != B_OK)
		return status;

	uint32 numBits = 8 * fBlocksPerGroup * fVolume->BlockSize();
	uint32 blockShift = fVolume->BlockShift();

	uint32* buffer = (uint32*)malloc(numBits >> 3);
	if (buffer == NULL)
		RETURN_ERROR(B_NO_MEMORY);

	memset(buffer, 0, numBits >> 3);

	off_t offset = 1;
		// the bitmap starts directly after the superblock

	// initialize the AllocationGroup objects and clear the on-disk bitmap

	for (int32 i = 0; i < fNumGroups; i++) {
		if (write_pos(fVolume->Device(), offset << blockShift, buffer,
				fBlocksPerGroup << blockShift) < B_OK) {
			free(buffer);
			return B_ERROR;
		}

		// the last allocation group may contain less blocks than the others
		if (i == fNumGroups - 1) {
			fGroups[i].fNumBits = fVolume->NumBlocks() - i * numBits;
			fGroups[i].fNumBitmapBlocks = 1 + ((fGroups[i].NumBits() - 1)
				>> (blockShift + 3));
		} else {
			fGroups[i].fNumBits = numBits;
			fGroups[i].fNumBitmapBlocks = fBlocksPerGroup;
		}
		fGroups[i].fStart = offset;
		fGroups[i].fFirstFree = fGroups[i].fLargestStart = 0;
		fGroups[i].fFreeBits = fGroups[i].fLargestLength = fGroups[i].fNumBits;
		fGroups[i].fLargestValid = true;

		offset += fBlocksPerGroup;
	}
	free(buffer);

	// reserve the boot block, the log area, and the block bitmap itself
	uint32 reservedBlocks = fVolume->ToBlock(fVolume->Log()) + fVolume->Log().Length();
	uint32 blocksToReserve = reservedBlocks;
	for (int32 i = 0; i < fNumGroups; i++) {
		int32 reservedBlocksInGroup = min_c(blocksToReserve, numBits);
		if (fGroups[i].Allocate(transaction, 0, reservedBlocksInGroup) < B_OK) {
			FATAL(("could not allocate reserved space for block bitmap/log!\n"));
			return B_ERROR;
		}
		blocksToReserve -= reservedBlocksInGroup;
		if (blocksToReserve == 0)
			break;
	}
	fVolume->SuperBlock().used_blocks
		= HOST_ENDIAN_TO_BFS_INT64(reservedBlocks);

	return B_OK;
}


status_t
BlockAllocator::_Initialize(BlockAllocator* allocator)
{
	// The lock must already be held at this point
	RecursiveLocker locker(allocator->fLock, true);

	Volume* volume = allocator->fVolume;
	uint32 blocks = allocator->fBlocksPerGroup;
	uint32 blockShift = volume->BlockShift();
	off_t freeBlocks = 0;

	uint32* buffer = (uint32*)malloc(blocks << blockShift);
	if (buffer == NULL)
		RETURN_ERROR(B_NO_MEMORY);

	AllocationGroup* groups = allocator->fGroups;
	off_t offset = 1;
	uint32 bitsPerGroup = 8 * (blocks << blockShift);
	int32 numGroups = allocator->fNumGroups;

	for (int32 i = 0; i < numGroups; i++) {
		if (read_pos(volume->Device(), offset << blockShift, buffer,
				blocks << blockShift) < B_OK)
			break;

		// the last allocation group may contain less blocks than the others
		if (i == numGroups - 1) {
			groups[i].fNumBits = volume->NumBlocks() - i * bitsPerGroup;
			groups[i].fNumBitmapBlocks = 1 + ((groups[i].NumBits() - 1)
				>> (blockShift + 3));
		} else {
			groups[i].fNumBits = bitsPerGroup;
			groups[i].fNumBitmapBlocks = blocks;
		}
		groups[i].fStart = offset;

		// finds all free ranges in this allocation group
		int32 start = -1, range = 0;
		int32 numBits = groups[i].fNumBits, bit = 0;
		int32 count = (numBits + 31) / 32;

		for (int32 k = 0; k < count; k++) {
			for (int32 j = 0; j < 32 && bit < numBits; j++, bit++) {
				if (buffer[k] & (1UL << j)) {
					// block is in use
					if (range > 0) {
						groups[i].AddFreeRange(start, range);
						range = 0;
					}
				} else if (range++ == 0) {
					// block is free, start new free range
					start = bit;
				}
			}
		}
		if (range)
			groups[i].AddFreeRange(start, range);

		freeBlocks += groups[i].fFreeBits;

		offset += blocks;
	}
	free(buffer);

	// check if block bitmap and log area are reserved
	uint32 reservedBlocks = volume->ToBlock(volume->Log()) + volume->Log().Length();

	if (allocator->CheckBlocks(0, reservedBlocks) != B_OK) {
		if (volume->IsReadOnly()) {
			FATAL(("Space for block bitmap or log area is not reserved "
				"(volume is mounted read-only)!\n"));
		} else {
			Transaction transaction(volume, 0);
			if (groups[0].Allocate(transaction, 0, reservedBlocks) != B_OK) {
				FATAL(("Could not allocate reserved space for block "
					"bitmap/log!\n"));
				volume->Panic();
			} else {
				transaction.Done();
				FATAL(("Space for block bitmap or log area was not "
					"reserved!\n"));
			}
		}
	}

	off_t usedBlocks = volume->NumBlocks() - freeBlocks;
	if (volume->UsedBlocks() != usedBlocks) {
		// If the disk in a dirty state at mount time, it's
		// normal that the values don't match
		INFORM(("volume reports %" B_PRIdOFF " used blocks, correct is %"
			B_PRIdOFF "\n", volume->UsedBlocks(), usedBlocks));
		volume->SuperBlock().used_blocks = HOST_ENDIAN_TO_BFS_INT64(usedBlocks);
	}

	return B_OK;
}


void
BlockAllocator::Uninitialize()
{
	// We only have to make sure that the initializer thread isn't running
	// anymore.
	recursive_lock_lock(&fLock);
}


/*!	Tries to allocate between \a minimum, and \a maximum blocks starting
	at group \a groupIndex with offset \a start. The resulting allocation
	is put into \a run.

	The number of allocated blocks is always a multiple of \a minimum which
	has to be a power of two value.
*/
status_t
BlockAllocator::AllocateBlocks(Transaction& transaction, int32 groupIndex,
	uint16 start, uint16 maximum, uint16 minimum, block_run& run)
{
	if (maximum == 0)
		return B_BAD_VALUE;

	FUNCTION_START(("group = %" B_PRId32 ", start = %" B_PRIu16
		", maximum = %" B_PRIu16 ", minimum = %" B_PRIu16 "\n",
		groupIndex, start, maximum, minimum));

	AllocationBlock cached(fVolume);
	RecursiveLocker lock(fLock);

	uint32 bitsPerFullBlock = fVolume->BlockSize() << 3;

	// Find the block_run that can fulfill the request best
	int32 bestGroup = -1;
	int32 bestStart = -1;
	int32 bestLength = -1;

	for (int32 i = 0; i < fNumGroups + 1; i++, groupIndex++, start = 0) {
		groupIndex = groupIndex % fNumGroups;
		AllocationGroup& group = fGroups[groupIndex];

		CHECK_ALLOCATION_GROUP(groupIndex);

		if (start >= group.NumBits() || group.IsFull())
			continue;

		// The wanted maximum is smaller than the largest free block in the
		// group or already smaller than the minimum

		if (start < group.fFirstFree)
			start = group.fFirstFree;

		if (group.fLargestValid) {
			if (group.fLargestLength < bestLength)
				continue;

			if (group.fLargestStart >= start) {
				if (group.fLargestLength >= bestLength) {
					bestGroup = groupIndex;
					bestStart = group.fLargestStart;
					bestLength = group.fLargestLength;

					if (bestLength >= maximum)
						break;
				}

				// We know everything about this group we have to, let's skip
				// to the next
				continue;
			}
		}

		// There may be more than one block per allocation group - and
		// we iterate through it to find a place for the allocation.
		// (one allocation can't exceed one allocation group)

		uint32 block = start / (fVolume->BlockSize() << 3);
		int32 currentStart = 0, currentLength = 0;
		int32 groupLargestStart = -1;
		int32 groupLargestLength = -1;
		int32 currentBit = start;
		bool canFindGroupLargest = start == 0;

		for (; block < group.NumBitmapBlocks(); block++) {
			if (cached.SetTo(group, block) < B_OK)
				RETURN_ERROR(B_ERROR);

			T(Block("alloc-in", group.Start() + block, cached.Block(),
				fVolume->BlockSize(), groupIndex, currentStart));

			// find a block large enough to hold the allocation
			for (uint32 bit = start % bitsPerFullBlock;
					bit < cached.NumBlockBits(); bit++) {
				if (!cached.IsUsed(bit)) {
					if (currentLength == 0) {
						// start new range
						currentStart = currentBit;
					}

					// have we found a range large enough to hold numBlocks?
					if (++currentLength >= maximum) {
						bestGroup = groupIndex;
						bestStart = currentStart;
						bestLength = currentLength;
						break;
					}
				} else {
					if (currentLength) {
						// end of a range
						if (currentLength > bestLength) {
							bestGroup = groupIndex;
							bestStart = currentStart;
							bestLength = currentLength;
						}
						if (currentLength > groupLargestLength) {
							groupLargestStart = currentStart;
							groupLargestLength = currentLength;
						}
						currentLength = 0;
					}
					if (((int32)group.NumBits() - currentBit)
							<= groupLargestLength) {
						// We can't find a bigger block in this group anymore,
						// let's skip the rest.
						block = group.NumBitmapBlocks();
						break;
					}
				}
				currentBit++;
			}

			T(Block("alloc-out", block, cached.Block(),
				fVolume->BlockSize(), groupIndex, currentStart));

			if (bestLength >= maximum) {
				canFindGroupLargest = false;
				break;
			}

			// start from the beginning of the next block
			start = 0;
		}

		if (currentBit == (int32)group.NumBits()) {
			if (currentLength > bestLength) {
				bestGroup = groupIndex;
				bestStart = currentStart;
				bestLength = currentLength;
			}
			if (canFindGroupLargest && currentLength > groupLargestLength) {
				groupLargestStart = currentStart;
				groupLargestLength = currentLength;
			}
		}

		if (canFindGroupLargest && !group.fLargestValid
			&& groupLargestLength >= 0) {
			group.fLargestStart = groupLargestStart;
			group.fLargestLength = groupLargestLength;
			group.fLargestValid = true;
		}

		if (bestLength >= maximum)
			break;
	}

	// If we found a suitable range, mark the blocks as in use, and
	// write the updated block bitmap back to disk
	if (bestLength < minimum)
		return B_DEVICE_FULL;

	if (bestLength > maximum)
		bestLength = maximum;
	else if (minimum > 1) {
		// make sure bestLength is a multiple of minimum
		bestLength = round_down(bestLength, minimum);
	}

	if (fGroups[bestGroup].Allocate(transaction, bestStart, bestLength) != B_OK)
		RETURN_ERROR(B_IO_ERROR);

	CHECK_ALLOCATION_GROUP(bestGroup);

	run.allocation_group = HOST_ENDIAN_TO_BFS_INT32(bestGroup);
	run.start = HOST_ENDIAN_TO_BFS_INT16(bestStart);
	run.length = HOST_ENDIAN_TO_BFS_INT16(bestLength);

	fVolume->SuperBlock().used_blocks
		= HOST_ENDIAN_TO_BFS_INT64(fVolume->UsedBlocks() + bestLength);
		// We are not writing back the disk's superblock - it's
		// either done by the journaling code, or when the disk
		// is unmounted.
		// If the value is not correct at mount time, it will be
		// fixed anyway.

	// We need to flush any remaining blocks in the new allocation to make sure
	// they won't interfere with the file cache.
	block_cache_discard(fVolume->BlockCache(), fVolume->ToBlock(run),
		run.Length());

	T(Allocate(run));
	return B_OK;
}


status_t
BlockAllocator::AllocateForInode(Transaction& transaction,
	const block_run* parent, mode_t type, block_run& run)
{
	// Apply some allocation policies here (AllocateBlocks() will break them
	// if necessary) - we will start with those described in Dominic Giampaolo's
	// "Practical File System Design", and see how good they work

	// Files are going in the same allocation group as its parent,
	// sub-directories will be inserted 8 allocation groups after
	// the one of the parent
	uint16 group = parent->AllocationGroup();
	if ((type & (S_DIRECTORY | S_INDEX_DIR | S_ATTR_DIR)) == S_DIRECTORY)
		group += 8;

	return AllocateBlocks(transaction, group, 0, 1, 1, run);
}


status_t
BlockAllocator::Allocate(Transaction& transaction, Inode* inode,
	off_t numBlocks, block_run& run, uint16 minimum)
{
	if (numBlocks <= 0)
		return B_ERROR;

	// one block_run can't hold more data than there is in one allocation group
	if (numBlocks > fGroups[0].NumBits())
		numBlocks = fGroups[0].NumBits();

	// since block_run.length is uint16, the largest number of blocks that
	// can be covered by a block_run is 65535
	// TODO: if we drop compatibility, couldn't we do this any better?
	// There are basically two possibilities:
	// a) since a length of zero doesn't have any sense, take that for 65536 -
	//    but that could cause many problems (bugs) in other areas
	// b) reduce the maximum amount of blocks per block_run, so that the
	//    remaining number of free blocks can be used in a useful manner
	//    (like 4 blocks) - but that would also reduce the maximum file size
	// c) have BlockRun::Length() return (length + 1).
	if (numBlocks > MAX_BLOCK_RUN_LENGTH)
		numBlocks = MAX_BLOCK_RUN_LENGTH;

	// Apply some allocation policies here (AllocateBlocks() will break them
	// if necessary)
	uint16 group = inode->BlockRun().AllocationGroup();
	uint16 start = 0;

	// Are there already allocated blocks? (then just try to allocate near the
	// last one)
	if (inode->Size() > 0) {
		const data_stream& data = inode->Node().data;
		// TODO: we currently don't care for when the data stream
		// is already grown into the indirect ranges
		if (data.max_double_indirect_range == 0
			&& data.max_indirect_range == 0) {
			// Since size > 0, there must be a valid block run in this stream
			int32 last = 0;
			for (; last < NUM_DIRECT_BLOCKS - 1; last++)
				if (data.direct[last + 1].IsZero())
					break;

			group = data.direct[last].AllocationGroup();
			start = data.direct[last].Start() + data.direct[last].Length();
		}
	} else if (inode->IsContainer() || inode->IsSymLink()) {
		// directory and symbolic link data will go in the same allocation
		// group as the inode is in but after the inode data
		start = inode->BlockRun().Start();
	} else {
		// file data will start in the next allocation group
		group = inode->BlockRun().AllocationGroup() + 1;
	}

	return AllocateBlocks(transaction, group, start, numBlocks, minimum, run);
}


status_t
BlockAllocator::Free(Transaction& transaction, block_run run)
{
	RecursiveLocker lock(fLock);

	int32 group = run.AllocationGroup();
	uint16 start = run.Start();
	uint16 length = run.Length();

	FUNCTION_START(("group = %" B_PRId32 ", start = %" B_PRIu16
		", length = %" B_PRIu16 "\n", group, start, length))
	T(Free(run));

	// doesn't use Volume::IsValidBlockRun() here because it can check better
	// against the group size (the last group may have a different length)
	if (group < 0 || group >= fNumGroups
		|| start > fGroups[group].NumBits()
		|| uint32(start + length) > fGroups[group].NumBits()
		|| length == 0) {
		FATAL(("tried to free an invalid block_run"
			" (%" B_PRId32 ", %" B_PRIu16 ", %" B_PRIu16")\n",
			group, start, length));
		DEBUGGER(("tried to free invalid block_run"));
		return B_BAD_VALUE;
	}
	// check if someone tries to free reserved areas at the beginning of the
	// drive
	if (group < fVolume->Log().AllocationGroup()
		|| (group == fVolume->Log().AllocationGroup()
			&& start < uint32(fVolume->Log().Start()) + fVolume->Log().Length())) {
		FATAL(("tried to free a reserved block_run"
			" (%" B_PRId32 ", %" B_PRIu16 ", %" B_PRIu16")\n",
			group, start, length));
		DEBUGGER(("tried to free reserved block"));
		return B_BAD_VALUE;
	}
#ifdef DEBUG
	if (CheckBlockRun(run) != B_OK)
		return B_BAD_DATA;
#endif

	CHECK_ALLOCATION_GROUP(group);

	if (fGroups[group].Free(transaction, start, length) != B_OK)
		RETURN_ERROR(B_IO_ERROR);

	CHECK_ALLOCATION_GROUP(group);

#ifdef DEBUG
	if (CheckBlockRun(run, NULL, false) != B_OK) {
		DEBUGGER(("CheckBlockRun() reports allocated blocks (which were just "
			"freed)\n"));
	}
#endif

	fVolume->SuperBlock().used_blocks =
		HOST_ENDIAN_TO_BFS_INT64(fVolume->UsedBlocks() - run.Length());
	return B_OK;
}


#ifdef DEBUG_FRAGMENTER
void
BlockAllocator::Fragment()
{
	AllocationBlock cached(fVolume);
	RecursiveLocker lock(fLock);

	// only leave 4 block holes
	static const uint32 kMask = 0x0f0f0f0f;
	uint32 valuesPerBlock = fVolume->BlockSize() / 4;

	for (int32 i = 0; i < fNumGroups; i++) {
		AllocationGroup& group = fGroups[i];

		for (uint32 block = 0; block < group.NumBlocks(); block++) {
			Transaction transaction(fVolume, 0);

			if (cached.SetToWritable(transaction, group, block) != B_OK)
				return;

			for (int32 index = 0; index < valuesPerBlock; index++) {
				cached.Block(index) |= HOST_ENDIAN_TO_BFS_INT32(kMask);
			}

			transaction.Done();
		}
	}
}
#endif	// DEBUG_FRAGMENTER


#ifdef DEBUG_ALLOCATION_GROUPS
void
BlockAllocator::_CheckGroup(int32 groupIndex) const
{
	AllocationBlock cached(fVolume);
	ASSERT_LOCKED_RECURSIVE(&fLock);

	AllocationGroup& group = fGroups[groupIndex];

	int32 currentStart = 0, currentLength = 0;
	int32 firstFree = -1;
	int32 largestStart = -1;
	int32 largestLength = 0;
	int32 currentBit = 0;

	for (uint32 block = 0; block < group.NumBlocks(); block++) {
		if (cached.SetTo(group, block) < B_OK) {
			panic("setting group block %d failed\n", (int)block);
			return;
		}

		for (uint32 bit = 0; bit < cached.NumBlockBits(); bit++) {
			if (!cached.IsUsed(bit)) {
				if (firstFree < 0) {
					firstFree = currentBit;
					if (!group.fLargestValid) {
						if (firstFree >= 0 && firstFree < group.fFirstFree) {
							// mostly harmless but noteworthy
							dprintf("group %d first free too late: should be "
								"%d, is %d\n", (int)groupIndex, (int)firstFree,
								(int)group.fFirstFree);
						}
						return;
					}
				}

				if (currentLength == 0) {
					// start new range
					currentStart = currentBit;
				}
				currentLength++;
			} else if (currentLength) {
				// end of a range
				if (currentLength > largestLength) {
					largestStart = currentStart;
					largestLength = currentLength;
				}
				currentLength = 0;
			}
			currentBit++;
		}
	}

	if (currentLength > largestLength) {
		largestStart = currentStart;
		largestLength = currentLength;
	}

	if (firstFree >= 0 && firstFree < group.fFirstFree) {
		// mostly harmless but noteworthy
		dprintf("group %d first free too late: should be %d, is %d\n",
			(int)groupIndex, (int)firstFree, (int)group.fFirstFree);
	}
	if (group.fLargestValid
		&& (largestStart != group.fLargestStart
			|| largestLength != group.fLargestLength)) {
		panic("bfs %p: group %d largest differs: %d.%d, checked %d.%d.\n",
			fVolume, (int)groupIndex, (int)group.fLargestStart,
			(int)group.fLargestLength, (int)largestStart, (int)largestLength);
	}
}
#endif	// DEBUG_ALLOCATION_GROUPS


status_t
BlockAllocator::Trim(uint64 offset, uint64 size, uint64& trimmedSize)
{
	// TODO: Remove this check when offset and size handling is implemented
	if (offset != 0
		|| fVolume->NumBlocks() < 0
		|| size < (uint64)fVolume->NumBlocks() * fVolume->BlockSize()) {
		INFORM(("BFS Trim: Ranges smaller than the file system size"
			" are not supported yet.\n"));
		return B_UNSUPPORTED;
	}

	const uint32 kTrimRanges = 128;
	fs_trim_data* trimData = (fs_trim_data*)malloc(sizeof(fs_trim_data)
		+ 2 * sizeof(uint64) * (kTrimRanges - 1));
	if (trimData == NULL)
		return B_NO_MEMORY;

	MemoryDeleter deleter(trimData);
	RecursiveLocker locker(fLock);

	// TODO: take given offset and size into account!
	int32 lastGroup = fNumGroups - 1;
	uint32 firstBlock = 0;
	uint32 firstBit = 0;
	uint64 currentBlock = 0;
	uint32 blockShift = fVolume->BlockShift();

	uint64 firstFree = 0;
	uint64 freeLength = 0;

	trimData->range_count = 0;
	trimmedSize = 0;

	AllocationBlock cached(fVolume);
	for (int32 groupIndex = 0; groupIndex <= lastGroup; groupIndex++) {
		AllocationGroup& group = fGroups[groupIndex];

		for (uint32 block = firstBlock; block < group.NumBitmapBlocks(); block++) {
			cached.SetTo(group, block);

			for (uint32 i = firstBit; i < cached.NumBlockBits(); i++) {
				if (cached.IsUsed(i)) {
					// Block is in use
					if (freeLength > 0) {
						// Overflow is unlikely to happen, but check it anyway
						if ((firstFree << blockShift) >> blockShift
								!= firstFree
							|| (freeLength << blockShift) >> blockShift
								!= freeLength) {
							FATAL(("BlockAllocator::Trim:"
								" Overflow detected!\n"));
							return B_ERROR;
						}
						status_t status = _TrimNext(*trimData, kTrimRanges,
							firstFree << blockShift, freeLength << blockShift,
							false, trimmedSize);
						if (status != B_OK)
							return status;

						freeLength = 0;
					}
				} else if (freeLength++ == 0) {
					// Block is free, start new free range
					firstFree = currentBlock;
				}

				currentBlock++;
			}
		}

		firstBlock = 0;
		firstBit = 0;
	}

	return _TrimNext(*trimData, kTrimRanges, firstFree << blockShift,
		freeLength << blockShift, true, trimmedSize);
}


//	#pragma mark -


/*!	Checks whether or not the specified block range is allocated or not,
	depending on the \a allocated argument.
*/
status_t
BlockAllocator::CheckBlocks(off_t start, off_t length, bool allocated,
	off_t* firstError)
{
	if (start < 0 || start + length > fVolume->NumBlocks())
		return B_BAD_VALUE;

	off_t block = start;

	int32 group = start >> fVolume->AllocationGroupShift();
	uint32 bitmapBlock = start / (fVolume->BlockSize() << 3);
	uint32 blockOffset = start % (fVolume->BlockSize() << 3);

	uint32 groupBlock = bitmapBlock % fBlocksPerGroup;

	AllocationBlock cached(fVolume);

	while (groupBlock < fGroups[group].NumBitmapBlocks() && length > 0) {
		if (cached.SetTo(fGroups[group], groupBlock) != B_OK)
			RETURN_ERROR(B_IO_ERROR);

		for (; blockOffset < cached.NumBlockBits() && length > 0;
				blockOffset++, length--, block++) {
			if (cached.IsUsed(blockOffset) != allocated) {
				PRINT(("CheckBlocks: Erroneous block (group = %" B_PRId32
					", groupBlock = %" B_PRIu32
					", blockOffset = %" B_PRIu32 ")!\n",
					group, groupBlock, blockOffset));

				if (firstError)
					*firstError = block;

				return B_BAD_DATA;
			}
		}

		blockOffset = 0;

		if (++groupBlock >= fGroups[group].NumBitmapBlocks()) {
			groupBlock = 0;
			group++;
		}
	}

	return B_OK;
}


bool
BlockAllocator::IsValidBlockRun(block_run run, const char* type)
{
	if (run.AllocationGroup() < 0 || run.AllocationGroup() >= fNumGroups
		|| run.Start() > fGroups[run.AllocationGroup()].fNumBits
		|| uint32(run.Start() + run.Length())
				> fGroups[run.AllocationGroup()].fNumBits
		|| run.length == 0) {
		PRINT(("%s: block_run(%" B_PRId32 ", %" B_PRIu16 ", %" B_PRIu16")"
			" is invalid!\n", type, run.AllocationGroup(), run.Start(),
			run.Length()));
		return false;
	}
	return true;
}


status_t
BlockAllocator::CheckBlockRun(block_run run, const char* type, bool allocated)
{
	if (!IsValidBlockRun(run, type))
		return B_BAD_DATA;

	status_t status = CheckBlocks(fVolume->ToBlock(run), run.Length(),
		allocated);
	if (status != B_OK) {
		PRINT(("%s: block_run(%" B_PRId32 ", %" B_PRIu16 ", %" B_PRIu16")"
			" is only partially allocated!\n", type, run.AllocationGroup(),
			run.Start(), run.Length()));
	}

	return status;
}


bool
BlockAllocator::_AddTrim(fs_trim_data& trimData, uint32 maxRanges,
	uint64 offset, uint64 size)
{
	ASSERT(trimData.range_count < maxRanges);
	if (size == 0)
		return false;

	trimData.ranges[trimData.range_count].offset = offset;
	trimData.ranges[trimData.range_count].size = size;
	trimData.range_count++;

	return (trimData.range_count == maxRanges);
}


status_t
BlockAllocator::_TrimNext(fs_trim_data& trimData, uint32 maxRanges,
	uint64 offset, uint64 size, bool force, uint64& trimmedSize)
{
	PRINT(("_TrimNext(index %" B_PRIu32 ", offset %" B_PRIu64 ", size %"
		B_PRIu64 ")\n", trimData.range_count, offset, size));

	const bool rangesFilled = _AddTrim(trimData, maxRanges, offset, size);

	if (rangesFilled || force) {
		// Trim now
		trimData.trimmed_size = 0;
#ifdef DEBUG_TRIM
		dprintf("TRIM: BFS: free ranges (bytes):\n");
		for (uint32 i = 0; i < trimData.range_count; i++) {
			dprintf("[%3" B_PRIu32 "] %" B_PRIu64 " : %" B_PRIu64 "\n", i,
				trimData.ranges[i].offset, trimData.ranges[i].size);
		}
#endif
		if (ioctl(fVolume->Device(), B_TRIM_DEVICE, &trimData,
				sizeof(fs_trim_data)
					+ 2 * sizeof(uint64) * (trimData.range_count - 1)) != 0) {
			return errno;
		}

		trimmedSize += trimData.trimmed_size;
		trimData.range_count = 0;
	}

	return B_OK;
}


//	#pragma mark - debugger commands


#ifdef BFS_DEBUGGER_COMMANDS

void
BlockAllocator::Dump(int32 index)
{
	kprintf("allocation groups: %" B_PRId32 " (base %p)\n", fNumGroups, fGroups);
	kprintf("blocks per group: %" B_PRId32 "\n", fBlocksPerGroup);

	for (int32 i = 0; i < fNumGroups; i++) {
		if (index != -1 && i != index)
			continue;

		AllocationGroup& group = fGroups[i];

		kprintf("[%3" B_PRId32 "] num bits:       %" B_PRIu32 "  (%p)\n", i,
			group.NumBits(), &group);
		kprintf("      num blocks:     %" B_PRIu32 "\n", group.NumBitmapBlocks());
		kprintf("      start:          %" B_PRId32 "\n", group.Start());
		kprintf("      first free:     %" B_PRId32 "\n", group.fFirstFree);
		kprintf("      largest start:  %" B_PRId32 "%s\n", group.fLargestStart,
			group.fLargestValid ? "" : "  (invalid)");
		kprintf("      largest length: %" B_PRId32 "\n", group.fLargestLength);
		kprintf("      free bits:      %" B_PRId32 "\n", group.fFreeBits);
	}
}


#if BFS_TRACING
static char kTraceBuffer[256];


int
dump_block_allocator_blocks(int argc, char** argv)
{
	if (argc != 3 || !strcmp(argv[1], "--help")) {
		kprintf("usage: %s <ptr-to-volume> <block>\n", argv[0]);
		return 0;
	}

	Volume* volume = (Volume*)parse_expression(argv[1]);
	off_t block = parse_expression(argv[2]);

	// iterate over all tracing entries to find overlapping actions

	using namespace BFSBlockTracing;

	LazyTraceOutput out(kTraceBuffer, sizeof(kTraceBuffer), 0);
	TraceEntryIterator iterator;
	while (TraceEntry* _entry = iterator.Next()) {
		if (Allocate* entry = dynamic_cast<Allocate*>(_entry)) {
			off_t first = volume->ToBlock(entry->Run());
			off_t last = first - 1 + entry->Run().Length();
			if (block >= first && block <= last) {
				out.Clear();
				const char* dump = out.DumpEntry(entry);
				kprintf("%5ld. %s\n", iterator.Index(), dump);
			}
		} else if (Free* entry = dynamic_cast<Free*>(_entry)) {
			off_t first = volume->ToBlock(entry->Run());
			off_t last = first - 1 + entry->Run().Length();
			if (block >= first && block <= last) {
				out.Clear();
				const char* dump = out.DumpEntry(entry);
				kprintf("%5ld. %s\n", iterator.Index(), dump);
			}
		}
	}

	return 0;
}
#endif


int
dump_block_allocator(int argc, char** argv)
{
	int32 group = -1;
	if (argc == 3) {
		group = parse_expression(argv[2]);
		argc--;
	}

	if (argc != 2 || !strcmp(argv[1], "--help")) {
		kprintf("usage: %s <ptr-to-volume> [group]\n", argv[0]);
		return 0;
	}

	Volume* volume = (Volume*)parse_expression(argv[1]);
	BlockAllocator& allocator = volume->Allocator();

	allocator.Dump(group);
	return 0;
}

#endif	// BFS_DEBUGGER_COMMANDS

