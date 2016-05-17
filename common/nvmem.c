/* Copyright 2016 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "common.h"
#include "console.h"
#include "flash.h"
#include "nvmem.h"
#include "shared_mem.h"
#include "timer.h"
#include "util.h"

#define CPRINTF(format, args...) cprintf(CC_COMMAND, format, ## args)

#define NVMEM_ACQUIRE_CACHE_SLEEP_MS 25
#define NVMEM_ACQUIRE_CACHE_MAX_ATTEMPTS (250 / NVMEM_ACQUIRE_CACHE_SLEEP_MS)
#define NVMEM_NOT_INITIALIZED (-1)

/* Structure MvMem Partition */
struct nvmem_partition {
	struct nvmem_tag tag;
	uint8_t buffer[NVMEM_PARTITION_SIZE -
		       sizeof(struct nvmem_tag)];
};

/* NvMem user buffer start offset table */
static uint32_t nvmem_user_start_offset[NVMEM_NUM_USERS];

/* A/B partion that is most up to date */
static int nvmem_act_partition;

/* NvMem Cache Memory pointer */
static uint8_t *cache_base_ptr;

static int nvmem_verify_partition_sha(int index)
{
	uint8_t sha_comp[NVMEM_SHA_SIZE];
	struct nvmem_partition *p_part;
	uint8_t *p_data;

	p_part = (struct nvmem_partition *)CONFIG_FLASH_NVMEM_BASE;
	p_part += index;
	p_data = (uint8_t *)p_part;
	p_data += sizeof(sha_comp);

	/* Number of bytes to compute sha over */
	nvmem_compute_sha(p_data,
			  (NVMEM_PARTITION_SIZE - NVMEM_SHA_SIZE),
			  sha_comp,
			  NVMEM_SHA_SIZE);
	/* Check if computed value matches stored value. */
	return memcmp(p_part->tag.sha, sha_comp, NVMEM_SHA_SIZE);
}

static int nvmem_acquire_cache(void)
{
	int attempts = 0;
	int ret;

	/* TODO Need to add mutex lock/unlock crosbug.com/p/52520 */

	if (shared_mem_size() < NVMEM_PARTITION_SIZE) {
		CPRINTF("Not enough shared mem! avail = 0x%x < reqd = 0x%x\n",
			shared_mem_size(), NVMEM_PARTITION_SIZE);
		return EC_ERROR_OVERFLOW;
	}

	while (attempts < NVMEM_ACQUIRE_CACHE_MAX_ATTEMPTS) {
		ret = shared_mem_acquire(NVMEM_PARTITION_SIZE,
					 (char **)&cache_base_ptr);
		if (ret == EC_SUCCESS)
			return EC_SUCCESS;
		else if (ret == EC_ERROR_BUSY) {
			CPRINTF("Shared Mem not avail! Attempt %d\n", attempts);
			/* wait NVMEM_ACQUIRE_CACHE_SLEEP_MS  msec */
			/* TODO: what time really makes sense? */
			msleep(NVMEM_ACQUIRE_CACHE_SLEEP_MS);
		}
		attempts++;
	}
	/* Timeout Error condition */
	CPRINTF("%s:%d\n", __func__, __LINE__);
	return EC_ERROR_TIMEOUT;
}

static int nvmem_update_cache_ptr(void)
{
	uint8_t *p_src;

	/*
	 * If cache_base_ptr is not NULL, then nothing to do. However, if NULL,
	 * then need to first acquire the shared memory buffer and the full
	 * partition needs to be copied from flash into the cache buffer.
	 */
	if (cache_base_ptr == NULL) {
		if (nvmem_acquire_cache() != EC_SUCCESS)
			return EC_ERROR_TIMEOUT;
		/* Copy partiion contents from flash into cache buffer */
		p_src = (uint8_t *)(CONFIG_FLASH_NVMEM_BASE +
				    nvmem_act_partition *
				    NVMEM_PARTITION_SIZE);
		memcpy(cache_base_ptr, p_src,
		       NVMEM_PARTITION_SIZE);
	}

	return EC_SUCCESS;
}

static void nvmem_release_cache(void)
{
	/* Done with shared memory buffer, release it. */
	shared_mem_release(cache_base_ptr);
	/* Inidicate cache is not available */
	cache_base_ptr = NULL;
	/* TODO Release mutex lock here crosbug.com/p/52520 */
}

static int nvmem_is_unitialized(void)
{
	int n;
	int ret;
	uint32_t *p_nvmem;
	struct nvmem_partition *p_part;

	/* Point to start of Nv Memory */
	p_nvmem = (uint32_t *)CONFIG_FLASH_NVMEM_BASE;
	/* Verify that each byte is 0xff (4 bytes at a time) */
	for (n = 0; n < (CONFIG_FLASH_NVMEM_SIZE >> 2); n++)
		if (p_nvmem[n] != 0xffffffff)
			return EC_ERROR_CRC;

	/*
	 * NvMem is fully unitialized. Need to initialize tag and write tag to
	 * flash so at least 1 partition is ready to be used.
	 */
	nvmem_act_partition = 0;
	/* Need to acquire the shared memory buffer */
	ret = nvmem_update_cache_ptr();
	if (ret != EC_SUCCESS)
		return ret;
	p_part = (struct nvmem_partition *)cache_base_ptr;
	/* Start with version 0 */
	p_part->tag.version = 0;
	/* Compute sha with updated tag */
	nvmem_compute_sha(&cache_base_ptr[NVMEM_SHA_SIZE],
			  NVMEM_PARTITION_SIZE - NVMEM_SHA_SIZE,
			  p_part->tag.sha,
			  NVMEM_SHA_SIZE);
	/*
	 * Partition 0 is initialized, write tag only to flash. Since the
	 * partition was just verified to be fully erased, can just do write
	 * operation.
	 */
	if (flash_physical_write(CONFIG_FLASH_NVMEM_OFFSET,
				 sizeof(struct nvmem_tag),
				 cache_base_ptr)) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		nvmem_release_cache();
		return EC_ERROR_UNKNOWN;
	}
	/* Can release the cache buffer now */
	nvmem_release_cache();

	return EC_SUCCESS;
}

static int nvmem_compare_version(void)
{
	struct nvmem_partition *p_part;
	uint16_t ver0, ver1;
	uint32_t delta;

	p_part = (struct nvmem_partition *)CONFIG_FLASH_NVMEM_BASE;
	ver0 = p_part->tag.version;
	p_part++;
	ver1 = p_part->tag.version;

	/* Compute version difference accounting for wrap condition */
	delta = (ver0 - ver1 + (1<<NVMEM_VERSION_BITS)) & NVMEM_VERSION_MASK;
	/*
	 * If version number delta is positive in a circular sense then
	 * partition 0 has the newest version number. Otherwise, it's
	 * partition 1.
	 */
	return delta < (1<<(NVMEM_VERSION_BITS-1)) ? 0 : 1;
}

static int nvmem_find_partition(void)
{
	int n;

	/* Don't know which partition to use yet */
	nvmem_act_partition = NVMEM_NOT_INITIALIZED;
	/*
	 * Check each partition to determine if the sha is good. If both
	 * partitions have valid sha(s), then compare version numbers to select
	 * the most recent one.
	 */
	for (n = 0; n < NVMEM_NUM_PARTITIONS; n++)
		if (nvmem_verify_partition_sha(n) == EC_SUCCESS) {
			if (nvmem_act_partition == NVMEM_NOT_INITIALIZED)
				nvmem_act_partition = n;
			else
				nvmem_act_partition = nvmem_compare_version();
		}
	/*
	 * If active_partition is still not selected, then neither partition is
	 * valid. In this case need to determine if they are simply erased or
	 * both are corrupt. If erased, then can initialze the tag for the first
	 * one. If not fully erased, then this is an error condition.
	 */
	if (nvmem_act_partition != NVMEM_NOT_INITIALIZED)
		return EC_SUCCESS;

	if (nvmem_is_unitialized()) {
		CPRINTF("NvMem: No Valid Paritions and not fully erased!!\n");
		return EC_ERROR_UNKNOWN;
	}

	return EC_SUCCESS;
}

static int nvmem_generate_offset_table(void)
{
	int n;
	uint32_t start_offset;

	/*
	 * Create table of starting offsets within partition for each user
	 * buffer that's been defined.
	 */
	start_offset = sizeof(struct nvmem_tag);
	for (n = 0; n < NVMEM_NUM_USERS; n++) {
		nvmem_user_start_offset[n] = start_offset;
		start_offset += nvmem_user_sizes[n];
	}
	/* Verify that all defined user buffers fit within the partition */
	if (start_offset > NVMEM_PARTITION_SIZE)
		return EC_ERROR_OVERFLOW;

	return EC_SUCCESS;
}
static int nvmem_get_partition_off(int user, uint32_t offset,
				   uint32_t len, uint32_t *p_buf_offset)
{
	uint32_t start_offset;

	/* Sanity check for user */
	if (user >= NVMEM_NUM_USERS)
		return EC_ERROR_OVERFLOW;

	/* Get offset within the partition for the start of user buffer */
	start_offset = nvmem_user_start_offset[user];
	/*
	 * Ensure that read/write operation that is calling this function
	 * doesn't exceed the end of its buffer.
	 */
	if (offset + len > nvmem_user_sizes[user])
		return EC_ERROR_OVERFLOW;
	/* Compute offset within the partition for the rd/wr operation */
	*p_buf_offset = start_offset + offset;

	return EC_SUCCESS;
}

int nvmem_setup(uint8_t starting_version)
{
	struct nvmem_partition *p_part;
	int part;
	int ret;

	CPRINTF("Configuring NVMEM FLash Partition\n");
	/*
	 * Initialize NVmem partition. This function will only be called
	 * if during nvmem_init() fails which implies that NvMem is not fully
	 * erased and neither partion tag contains a valid sha meaning they are
	 * both corrupted
	 */
	for (part = 0; part < NVMEM_NUM_PARTITIONS; part++) {
		/* Set active partition variable */
		nvmem_act_partition = part;
		/* Get the cache buffer */
		if (nvmem_update_cache_ptr() != EC_SUCCESS) {
			CPRINTF("NvMem: Cache ram not available!\n");
			return EC_ERROR_TIMEOUT;
		}

		/* Fill in tag info */
		p_part = (struct nvmem_partition *)cache_base_ptr;
		/* Commit function will increment version number */
		p_part->tag.version = starting_version + part - 1;
		nvmem_compute_sha(&cache_base_ptr[NVMEM_SHA_SIZE],
				  NVMEM_PARTITION_SIZE -
				  NVMEM_SHA_SIZE,
				  p_part->tag.sha,
				  NVMEM_SHA_SIZE);
		/*
		 * TODO: Should erase parition area prior to this function being
		 * called, or could write all user buffer data to 0xff here
		 * before the commit() call.
		 */
		/* Partition is now ready, write it to flash. */
		ret = nvmem_commit();
		if (ret != EC_SUCCESS)
			return ret;
	}

	return EC_SUCCESS;
}

int nvmem_init(void)
{
	int ret;

	/* Generate start offsets within partiion for user buffers */
	ret = nvmem_generate_offset_table();
	if (ret) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		return ret;
	}
	/* Default state for cache_base_ptr */
	cache_base_ptr = NULL;
	ret = nvmem_find_partition();
	if (ret != EC_SUCCESS) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		return ret;
	}

	return EC_SUCCESS;
}

int nvmem_read(unsigned int offset, unsigned int size,
		    void *data, enum nvmem_users user)
{
	int ret;
	uint8_t *p_src;
	uintptr_t src_addr;
	uint32_t src_offset;

	/* Point to either NvMem flash or ram if that's active */
	if (cache_base_ptr == NULL)
		src_addr = CONFIG_FLASH_NVMEM_BASE + nvmem_act_partition *
			NVMEM_PARTITION_SIZE;

	else
		src_addr = (uintptr_t)cache_base_ptr;
	/* Get partition offset for this read operation */
	ret = nvmem_get_partition_off(user, offset, size, &src_offset);
	if (ret != EC_SUCCESS)
		return ret;
	/* Advance to the correct byte within the data buffer */
	src_addr += src_offset;
	p_src = (uint8_t *)src_addr;

	/* Copy from src into the caller's destination buffer */
	memcpy(data, p_src, size);

	return EC_SUCCESS;
}

int nvmem_write(unsigned int offset, unsigned int size,
		 void *data, enum nvmem_users user)
{
	int ret;
	uint8_t *p_dest;
	uintptr_t dest_addr;
	uint32_t dest_offset;

	/* Make sure that the cache buffer is active */
	ret = nvmem_update_cache_ptr();
	if (ret)
		/* TODO: What to do when can't access cache buffer? */
		return ret;

	/* Compute partition offset for this write operation */
	ret = nvmem_get_partition_off(user, offset, size, &dest_offset);
	if (ret != EC_SUCCESS)
		return ret;

	/* Advance to correct offset within data buffer */
	dest_addr = (uintptr_t)cache_base_ptr;
	dest_addr += dest_offset;
	p_dest = (uint8_t *)dest_addr;
	/* Copy data from caller into destination buffer */
	memcpy(p_dest, data, size);

	return EC_SUCCESS;
}

int nvmem_commit(void)
{
	int nvmem_offset;
	int new_active_partition;
	uint16_t version;
	struct nvmem_partition *p_part;

	/*
	 * All scratch buffer blocks must be written to physical flash
	 * memory. In addition, the scratch block buffer index table
	 * entries must be reset along with the index itself.
	 */

	/* Update version number */
	if (cache_base_ptr == NULL) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		return EC_ERROR_UNKNOWN;
	}
	p_part = (struct nvmem_partition *)cache_base_ptr;
	version = p_part->tag.version + 1;
	/* Check for restricted version number */
	if (version == NVMEM_VERSION_MASK)
		version = 0;
	p_part->tag.version = version;
	/* Update the sha */
	nvmem_compute_sha(&cache_base_ptr[NVMEM_SHA_SIZE],
			  NVMEM_PARTITION_SIZE - NVMEM_SHA_SIZE,
			  p_part->tag.sha,
			  NVMEM_SHA_SIZE);

	/* Toggle parition being used (always write to current spare) */
	new_active_partition = nvmem_act_partition ^ 1;
	/* Point to first block within active partition */
	nvmem_offset = CONFIG_FLASH_NVMEM_OFFSET + new_active_partition *
			NVMEM_PARTITION_SIZE;
	/* Write partition to NvMem */

	/* Erase partition */
	if (flash_physical_erase(nvmem_offset,
				 NVMEM_PARTITION_SIZE)) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		/* Free up scratch buffers */
		nvmem_release_cache();
		return EC_ERROR_UNKNOWN;
	}
	/* Write partition */
	if (flash_physical_write(nvmem_offset,
				 NVMEM_PARTITION_SIZE,
				 cache_base_ptr)) {
		CPRINTF("%s:%d\n", __func__, __LINE__);
		/* Free up scratch buffers */
		nvmem_release_cache();
		return EC_ERROR_UNKNOWN;
	}

	/* Free up scratch buffers */
	nvmem_release_cache();
	/* Update newest partition index */
	nvmem_act_partition = new_active_partition;
	return EC_SUCCESS;
}
