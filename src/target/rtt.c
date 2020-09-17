/*
 * Copyright (C) 2016-2020 by Marc Schink <dev@zapb.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdint.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <helper/command.h>
#include <rtt/rtt.h>

#include "target.h"

static int read_rtt_channel(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel_index,
		enum rtt_channel_type type, struct rtt_channel *channel)
{
	int ret;
	uint8_t buf[RTT_CHANNEL_SIZE];
	target_addr_t address;

	address = ctrl->address + RTT_CB_SIZE + (channel_index * RTT_CHANNEL_SIZE);

	if (type == RTT_CHANNEL_TYPE_DOWN)
		address += ctrl->num_up_channels * RTT_CHANNEL_SIZE;

	ret = target_read_buffer(target, address, RTT_CHANNEL_SIZE, buf);

	if (ret != ERROR_OK)
		return ret;

	channel->address = address;
	channel->name_addr = buf_get_u32(buf + 0, 0, 32);
	channel->buffer_addr = buf_get_u32(buf + 4, 0, 32);
	channel->size = buf_get_u32(buf + 8, 0, 32);
	channel->write_pos = buf_get_u32(buf + 12, 0, 32);
	channel->read_pos = buf_get_u32(buf + 16, 0, 32);
	channel->flags = buf_get_u32(buf + 20, 0, 32);

	return ERROR_OK;
}

int target_rtt_start(struct target *target, const struct rtt_control *ctrl,
		void *user_data)
{
	return ERROR_OK;
}

int target_rtt_stop(struct target *target, void *user_data)
{
	return ERROR_OK;
}

static int read_channel_name(struct target *target, target_addr_t address,
		char *name, size_t length)
{
	size_t offset;

	offset = 0;

	while (offset < length) {
		int ret;
		size_t read_length;

		read_length = MIN(32, length - offset);
		ret = target_read_buffer(target, address + offset, read_length,
			(uint8_t *)name + offset);

		if (ret != ERROR_OK)
			return ret;

		if (memchr(name + offset, '\0', read_length))
			return ERROR_OK;

		offset += read_length;
	}

	name[length - 1] = '\0';

	return ERROR_OK;
}

static int write_to_channel(struct target *target,
		const struct rtt_channel *channel, const uint8_t *buffer,
		size_t *length)
{
	int ret;
	uint32_t len;

	if (!*length)
		return ERROR_OK;

	if (channel->write_pos == channel->read_pos) {
		uint32_t first_length;

		len = MIN(*length, channel->size - 1);
		first_length = MIN(len, channel->size - channel->write_pos);

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, first_length,
			buffer);

		if (ret != ERROR_OK)
			return ret;

		ret = target_write_buffer(target, channel->buffer_addr,
			len - first_length, buffer + first_length);

		if (ret != ERROR_OK)
			return ret;
	} else if (channel->write_pos < channel->read_pos) {
		len = MIN(*length, channel->read_pos - channel->write_pos - 1);

		if (!len) {
			*length = 0;
			return ERROR_OK;
		}

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, len, buffer);

		if (ret != ERROR_OK)
			return ret;
	} else {
		uint32_t first_length;

		len = MIN(*length,
			channel->size - channel->write_pos + channel->read_pos - 1);

		if (!len) {
			*length = 0;
			return ERROR_OK;
		}

		first_length = MIN(len, channel->size - channel->write_pos);

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, first_length,
			buffer);

		if (ret != ERROR_OK)
			return ret;

		buffer = buffer + first_length;

		ret = target_write_buffer(target, channel->buffer_addr,
			len - first_length, buffer);

		if (ret != ERROR_OK)
			return ret;
	}

	ret = target_write_u32(target, channel->address + 12,
		(channel->write_pos + len) % channel->size);

	if (ret != ERROR_OK)
		return ret;

	*length = len;

	return ERROR_OK;
}

static bool channel_is_active(const struct rtt_channel *channel)
{
	if (!channel)
		return false;

	if (!channel->size)
		return false;

	return true;
}

int target_rtt_write_callback(struct target *target, struct rtt_control *ctrl,
		unsigned int channel_index, const uint8_t *buffer, size_t *length,
		void *user_data)
{
	int ret;
	struct rtt_channel channel;

	ret = read_rtt_channel(target, ctrl, channel_index,
		RTT_CHANNEL_TYPE_DOWN, &channel);

	if (ret != ERROR_OK) {
		LOG_ERROR("rtt: Failed to read down-channel %u description",
			channel_index);
		return ret;
	}

	if (!channel_is_active(&channel)) {
		LOG_WARNING("rtt: Down-channel %u is not active", channel_index);
		return ERROR_OK;
	}

	if (channel.size < RTT_CHANNEL_BUFFER_MIN_SIZE) {
		LOG_WARNING("rtt: Down-channel %u is not large enough",
			channel_index);
		return ERROR_OK;
	}

	ret = write_to_channel(target, &channel, buffer, length);

	if (ret != ERROR_OK)
		return ret;

	LOG_DEBUG("rtt: Wrote %zu bytes into down-channel %u", *length,
		channel_index);

	return ERROR_OK;
}

int target_rtt_read_control_block(struct target *target,
		target_addr_t address, struct rtt_control *ctrl, void *user_data)
{
	int ret;
	uint8_t buf[RTT_CB_SIZE];

	ret = target_read_buffer(target, address, RTT_CB_SIZE, buf);

	if (ret != ERROR_OK)
		return ret;

	memcpy(ctrl->id, buf, RTT_CB_MAX_ID_LENGTH);
	ctrl->id[RTT_CB_MAX_ID_LENGTH - 1] = '\0';
	ctrl->num_up_channels = buf_get_u32(buf + RTT_CB_MAX_ID_LENGTH + 0,
		0, 32);
	ctrl->num_down_channels = buf_get_u32(buf + RTT_CB_MAX_ID_LENGTH + 4,
		0, 32);

	return ERROR_OK;
}

int target_rtt_find_control_block(struct target *target,
		target_addr_t *address, size_t size, const char *id, bool *found,
		void *user_data)
{
	uint8_t buf[1024];

	*found = false;

	size_t j = 0;
	size_t cb_offset = 0;
	const size_t id_length = strlen(id);

	LOG_INFO("rtt: Searching for control block '%s'", id);

	for (target_addr_t addr = 0; addr < size; addr = addr + sizeof(buf)) {
		int ret;

		const size_t buf_size = MIN(sizeof(buf), size - addr);
		ret = target_read_buffer(target, *address + addr, buf_size, buf);

		if (ret != ERROR_OK)
			return ret;

		size_t start = 0;
		size_t i = 0;

		while (i < buf_size) {
			if (buf[i] != id[j]) {
				start++;
				cb_offset++;
				i = start;
				j = 0;

				continue;
			}

			i++;
			j++;

			if (j == id_length) {
				*address = *address + cb_offset;
				*found = true;
				return ERROR_OK;
			}
		}
	}

	return ERROR_OK;
}

int target_rtt_read_channel_info(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel_index,
		enum rtt_channel_type type, struct rtt_channel_info *info,
		void *user_data)
{
	int ret;
	struct rtt_channel channel;

	ret = read_rtt_channel(target, ctrl, channel_index, type, &channel);

	if (ret != ERROR_OK) {
		LOG_ERROR("rtt: Failed to read channel %u description",
			channel_index);
		return ret;
	}

	ret = read_channel_name(target, channel.name_addr, info->name,
		info->name_length);

	if (ret != ERROR_OK)
		return ret;

	info->size = channel.size;
	info->flags = channel.flags;

	return ERROR_OK;
}

static int read_from_channel(struct target *target,
		const struct rtt_channel *channel, uint8_t *buffer,
		size_t *length)
{
	int ret;
	uint32_t len;

	if (!*length)
		return ERROR_OK;

	if (channel->read_pos == channel->write_pos) {
		len = 0;
	} else if (channel->read_pos < channel->write_pos) {
		len = MIN(*length, channel->write_pos - channel->read_pos);

		ret = target_read_buffer(target,
			channel->buffer_addr + channel->read_pos, len, buffer);

		if (ret != ERROR_OK)
			return ret;
	} else {
		uint32_t first_length;

		len = MIN(*length,
			channel->size - channel->read_pos + channel->write_pos);
		first_length = MIN(len, channel->size - channel->read_pos);

		ret = target_read_buffer(target,
			channel->buffer_addr + channel->read_pos, first_length, buffer);

		if (ret != ERROR_OK)
			return ret;

		ret = target_read_buffer(target, channel->buffer_addr,
			len - first_length, buffer + first_length);

		if (ret != ERROR_OK)
			return ret;
	}

	if (len > 0) {
		ret = target_write_u32(target, channel->address + 16,
			(channel->read_pos + len) % channel->size);

		if (ret != ERROR_OK)
			return ret;
	}

	*length = len;

	return ERROR_OK;
}

int target_rtt_read_callback(struct target *target,
		const struct rtt_control *ctrl, struct rtt_sink_list **sinks,
		size_t num_channels, void *user_data)
{
	num_channels = MIN(num_channels, ctrl->num_up_channels);

	for (size_t i = 0; i < num_channels; i++) {
		int ret;
		struct rtt_channel channel;
		uint8_t buffer[1024];
		size_t length;

		if (!sinks[i])
			continue;

		ret = read_rtt_channel(target, ctrl, i, RTT_CHANNEL_TYPE_UP,
			&channel);

		if (ret != ERROR_OK) {
			LOG_ERROR("rtt: Failed to read up-channel %zu description", i);
			return ret;
		}

		if (!channel_is_active(&channel)) {
			LOG_WARNING("rtt: Up-channel %zu is not active", i);
			continue;
		}

		if (channel.size < RTT_CHANNEL_BUFFER_MIN_SIZE) {
			LOG_WARNING("rtt: Up-channel %zu is not large enough", i);
			continue;
		}

		length = sizeof(buffer);
		ret = read_from_channel(target, &channel, buffer, &length);

		if (ret != ERROR_OK) {
			LOG_ERROR("rtt: Failed to read from up-channel %zu", i);
			return ret;
		}

		for (struct rtt_sink_list *sink = sinks[i]; sink; sink = sink->next)
			sink->read(i, buffer, length, sink->user_data);
	}

	return ERROR_OK;
}
