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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <helper/log.h>
#include <helper/list.h>
#include <target/target.h>
#include <target/rtt.h>

#include "rtt.h"

static struct {
	struct rtt_source source;
	/** Control block. */
	struct rtt_control ctrl;
	struct target *target;
	/** Start address to search for the control block. */
	target_addr_t addr;
	/** Size of the control block search area. */
	size_t size;
	/** Control block identifier. */
	char id[RTT_CB_MAX_ID_LENGTH];
	/** Whether RTT is configured. */
	bool configured;
	/** Whether RTT is started. */
	bool started;
	/** Whether configuration changed. */
	bool changed;
	/** Whether the control block was found. */
	bool found_cb;

	struct rtt_sink_list **sink_list;
	size_t sink_list_length;

	unsigned int polling_interval;
} rtt;

int rtt_init(void)
{
	rtt.sink_list_length = 1;
	rtt.sink_list = calloc(rtt.sink_list_length,
		sizeof(struct rtt_sink_list *));

	if (!rtt.sink_list)
		return ERROR_FAIL;

	rtt.sink_list[0] = NULL;
	rtt.started = false;

	rtt.polling_interval = 100;

	return ERROR_OK;
}

int rtt_exit(void)
{
	free(rtt.sink_list);

	return ERROR_OK;
}

static int read_channel_callback(void *user_data)
{
	int ret;

	ret = rtt.source.read(rtt.target, &rtt.ctrl, rtt.sink_list,
		rtt.sink_list_length, NULL);

	if (ret != ERROR_OK) {
		target_unregister_timer_callback(&read_channel_callback, NULL);
		rtt.source.stop(rtt.target, NULL);
		return ret;
	}

	return ERROR_OK;
}

int rtt_setup(target_addr_t address, size_t size, const char *id)
{
	size_t id_length = strlen(id);

	if (!id_length || id_length >= RTT_CB_MAX_ID_LENGTH) {
		LOG_ERROR("rtt: Invalid control block ID");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	rtt.addr = address;
	rtt.size = size;
	strncpy(rtt.id, id, id_length + 1);
	rtt.changed = true;
	rtt.configured = true;

	return ERROR_OK;
}

int rtt_register_source(const struct rtt_source source,
		struct target *target)
{
	if (!source.find_cb || !source.read_cb || !source.read_channel_info)
		return ERROR_FAIL;

	if (!source.start || !source.stop)
		return ERROR_FAIL;

	if (!source.read || !source.write)
		return ERROR_FAIL;

	rtt.source = source;
	rtt.target = target;

	return ERROR_OK;
}

int rtt_start(void)
{
	int ret;
	target_addr_t addr = rtt.addr;

	if (rtt.started)
		return ERROR_OK;

	if (!rtt.found_cb || rtt.changed) {
		rtt.source.find_cb(rtt.target, &addr, rtt.size, rtt.id,
			&rtt.found_cb, NULL);

		rtt.changed = false;

		if (rtt.found_cb) {
			LOG_INFO("rtt: Control block found at 0x%" TARGET_PRIxADDR,
				addr);
			rtt.ctrl.address = addr;
		} else {
			LOG_INFO("rtt: No control block found");
			return ERROR_OK;
		}
	}

	ret = rtt.source.read_cb(rtt.target, rtt.ctrl.address, &rtt.ctrl, NULL);

	if (ret != ERROR_OK)
		return ret;

	ret = rtt.source.start(rtt.target, &rtt.ctrl, NULL);

	if (ret != ERROR_OK)
		return ret;

	target_register_timer_callback(&read_channel_callback,
		rtt.polling_interval, 1, NULL);
	rtt.started = true;

	return ERROR_OK;
}

int rtt_stop(void)
{
	int ret;

	if (!rtt.configured) {
		LOG_ERROR("rtt: Not configured");
		return ERROR_FAIL;
	}

	target_unregister_timer_callback(&read_channel_callback, NULL);
	rtt.started = false;

	ret = rtt.source.stop(rtt.target, NULL);

	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int adjust_sink_list(size_t length)
{
	struct rtt_sink_list **tmp;

	if (length <= rtt.sink_list_length)
		return ERROR_OK;

	tmp = realloc(rtt.sink_list, sizeof(struct rtt_sink_list *) * length);

	if (!tmp)
		return ERROR_FAIL;

	for (size_t i = rtt.sink_list_length; i < length; i++)
		tmp[i] = NULL;

	rtt.sink_list = tmp;
	rtt.sink_list_length = length;

	return ERROR_OK;
}

int rtt_register_sink(unsigned int channel_index, rtt_sink_read read,
		void *user_data)
{
	struct rtt_sink_list *tmp;

	if (channel_index >= rtt.sink_list_length) {
		if (adjust_sink_list(channel_index + 1) != ERROR_OK)
			return ERROR_FAIL;
	}

	LOG_DEBUG("rtt: Registering sink for channel %u", channel_index);

	tmp = malloc(sizeof(struct rtt_sink_list));

	if (!tmp)
		return ERROR_FAIL;

	tmp->read = read;
	tmp->user_data = user_data;
	tmp->next = rtt.sink_list[channel_index];

	rtt.sink_list[channel_index] = tmp;

	return ERROR_OK;
}

int rtt_unregister_sink(unsigned int channel_index, rtt_sink_read read,
		void *user_data)
{
	struct rtt_sink_list *prev_sink;

	LOG_DEBUG("rtt: Unregistering sink for channel %u", channel_index);

	if (channel_index >= rtt.sink_list_length)
		return ERROR_FAIL;

	prev_sink = rtt.sink_list[channel_index];

	for (struct rtt_sink_list *sink = rtt.sink_list[channel_index]; sink;
			prev_sink = sink, sink = sink->next) {
		if (sink->read == read && sink->user_data == user_data) {

			if (sink == rtt.sink_list[channel_index])
				rtt.sink_list[channel_index] = sink->next;
			else
				prev_sink->next = sink->next;

			free(sink);

			return ERROR_OK;
		}
	}

	return ERROR_OK;
}

int rtt_get_polling_interval(unsigned int *interval)
{
	if (!interval)
		return ERROR_FAIL;

	*interval = rtt.polling_interval;

	return ERROR_OK;
}

int rtt_set_polling_interval(unsigned int interval)
{
	if (!interval)
		return ERROR_FAIL;

	if (rtt.polling_interval != interval) {
		target_unregister_timer_callback(&read_channel_callback, NULL);
		target_register_timer_callback(&read_channel_callback, interval, 1,
			NULL);
	}

	rtt.polling_interval = interval;

	return ERROR_OK;
}

int rtt_write_channel(unsigned int channel_index, const uint8_t *buffer,
		size_t *length)
{
	if (channel_index >= rtt.ctrl.num_up_channels) {
		LOG_WARNING("rtt: Down-channel %u is not available", channel_index);
		return ERROR_OK;
	}

	return rtt.source.write(rtt.target, &rtt.ctrl, channel_index, buffer,
		length, NULL);
}

bool rtt_started(void)
{
	return rtt.started;
}

bool rtt_configured(void)
{
	return rtt.configured;
}

bool rtt_found_cb(void)
{
	return rtt.found_cb;
}

const struct rtt_control *rtt_get_control(void)
{
	return &rtt.ctrl;
}

int rtt_read_channel_info(unsigned int channel_index,
	enum rtt_channel_type type, struct rtt_channel_info *info)
{
	return rtt.source.read_channel_info(rtt.target, &rtt.ctrl,
		channel_index, type, info, NULL);
}
