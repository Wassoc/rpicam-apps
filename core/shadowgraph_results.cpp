/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * shadowgraph_results.cpp - timestamp queue of shadowgraph sidecars.
 */

#include <map>
#include <mutex>
#include <utility>

#include "core/logging.hpp"
#include "core/shadowgraph_results.hpp"

namespace
{

constexpr size_t kMaxPending = 32;

std::mutex &queueMutex()
{
	static std::mutex mutex;
	return mutex;
}

std::map<int64_t, ShadowgraphSidecar> &queue()
{
	static std::map<int64_t, ShadowgraphSidecar> pending;
	return pending;
}

} // namespace

void ShadowgraphResults::Publish(int64_t timestamp_us, ShadowgraphSidecar sidecar)
{
	std::lock_guard<std::mutex> lock(queueMutex());
	auto &pending = queue();
	if (pending.size() >= kMaxPending)
	{
		pending.erase(pending.begin());
		LOG(1, "shadowgraph: dropped an unpublished result");
	}
	pending[timestamp_us] = std::move(sidecar);
}

bool ShadowgraphResults::Take(int64_t timestamp_us, ShadowgraphSidecar &out)
{
	std::lock_guard<std::mutex> lock(queueMutex());
	auto &pending = queue();
	auto it = pending.find(timestamp_us);
	if (it == pending.end())
		return false;
	out = std::move(it->second);
	pending.erase(it);
	return true;
}

void ShadowgraphResults::Discard(int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(queueMutex());
	queue().erase(timestamp_us);
}

size_t ShadowgraphResults::Pending()
{
	std::lock_guard<std::mutex> lock(queueMutex());
	return queue().size();
}
