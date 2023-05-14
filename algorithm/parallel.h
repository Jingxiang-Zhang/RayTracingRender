#pragma once
#ifndef _PARALLEL_H_
#define _PARALLEL_H_
#include <thread>
#include <vector>
#include <functional>

void parallelRun(const std::function<void(long long unsigned int)>& func,
	const long long unsigned int size, const int nThreads =
	std::thread::hardware_concurrency() - 1);

#endif // !_PARALLEL_H_
