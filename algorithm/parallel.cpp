#include "parallel.h"

void parallelRun(const std::function<void(long long unsigned int)>& func,
	const long long unsigned int size, const int nThreads) {
	std::vector<std::thread> threads_list;
	for (int thread = 0; thread < nThreads; thread++) {
		std::function<void(int)> warper = [&](int start) {
			for (long long unsigned int i = start; i < size; i += nThreads)
				func(i);
		};
		threads_list.push_back(std::thread(warper, thread));
	}
	for (int thread = 0; thread < nThreads; thread++) {
		threads_list[thread].join();
	}
}