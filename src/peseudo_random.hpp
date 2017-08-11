#pragma once
#include <atomic>
#include <algorithm>

namespace rt {
	struct PeseudoRandom {
		virtual ~PeseudoRandom() {}

		virtual uint32_t generate() = 0;
		virtual double uniform() = 0;
		virtual double uniform(double a, double b) = 0;
	};
	struct Xor : public PeseudoRandom {
		Xor() {

		}
		Xor(uint32_t seed) {
			_y = std::max(seed, 1u);
		}

		// 0 <= x <= 0x7FFFFFFF
		uint32_t generate() {
			_y = _y ^ (_y << 13); _y = _y ^ (_y >> 17);
			uint32_t value = _y = _y ^ (_y << 5); // 1 ~ 0xFFFFFFFF(4294967295
			return value >> 1;
		}
		// 0.0 <= x < 1.0
		double uniform() {
			return double(generate()) / double(0x80000000);
		}
		double uniform(double a, double b) {
			return a + (b - a) * double(uniform());
		}
	public:
		uint32_t _y = 2463534242;
	};
	struct AtomicXor : public PeseudoRandom {
		AtomicXor() {

		}
		AtomicXor(uint32_t seed) {
			_y = std::max(seed, 1u);
		}

		// 0 <= x <= 0x7FFFFFFF
		uint32_t generate() {
			uint32_t expected = _y.load();
			uint32_t desired;
			do {
				uint32_t y = expected;
				y = y ^ (y << 13);
				y = y ^ (y >> 17);
				y = y ^ (y << 5);
				desired = y;
			} while (!_y.compare_exchange_weak(expected, desired));
			uint32_t value = desired; // 1 ~ 0xFFFFFFFF(4294967295
			return value >> 1;
		}
		// 0.0 <= x < 1.0
		double uniform() {
			return double(generate()) / double(0x80000000);
		}
		double uniform(double a, double b) {
			return a + (b - a) * double(uniform());
		}
	public:
		std::atomic<uint32_t> _y = 2463534242;
	};

	// 64bit
	//struct AtomicXor : public PeseudoRandom {
	//	AtomicXor() {

	//	}
	//	AtomicXor(uint32_t seed) {
	//		_y = std::max(seed, 1u);
	//	}

	//	// 0 <= x <= 0x7FFFFFFF
	//	uint32_t generate() {
	//		uint64_t expected = _y.load();
	//		uint64_t desired;
	//		do {
	//			uint64_t y = expected;
	//			y = y ^ (y << 13);
	//			y = y ^ (y >> 7);
	//			y = y ^ (y << 17);
	//			desired = y;
	//		} while (!_y.compare_exchange_weak(expected, desired));
	//		uint64_t value = desired; // 1 ~ 0xFFFFFFFF FFFFFFFF
	//		return uint32_t(value) >> 1;
	//	}
	//	// 0.0 <= x < 1.0
	//	double uniform() {
	//		return double(generate()) / double(0x80000000);
	//	}
	//	double uniform(double a, double b) {
	//		return a + (b - a) * double(uniform());
	//	}
	//public:
	//	std::atomic<uint64_t> _y = 2463534242;
	//};
}