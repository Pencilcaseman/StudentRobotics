#pragma once

namespace librapid {
	inline double map(double val,
		double start1, double stop1,
		double start2, double stop2) {
		return start2 + (stop2 - start2) * ((val - start1) / (stop1 - start1));
	}
}