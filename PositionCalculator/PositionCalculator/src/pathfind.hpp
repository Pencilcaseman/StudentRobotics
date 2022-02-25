#pragma once

#include <vector>
#include <cmath>
#include <cstdint>

#include "vector.hpp"

class PathFinder {
public:
	PathFinder() { }

	/**
	 * Construct the $PathFinder object. This initializes internal member
	 * variables, but does not populate the point-map
	 *
	 * NOTE: $size should be provided as {width, height}
	 * NOTE: $dims should be provided as {rows, cols}
	 */
	void construct(const Vec2f& size, const Vec2i& dims) {
		m_size = size;
		m_dims = {dims.y, dims.x}; // Swizzle -- dims now contains {x_cols, y_cols}
		m_offset = size / (dims * 2);
		m_increment = m_size / m_dims;

		m_points = std::vector<std::vector<Vec3f>>(dims.x, std::vector<Vec3f>(dims.y));
		for (int64_t i = 0; i < m_dims.y; ++i)
			for (int64_t j = 0; j < m_dims.x; ++j)
				m_points[i][j] = m_offset + m_increment * Vec3f{i, j};
	}

	/**
	 * Remove an avoidance point (i.e. can) around a given location at a set radius.
	 * Any points within this radius from the provided coordinate will be removed.
	 *
	 * NOTE: This does not remove any baked values. Make sure this is taken into account
	 */
	bool removePointNear(const Vec3f& point, float maxAbsDist = 1e-5f) {
		std::vector<size_t> indices;

		for (size_t i = 0; i < m_avoidPoints.size(); ++i)
			if ((m_avoidPoints[i].first - point).mag2() > maxAbsDist * maxAbsDist)
				indices.emplace_back(i); // Keep this

		if (indices.size() == m_avoidPoints.size()) return false;

		std::vector<std::pair<Vec3f, float>> tmp;
		for (const auto& index : indices)
			tmp.emplace_back(m_avoidPoints[index]);
		std::swap(m_avoidPoints, tmp);

		return true;
	}

private:
	Vec2f m_size;
	Vec2i m_dims;
	Vec3f m_offset;
	Vec3f m_increment;
	std::vector<std::pair<Vec3f, float>> m_avoidPoints; // Vector containing {coordinate, distance}
	std::vector<std::vector<Vec3f>> m_points; // List of points with information in z coordinate?
};
