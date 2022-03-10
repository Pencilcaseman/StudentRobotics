#pragma once

#include <vector>
#include <cmath>
#include <cstdint>

#include "vector.hpp"

constexpr int pathFindNull = -1;
constexpr int pathFindDefault = 0;
constexpr int pathFindInvalid = 1;
constexpr int pathFindPath = 2;
constexpr int pathFindTarget = 3;

class PathFinder {
public:
	PathFinder() = default;

	/**
	 * Construct the $PathFinder object. This initializes internal member
	 * variables, but does not populate the point-map
	 *
	 * NOTE: $size should be provided as {width, height}
	 * NOTE: $dims should be provided as {rows, cols}
	 */
	void construct(const Vec2f& size, const Vec2i& dims) {
		m_size = size;
		m_dims = dims.yx(); // Swizzle -- dims now contains {x_cols, y_cols} (i.e. {rows, cols})
		m_offset = size / (dims * 2);
		m_increment = m_size / m_dims;

		clear();
	}

	void clear() {
		m_points = std::vector<std::vector<Vec3f>>(m_dims.x, std::vector<Vec3f>(m_dims.y));
		for (int64_t i = 0; i < m_dims.y; ++i)
			for (int64_t j = 0; j < m_dims.x; ++j)
				m_points[i][j] = m_offset + m_increment * Vec3f{ j, i, pathFindNull };
	}

	void addPoint(const Vec3f& point, float dist = 0) { m_avoidPoints.emplace_back(std::make_pair(point, dist)); }

	void addLine(const Vec3f& start, const Vec3f& end, float dist = 0) {
		m_avoidLines.emplace_back(std::make_pair(std::make_pair(start, end), dist));
	}

	/**
	 * Remove an avoidance point (i.e. a can) around a given location at a set radius.
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

	void setTarget(int64_t row, int64_t col) {
		if (row < 0 || row > m_dims.x - 1 || col < 0 || col > m_dims.y - 1) return;
		m_targetCell = { row, col };
	}

	/**
	 * Takes all avoidance points and lines and bakes the information into the $points
	 * array, reducing the required calculations later on.
	 */
	void bake() {
		clear();

		for (auto& row : m_points) {
			for (auto& point : row) {
				// Check for edges
				for (const auto& line : m_avoidLines) {
					if (abs(point.y - line.first.first.y) < line.second && point.x > line.first.first.x && point.x < line.first.second.x) {
						// Invalid above and below line
						point.z = pathFindInvalid;
					}
					else if (abs(point.x - line.first.first.x) < line.second && point.y > line.first.first.y && point.y < line.first.second.y) {
						// Invalid left and right of line
						point.z = pathFindInvalid;
					}
					// Note -- the z coordinate of these vectors may be non-zero, so we grab the xy component so that stuff actually works
					else if ((point - line.first.first).xy().mag2() < line.second * line.second) {
						// Invalid around radius of line start
						point.z = pathFindInvalid;
					}
					else if ((point - line.first.second).xy().mag2() < line.second * line.second) {
						// Invalid around radius of line end
						point.z = pathFindInvalid;
					}
				}

				// Check for avoidance points
				for (const auto& avoidPoint : m_avoidPoints) {
					if ((avoidPoint.first - point).xy().mag2() < avoidPoint.second * avoidPoint.second) {
						point.z = pathFindInvalid;
					}
				}

				if (point.z != pathFindInvalid) {
					point.z = pathFindDefault;
				}
			}
		}

		// Set the target cell
		if (m_targetCell != Vec2i{ -1, -1 })
			m_points[m_targetCell.x][m_targetCell.y].z = pathFindTarget;
	}

	const std::vector<std::vector<Vec3f>>& getPoints() const { return m_points; }

	// private:
public:
	Vec2f m_size;
	Vec2i m_dims;
	Vec3f m_offset;
	Vec3f m_increment;
	Vec2i m_targetCell = { -1, -1 };
	std::vector<std::pair<Vec3f, float>> m_avoidPoints; // Vector containing {coordinate, distance}
	std::vector<std::pair<std::pair<Vec3f, Vec3f>, float>> m_avoidLines; // Vector containing {{start, end}, distance}
	std::vector<std::vector<Vec3f>> m_points; // List of points with information in z coordinate?
};
