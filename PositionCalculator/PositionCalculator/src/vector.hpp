#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>

template<typename DTYPE, int64_t dims>
class Vec {
	template<typename T>
	using Common = typename std::common_type<DTYPE, T>::type;

public:
	Vec() = default;

	template<typename X, typename ...YZ>
	Vec(X x, YZ ... yz) : m_components{ (DTYPE)x, (DTYPE)yz... } {
		static_assert(1 + sizeof...(YZ) <= dims, "Parameters cannot exceed vector dimensions");
	}

	Vec(const Vec<DTYPE, dims>& other) {
		int64_t i;
		for (i = 0; i < dims; ++i) m_components[i] = other.m_components[i];
	}

	Vec<DTYPE, dims>& operator=(const Vec<DTYPE, dims>& other) {
		if (this == &other) return *this;
		for (int64_t i = 0; i < dims; ++i) m_components[i] = other.m_components[i];
		return *this;
	}

	/**
	 * Implement indexing (const and non-const)
	 * Functions take a single index and return a scalar value
	 */

	const DTYPE& operator[](int64_t index) const { return m_components[index]; }
	DTYPE& operator[](int64_t index) { return m_components[index]; }

	/**
	 * Implement simple arithmetic operators + - * /
	 *
	 * Operations take two Vec objects and return a new vector (with common type)
	 * containing the result of the element-wise operation.
	 *
	 * Vectors must have same dimensions. To cast, use Vec.as<TYPE, DIMS>()
	 */

	template<typename T>
	Vec<Common<T>, dims> operator+(const Vec<T, dims>& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] + other[i];
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator-(const Vec<T, dims>& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] - other[i];
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator*(const Vec<T, dims>& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] * other[i];
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator/(const Vec<T, dims>& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] / other[i];
		return res;
	}

	/**
	 * Implement simple arithmetic operators + - * /
	 *
	 * Operations take a vector and a scalar, and return a new vector (with common type)
	 * containing the result of the element-wise operation.
	 */

	template<typename T>
	Vec<Common<T>, dims> operator+(const T& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] + other;
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator-(const T& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] - other;
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator*(const T& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] * other;
		return res;
	}

	template<typename T>
	Vec<Common<T>, dims> operator/(const T& other) const {
		Vec<Common<T>, dims> res;
		for (int64_t i = 0; i < dims; ++i) res[i] = m_components[i] / other;
		return res;
	}

	/**
	 * Return the magnitude squared of a vector
	 */
	DTYPE mag2() const {
		DTYPE res = 0;
		for (const auto& val : m_components) res += val * val;
		return res;
	}

	/**
	 * Return the magnitude of a vector
	 */
	DTYPE mag() const {
		return sqrt(mag2());
	}

	/**
	 * Compute the vector dot product
	 * AxBx + AyBy + AzCz + ...
	 */
	template<typename T>
	Common<T> dot(const Vec<T, dims>& other) {
		Common<T> res = 0;
		for (int64_t i = 0; i < dims; ++i) res += m_components[i] * other[i];
		return res;
	}

	/**
	 * Compute the vector cross product
	 */
	template<typename T>
	Vec<Common<T>, dims> cross(const Vec<T, dims>& other) const {
		static_assert(dims == 2 || dims == 3, "Only 2D and 3D vectors support the cross product");

		Vec<Common<T>, dims> res;

		if constexpr (dims == 2) {
			m_components[2] = 0;
			other[2] = 0;
		}

		res.x = y * other.z - z * other.y;
		res.y = z * other.x - x * other.z;
		res.z = x * other.y - y * other.x;

		return res;
	}

	std::string str() const {
		std::string res = "(";
		for (int64_t i = 0; i < dims; ++i) res += std::to_string(m_components[i]) + (i == dims - 1 ? ")" : ", ");
		return res;
	}

	DTYPE& x = m_components[0];
	DTYPE& y = m_components[1];
	DTYPE& z = m_components[2];
	DTYPE& w = m_components[3];
private:
	DTYPE m_components[dims > 4 ? dims : 4];
};

/**
 * Implement simple arithmetic operators + - * /
 *
 * Operations take a scalar and a vector and return a new vector (with common type)
 * containing the result of the element-wise operation.
 */

template<typename T, typename DTYPE, int64_t dims>
Vec<typename std::common_type<T, DTYPE>::type, dims> operator+(const T& value, const Vec<DTYPE, dims>& vec) {
	Vec<typename std::common_type<T, DTYPE>::type, dims> res;
	for (int64_t i = 0; i < dims; ++i) res[i] = value + vec[i];
	return res;
}

template<typename T, typename DTYPE, int64_t dims>
Vec<typename std::common_type<T, DTYPE>::type, dims> operator-(const T& value, const Vec<DTYPE, dims>& vec) {
	Vec<typename std::common_type<T, DTYPE>::type, dims> res;
	for (int64_t i = 0; i < dims; ++i) res[i] = value - vec[i];
	return res;
}

template<typename T, typename DTYPE, int64_t dims>
Vec<typename std::common_type<T, DTYPE>::type, dims> operator*(const T& value, const Vec<DTYPE, dims>& vec) {
	Vec<typename std::common_type<T, DTYPE>::type, dims> res;
	for (int64_t i = 0; i < dims; ++i) res[i] = value * vec[i];
	return res;
}

template<typename T, typename DTYPE, int64_t dims>
Vec<typename std::common_type<T, DTYPE>::type, dims> operator/(const T& value, const Vec<DTYPE, dims>& vec) {
	Vec<typename std::common_type<T, DTYPE>::type, dims> res;
	for (int64_t i = 0; i < dims; ++i) res[i] = value / vec[i];
	return res;
}

using Vec2i = Vec<int64_t, 2>;
using Vec2f = Vec<float, 2>;
using Vec2d = Vec<double, 2>;

using Vec3i = Vec<int64_t, 3>;
using Vec3f = Vec<float, 3>;
using Vec3d = Vec<double, 3>;

using Vec4i = Vec<int64_t, 4>;
using Vec4f = Vec<float, 4>;
using Vec4d = Vec<double, 4>;
