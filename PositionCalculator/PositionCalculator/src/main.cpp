#include "ofMain.h"
#include "ofApp.h"

#include "vector.hpp"
#include <chrono>

int main() {
	ofSetupOpenGL(575 + 600, 575 + 350, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

	//========================================================//
	// Ignore stuff below here. This is just me testing stuff //
	//========================================================//

	Line a({ 0, -2 }, { 10, 8 });
	Line b({ 2, -3 }, { 10, 13 });
	std::cout << a.intersects(b).first << "\n";
	std::cout << a.intersects(b).second.str() << "\n";

	// uint64_t iters = 13954300;
	// {
	// 	auto a = Vec3d(1 + 1E-12, 1 + 1E-12, 1 + 1E-12);
	// 	auto b = Vec3d(123, 456, 789);
	// 	Vec3d res;
	// 
	// 	double start = (double)std::chrono::high_resolution_clock().now().time_since_epoch().count();
	// 	for (uint64_t i = 0; i < iters; ++i) {
	// 		b = a * b;
	// 	}
	// 	double end = (double)std::chrono::high_resolution_clock().now().time_since_epoch().count();
	// 	std::cout << "Elapsed: " << (end - start) / 1000000 << " ms\n";
	// 	std::cout << "Average: " << (end - start) / (double)iters << " ns\n";
	// 	std::cout << b.str() << "\n";
	// }
	// 
	// {
	// 	auto a = glm::vec<3, double>(1 + 1E-15, 1 + 1E-15, 1 + 1E-15);
	// 	auto b = glm::vec<3, double>(123, 456, 789);
	// 	glm::vec<3, double> res;
	// 
	// 	double start = (double)std::chrono::high_resolution_clock().now().time_since_epoch().count();
	// 	for (uint64_t i = 0; i < iters; ++i) {
	// 		b = a * b;
	// 	}
	// 	double end = (double)std::chrono::high_resolution_clock().now().time_since_epoch().count();
	// 	std::cout << "Elapsed: " << (end - start) / 1000000 << " ms\n";
	// 	std::cout << "Average: " << (end - start) / (double)iters << " ns\n";
	// 
	// 	std::cout << b << "\n";
	// }

	return 0;
}
