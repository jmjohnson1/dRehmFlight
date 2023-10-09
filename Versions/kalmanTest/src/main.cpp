#include <eigen3/Eigen/Eigen>
#include "navigation.h"

int main() {
	Navigation nav;
	nav.navigationEquations();
	std::cout << "end" << "\n";
	return 0;
}
