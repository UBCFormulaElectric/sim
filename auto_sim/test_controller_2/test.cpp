#include "controller.hpp"
#include "path.hpp"
#include "perception.hpp"
#include "types.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    // Default to the labels directory next to this source tree; allow overriding via argv[1].
    const std::filesystem::path label_path = argc > 1
        ? std::filesystem::path(argv[1])
        : std::filesystem::path(__FILE__).parent_path() / ".." / "labels" / "0000001.txt";
    std::ifstream infile(label_path);
    // Check if the file opened successfully
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open the file " << label_path << std::endl;
        return 1;
    }

    {
        std::vector<Cone> input_cones { };
        double x, y, a, b, d, e, f;
        std::string c;
        while (infile >> x >> y >> a >> b >> d >> e >> f >> c) {
            ConeColor cone_color;
            if (c == "Cone_Blue") {
                cone_color = ConeColor::BLUE;
            } else if (c == "Cone_Yellow") {
                cone_color = ConeColor::YELLOW;
            } else {
                cone_color = ConeColor::ORANGE;
            }
            input_cones.emplace_back(x, y, cone_color);
        }
        mock_perception(input_cones);
    }
    compute_path_from_percepted_cones();

    // point the car along the center line where it starts, so that progress along the path (theta) means driving forward
    constexpr double x0 = 0, y0 = 0;
    const Eigen::Vector2d heading = spline_t_prime(autodiff::dual2nd(project(x0, y0))).cast<double>();
    const VehicleState state(x0, y0, std::atan2(heading.y(), heading.x()), 1, 0, 0);
    const proxsuite::proxqp::sparse::Vec<double> computation_output = compute(state, 0.0); // first call, so there is no previous solution to shift
    const auto out = get_control(computation_output);
    std::cout << "Control Output: ax=" << out.ax << ", omega_dot=" << out.omega_dot << std::endl;
}