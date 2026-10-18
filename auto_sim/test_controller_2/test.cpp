#include "controller.hpp"
#include "path.hpp"
#include "perception.hpp"
#include "types.hpp"

#include <fstream>
#include <string>
#include <vector>

int main() {
    std::ifstream infile("C:/Users/jessi/Documents/School/fsae/sim/auto_sim/labels/0000001.txt");
    // Check if the file opened successfully
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
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

    const VehicleState state(0, 0, 0, 1, 0, 0);
    const proxsuite::proxqp::sparse::Vec<double> computation_output = compute(state);
    const auto out = get_control(computation_output);
    std::cout << "Control Output: ax=" << out.ax << ", omega_dot=" << out.omega_dot << std::endl;
}