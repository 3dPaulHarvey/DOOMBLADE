#ifndef GRAPH_PLOTTER_H
#define GRAPH_PLOTTER_H

#include <vector>
#include <string>
#include <fstream>
#include <cstdio>

class GraphPlotter {
public:
    GraphPlotter(const std::string& dataFileName = "torques.dat")
        : dataFileName(dataFileName) {}

    void plot(const std::vector<float>& torques, const std::string& title = "Torque Plot") {
        // Write data to a temporary file
        std::ofstream dataFile(dataFileName);
        for (size_t i = 0; i < torques.size(); i++) {
            dataFile << i << " " << torques[i] << std::endl;
        }
        dataFile.close();

        // Open a pipe to GNUplot
        FILE *gnuplotPipe = popen("gnuplot -persistent", "w");
        if (gnuplotPipe != nullptr) {
            // Set plot title, labels, and plot data using linespoints
            fprintf(gnuplotPipe, "set title '%s'\n", title.c_str());
            fprintf(gnuplotPipe, "set xlabel 'Time (control loop iterations)'\n");
            fprintf(gnuplotPipe, "set ylabel 'Torque (units)'\n");
            fprintf(gnuplotPipe, "plot '%s' with linespoints title 'Torque over Time'\n", dataFileName.c_str());
            pclose(gnuplotPipe);
        } else {
            std::cerr << "Failed to open pipe to GNUplot." << std::endl;
        }
    }

private:
    std::string dataFileName;
};

#endif // GRAPH_PLOTTER_H
