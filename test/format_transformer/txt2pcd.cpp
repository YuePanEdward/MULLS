#include "dataio.hpp"
#include "utility.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace lo;

// Brief: Do the batch txt2pcd transformation

int main(int argc, char **argv)
{
    // The file to read from.
    string infile = argv[1];

    // The file to output to.
    string outfile = argv[2];

    std::cout << "Transform begin" << std::endl;

    // Load point cloud
    fstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.good())
    {
        cerr << "Could not read file: " << infile << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    pcTPtr pointCloud(new pcT);

    DataIo<Point_T> dataio;
    dataio.read_txt_file(infile, pointCloud);

    cout << "Read txt file from [" << infile << "]: " << pointCloud->points.size() << " points, writing to [" << outfile << "]" << endl;

    dataio.write_pcd_file(outfile, pointCloud);

    std::cout << "Transform done" << std::endl;
}
