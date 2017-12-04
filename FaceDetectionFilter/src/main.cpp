#include <iostream>
#include <string>
#include <vector>

#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "common/CSVRow.h"
#include "common/math.h"
#include "common/get_arguments.h"
#include "FaceDetectionFilter/version.h"
#include "FaceDetectionFilter/FaceDetectionFilter.h"

using namespace ar::FaceDetectionFilter;
using namespace ar::common;

using std::max;
using std::min;

struct Parameters
{
    std::string inputCsvFile;
    std::string outputCsvFile;
    double filterInterval;

    Parameters()
    {
        inputCsvFile = std::string("results.csv");
        outputCsvFile = std::string("filtered.csv");
        filterInterval = 0.05;
    }

    void print()
    {
        std::cout << "inputCsvFile \t" << inputCsvFile << std::endl;
        std::cout << "outputCsvFile \t" << outputCsvFile << std::endl;
        std::cout << "filterInterval \t" << filterInterval << std::endl;
    }

    void applyArguments(
        std::vector<std::string>& arguments
    ) 
    {
        std::vector<bool> consumeArgument(arguments.size());
        for(int i=0; i < arguments.size(); i++)
        {
            consumeArgument.push_back(false);
        }

        for (size_t i = 1; i < arguments.size(); ++i)
        {
            consumeArgument[i] = false;

            if (arguments[i].compare("-input") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    inputCsvFile = arguments[i + 1];
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-output") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    outputCsvFile = arguments[i + 1];
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-filterInterval") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                   std::stringstream sstream(arguments[i + 1]);
                    double value;
                    sstream >> value;
                    filterInterval = value;
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
        }
        for (int i = (int)arguments.size() - 1; i >= 0; --i)
        {
            if (consumeArgument[i])
            {
                arguments.erase(arguments.begin() + i);
            }
        }
    }
};



int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;

    auto arguments = get_arguments(argc, argv);

    FaceDetectionFilterParameters filterParameters;
    filterParameters.applyArguments(arguments);

    Parameters parameters;
    parameters.applyArguments(arguments);

    parameters.print();

    std::ifstream inputFile(parameters.inputCsvFile);
    CSVRow header;
    if(!(inputFile >> header))
    {
        std::cerr << "could not read headers" << std::endl;
        return -1;
    }

    int idx_time = header.find("time", "\"", "\"");
    int idx_detectionCertainty = header.find("detectionCertainty", "\"", "\"");
    int idx_x = header.find("x", "\"", "\"");
    int idx_y = header.find("y", "\"", "\"");
    int idx_z = header.find("z", "\"", "\"");
    int idx_eulerX = header.find("eulerX", "\"", "\"");
    int idx_eulerY = header.find("eulerY", "\"", "\"");
    int idx_eulerZ = header.find("eulerZ", "\"", "\"");
    if (idx_eulerX<0) idx_eulerX = header.find("euler0", "\"", "\"");
    if (idx_eulerY<0) idx_eulerY = header.find("euler1", "\"", "\"");
    if (idx_eulerZ<0) idx_eulerZ = header.find("euler2", "\"", "\"");
    int max_idx = max(
        max(
            max(idx_time, idx_detectionCertainty),
            max(idx_x, idx_y)
        ),
        max(
            max(idx_z, idx_eulerX),
            max(idx_eulerY, idx_eulerZ)
        )
    );
    int min_idx = min(
        min(
            min(idx_time, idx_detectionCertainty),
            min(idx_x, idx_y)
        ),
        min(
            min(idx_z, idx_eulerX),
            min(idx_eulerY, idx_eulerZ)
        )
    );
    if(min_idx < 0)
    {
        std::cerr << "could not find all header names" << std::endl;
        std::cerr << "idx_time\t" << idx_time << std::endl;
        std::cerr << "idx_detectionCertainty\t" << idx_detectionCertainty << std::endl;
        std::cerr << "idx_x\t" << idx_x << std::endl;
        std::cerr << "idx_y\t" << idx_y << std::endl;
        std::cerr << "idx_z\t" << idx_z << std::endl;
        std::cerr << "idx_eulerX\t" << idx_eulerX << std::endl;
        std::cerr << "idx_eulerY\t" << idx_eulerY << std::endl;
        std::cerr << "idx_eulerZ\t" << idx_eulerZ << std::endl;
        return -1;
    }


    std::cout 
        << std::fixed
        << std::setprecision(3)
        ;

    std::vector<cv::Vec<double, 8>> dataVector;
    CSVRow row;
    double startTime = -1;
    double endTime;
    while(inputFile >> row)
    {
        if(max_idx > row.size()) continue;

        cv::Vec<double, 8> rowVec(
           row.getAs<double>(idx_time) / (750000. * 20.),
           row.getAs<double>(idx_detectionCertainty),
           row.getAs<double>(idx_x),
           row.getAs<double>(idx_y),
           row.getAs<double>(idx_z),
           row.getAs<double>(idx_eulerX),
           row.getAs<double>(idx_eulerY),
           row.getAs<double>(idx_eulerZ)
        );
        if(startTime < 0)
        {
            startTime = rowVec[0];
        }
        endTime = rowVec[0];
        // std::cout << rowVec << std::endl;
        dataVector.push_back(rowVec);
    }

    cv::Mat_<double> dataMatrix(dataVector.size(), 8, &(dataVector[0][0]));

    std::ofstream outputFile;
    outputFile.open(parameters.outputCsvFile, std::ios::out);
    outputFile << "\"time\"" 
        << ";" << "\"x\""
        << ";" << "\"y\"" 
        << ";" << "\"z\"" 
        << ";" << "\"eulerX\"" 
        << ";" << "\"eulerY\"" 
        << ";" << "\"eulerZ\""
        << ";" << "\"filtered_x\""
        << ";" << "\"filtered_y\"" 
        << ";" << "\"filtered_z\"" 
        << ";" << "\"filtered_eulerX\"" 
        << ";" << "\"filtered_eulerY\"" 
        << ";" << "\"filtered_eulerZ\""
        << ";" << "\"residual_x\""
        << ";" << "\"residual_y\"" 
        << ";" << "\"residual_z\"" 
        << ";" << "\"residual_eulerX\"" 
        << ";" << "\"residual_eulerY\"" 
        << ";" << "\"residual_eulerZ\""
        << std::fixed
        << std::setprecision(3)
        << std::endl;

    FaceDetectionFilter filter(filterParameters);

    if(parameters.filterInterval == 0)
    {
        for(int i=0; i<dataMatrix.rows; i++)
        {
            cv::Vec6d headPose(
                dataMatrix(i, 2),
                dataMatrix(i, 3),
                dataMatrix(i, 4),
                dataMatrix(i, 5),
                dataMatrix(i, 6),
                dataMatrix(i, 7)
            );
            cv::Vec6d filteredHeadPose = filter.update(
                dataMatrix(i, 0) - startTime, headPose, dataMatrix(i, 1)
            );
            outputFile
                << filter.getFilterTime()
                << ";" << headPose[0]
                << ";" << headPose[1]
                << ";" << headPose[2]
                << ";" << headPose[3]
                << ";" << headPose[4]
                << ";" << headPose[5]
                << ";" << filteredHeadPose[0]
                << ";" << filteredHeadPose[1]
                << ";" << filteredHeadPose[2]
                << ";" << filteredHeadPose[3]
                << ";" << filteredHeadPose[4]
                << ";" << filteredHeadPose[5]
                << ";" << headPose[0] - filteredHeadPose[0]
                << ";" << headPose[1] - filteredHeadPose[1]
                << ";" << headPose[2] - filteredHeadPose[2]
                << ";" << headPose[3] - filteredHeadPose[3]
                << ";" << headPose[4] - filteredHeadPose[4]
                << ";" << headPose[5] - filteredHeadPose[5]
                << std::endl;

        }
    }
    else
    {
        int i=0;
        double time = startTime;
        while(time < endTime)
        {
            if( 240.745 <= time && time <= 240.755)
            {
                std::cout << "now" << std::endl;
            }
            cv::Vec6d filteredHeadPose;
            if(time < dataMatrix(i, 0))
            {
                // predict
                filteredHeadPose = filter.predict(time - startTime);            
                outputFile
                    << filter.getFilterTime()
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << filteredHeadPose[0]
                    << ";" << filteredHeadPose[1]
                    << ";" << filteredHeadPose[2]
                    << ";" << filteredHeadPose[3]
                    << ";" << filteredHeadPose[4]
                    << ";" << filteredHeadPose[5]
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << ";" << ""
                    << std::endl;
            }
            else if (dataMatrix(i,0) <= time)
            {
                // observe
                cv::Vec6d headPose(
                    dataMatrix(i, 2),
                    dataMatrix(i, 3),
                    dataMatrix(i, 4),
                    dataMatrix(i, 5),
                    dataMatrix(i, 6),
                    dataMatrix(i, 7)
                );
                filteredHeadPose = filter.update(
                    dataMatrix(i, 0) - startTime, headPose, dataMatrix(i, 1)
                );
                filteredHeadPose = filter.predict(time - startTime);            
                outputFile
                    << filter.getFilterTime()
                    << ";" << headPose[0]
                    << ";" << headPose[1]
                    << ";" << headPose[2]
                    << ";" << headPose[3]
                    << ";" << headPose[4]
                    << ";" << headPose[5]
                    << ";" << filteredHeadPose[0]
                    << ";" << filteredHeadPose[1]
                    << ";" << filteredHeadPose[2]
                    << ";" << filteredHeadPose[3]
                    << ";" << filteredHeadPose[4]
                    << ";" << filteredHeadPose[5]
                    << ";" << headPose[0] - filteredHeadPose[0]
                    << ";" << headPose[1] - filteredHeadPose[1]
                    << ";" << headPose[2] - filteredHeadPose[2]
                    << ";" << headPose[3] - filteredHeadPose[3]
                    << ";" << headPose[4] - filteredHeadPose[4]
                    << ";" << headPose[5] - filteredHeadPose[5]
                    << std::endl;

                i++;
            }

            time += parameters.filterInterval;
        }
    }


    outputFile.close();

}
