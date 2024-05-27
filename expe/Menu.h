// Menu.h
#ifndef MENU_H
#define MENU_H

#include "CLI11.hpp"
#include <string>

typedef char MenuMode;

class CLIMENU
{
public:
    CLIMENU() : app("Loads an .obj mesh and an .off extracted accumulation then generate a .obj colored mesh and a SDP file"
                    "Typical use example:\n "
                    "\t ./segmentFromAcc \n"
                    "\t ${modelDir}/${modelName}.off \n"
                    "\t ${modelDir}/${sheetName}.dat \n"
                    "\t ${mode} \n"
                    "\t ${shader} \n"
                    "\t ${theta} \n")
    {
        // Set up the options
        app.add_option("-i,--input, 1", inputFile, "An input mesh file in .obj format.")
            ->required()
            ->check(CLI::ExistingFile);
        app.add_option("-e,--extraFile,2", extraData, "An input file containing accumulation, confidence and index to colored.")
            ->required()
            ->check(CLI::ExistingFile);
        app.add_option("-m,--outputMode,3", outputMode, "Input 2 for confidence segmentation.\n"
                                                        "Input 1 for accumulation segmentation.\n"
                                                        "Input 0 for both by default.\n")
            ->required();
        app.add_option("-s,--shaderMode,4", shaderMode, "Input 1 for Hue Shader.\n"
                                                        "Input 0 or default for Gradient Shader.\n")
            ->required();
        app.add_option("-t,--threshold,5", theta, "Threshold value [0,1] for confidence segmentation.\n");
        app.add_option("--outputMesh,6", outputMesh, "Output the colored mesh file");
        app.add_option("--outputSDP, 7", outputSDP, "Output the colored SDP file");

        app.get_formatter()->column_width(40);

        fileName = inputFile.substr(0, inputFile.length() - 4);
    }

    int menuParse(int argc, char **argv)
    {
        CLI11_PARSE(app, argc, argv);
        fileName = extraData.substr(0, extraData.length() - 4);
        return 0;
    }

    std::string inputFile;
    std::string extraData;
    std::string outputMesh;
    std::string outputSDP;
    std::string fileName;
    MenuMode outputMode = 2;
    MenuMode shaderMode = 0;
    float theta = 0.5f;

private:
    CLI::App app;
};

#endif // CLIOPTIONS_H
