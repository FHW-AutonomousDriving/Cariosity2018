#pragma once

#include <opencv2/opencv.hpp>
#include <limits>
#include "Entity.h"
#include "../../Helpers/cariosity_structs.h"

using namespace adtf_util;
using namespace std;



namespace fhw {

    class cMap {

        // - fields:
        // tree?: map of road segments + signs + PhysicalObject with confidence


    private:

        tBool m_enableConsoleOutput = tFalse;

        cMap();

        std::vector<std::vector<cEntity>> matrix;

        tUInt32 resolution;
        tUInt64 mapFloatToIndex(tFloat32 value);
        tUInt64 width;
        tUInt64 height;
    public:

        /*! Default constructor. */
        cMap(const cMap&) = delete;
        void operator=(cMap const &) = delete;

        static cMap& getInstance();

        tBool isInitialised;
        cv::Mat matMap;

        cEntity get(tFloat32 x, tFloat32 y);
        void set(tFloat32 x, tFloat32 y, eEntityType value);
        void setup(cString filePath, tUInt32 resolution);

        cv::Mat generateImage(tVehiclePosition currentPos);

    };






};