//
// Created by aadc on 06.09.18.
//

#include "Map.h"

#define CONSOLE_LOG(_text, _log_level) if (m_enableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages


namespace fhw {

    cMap::cMap() {}


    cEntity cMap::get(tFloat32 x, tFloat32 y) {
        // convert from float to index and access array
        tUInt64 xIdx = mapFloatToIndex(x);
        tUInt64 yIdx = mapFloatToIndex(y);
        if (isInitialised && xIdx >= 0 && xIdx < width && yIdx >= 0 && yIdx < height) {
            return matrix[xIdx][yIdx];
        } else {
            return cEntity();
        }
    }

    void cMap::set(tFloat32 x, tFloat32 y, eEntityType value) {
        // convert from float to index and access array
        tUInt64 xIdx = mapFloatToIndex(x);
        tUInt64 yIdx = mapFloatToIndex(y);
        // LOG_INFO("Adding object %d at coordinate %f, %f to map at %d, %d", value, x, y, xIdx, yIdx);
        if (isInitialised && xIdx >= 0 && xIdx < width && yIdx >= 0 && yIdx < height) {
            matrix[xIdx][yIdx].setEntityType(value);
        }
    }


    cMap& cMap::getInstance() {
        static cMap map;
        return map;
    }

    void cMap::setup(cString filePath, tUInt32 resolution) {
        if (isInitialised) {
            return;
        }

        this->resolution = resolution;


        // TODO read XML
        // TODO get smallest rectangle
        // TODO get dimensions

        tFloat32 minWidth = 10;
        tFloat32 minHeight = 10;

        // ...
        width =  mapFloatToIndex(minWidth);
        height = mapFloatToIndex(minHeight);

        matrix.resize(height, std::vector<cEntity>(width));

        for(unsigned int y=0; y<height; ++y)
            for(unsigned int x=0; x<width; ++x) {
                matrix[x][y] = cEntity();
        }

        isInitialised = tTrue;
        matMap = cv::Mat(int(height), int(width), CV_8UC1);
    }


    tUInt64 cMap::mapFloatToIndex(tFloat32 value) {
        return tUInt64(ceil(value * this->resolution));
    }

    cv::Mat cMap::generateImage(tVehiclePosition currentPos) {
        for(int row=0; row<matMap.rows; ++row) {
            int y = matMap.rows - row - 1;

            for(int column=0; column<matMap.cols; ++column) {
                int x = column;
                cEntity entity = matrix.at(x).at(y);
                entity.calculateConfidenceDecay();

                tUInt16 ui16Conf = entity.getConfidence();
//                tFloat32 percentage = tFloat32(ui16Conf) / numeric_limits<tUInt16>::max();
//                tFloat32 floaty = percentage * numeric_limits<tUInt8>::max();
//                tUInt8 ui8Conf = tUInt8(floaty);

                matMap.at<tUInt8>(row, column) = entity.getEntityType() != EMPTY ? tUInt8(ui16Conf) : tUInt8(1);
                if (ui16Conf != 0) {
                    CONSOLE_LOG_INFO(cString::Format("Confidence at: %d: %d -- %u || %u", column, row, matMap.at<tUInt8>(row, column), ui16Conf));
                }

            }
        }
        cv::Mat result;
        cv::cvtColor(matMap, result, cv::COLOR_GRAY2RGB);

        cv::Point2f anchor = cv::Point2f(
                currentPos.f32x * resolution,
                matMap.rows - 1 - (currentPos.f32y * resolution)
        );
        cv::Point2f tip = cv::Point2f(
                (currentPos.f32x + cos(currentPos.f32heading) * 0.395f) * resolution,
                matMap.rows - 1 - ((currentPos.f32y + sin(currentPos.f32heading) * 0.395f) * resolution)
        );
        circle(result, anchor, 1, cv::Scalar(255, 0, 0), -1);
        circle(result, tip, 1, cv::Scalar(0, 255, 0), -1);

        return result;
    }

};