//
// Created by aadc on 03.09.18.
//

#pragma once

#include <math.h>
#include <vector>
#include <algorithm>


/**
 * An implementation of a median filter.
 *
 * @param windowSize The number of values which should be considered.
 *                   This can be changed at any time.
 */
class cMedian {

public:
    cMedian(int windowSize);

    static float median(std::vector<float> values);

    /**
     * Adds a new value to the median filter.
     * If the number of values exceeds the windowSize, the oldest value is discarded.
     *
     * @param value the value to add
     */
    void pushValue(float value);

    /**
     * Calculates the current median on the stored values.
     *
     * @return the median value.
     */
    float calculateMedian();

    /*! the number of values to consider */
    int windowSize;

private:


    /*! Flag indicating whether the median has been calculated for the current input set. */
    bool medianIsUpToDate;

    /*! The most recent median value. */
    float calculatedMedian;

    /*! the most up to date values */
    std::vector<float> storedValues;

};