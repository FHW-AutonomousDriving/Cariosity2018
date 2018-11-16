//
// Created by aadc on 03.09.18.
//

#include "Median.h"

cMedian::cMedian(int windowSize) :  windowSize (windowSize), medianIsUpToDate (false), calculatedMedian (0) {
    storedValues.push_back(0.0f); // to make indexing work when no vaue has yet been entered
}

void cMedian::pushValue(float newValue) {
    // erase oldest values if window size has been reached
    long amountOfOverflownElements = storedValues.size() - windowSize;
    if (amountOfOverflownElements > 0) {
        storedValues.erase(
                storedValues.begin(),
                storedValues.begin() + amountOfOverflownElements
        );
    }

    // append new value
    storedValues.push_back(newValue);
    medianIsUpToDate = false;
}

float cMedian::calculateMedian() {

    if (!medianIsUpToDate) {
        calculatedMedian = cMedian::median(storedValues);
    }
    return calculatedMedian;
}

float cMedian::median(std::vector<float> values) {
    if (values.empty()) return 0;

    // sort values ascending
    std::sort(values.begin(), values.end());

    // 0 / 2 = 0    -> 0
    // 1 / 2 = 0.5  -> 0
    // 2 / 2 = 1    -> 1
    // 3 / 2 = 1.5  -> 1
    // 4 / 2 = 2    -> 2
    // 5 / 2 = 2.5  -> 2
    long medianIndex = values.size() / 2;

    return values.at(medianIndex);
}