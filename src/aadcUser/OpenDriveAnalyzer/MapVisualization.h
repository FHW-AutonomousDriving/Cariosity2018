#pragma once
#include "stdafx.h"
#include "../Helpers/OpenDriveAnalyzer/OpenDriveMapAnalyzer.h"


#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "map_visualization.filter.user.aadc.cid"

/*! this is the main class for the map visualization. */
class fcMapVisualization : public adtf::ui::cQtUIFilter, QObject
{
public:
    ADTF_CLASS_ID_NAME(fcMapVisualization, CID_CARIOSITY_DATA_TRIGGERED_FILTER, "Map Visualization 2");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::IQtXSystem));

private:

    /*! The invalidated */
    std::atomic_bool  m_bInvalidated;


    /*! Scale for map in x and y direction. */
    tFloat32 m_pixelScaleX;
    /*! The pixel scale y coordinate */
    tFloat32 m_pixelScaleY;

    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;


    /*! The reader position */
    adtf::filter::cPinReader m_oReaderPos;
    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    /*! The display widget */
    DisplayWidget* m_pDisplayWidget;

    /*! The show trace */
    adtf::base::property_variable<tBool> m_ShowTrace = tFalse;

    tBool m_recievedMap = tFalse;

    struct tDDLPointerValueIndex
    {
        tSize address;
    } g_ddlPointerValueIndex;

    adtf::mediadescription::cSampleCodecFactory g_PointerValueFactory;
    adtf::filter::cPinReader m_oReaderOpenDriveMapAnalyzerInstanceSynchronization;

public:
    /*! Default constructor. */
    fcMapVisualization();
    /*! Destructor. */
    ~fcMapVisualization() = default;

protected:
    QWidget* CreateView() override;
    tVoid    ReleaseView() override;
    tResult  OnTimer() override;

private:

    /*!
     * Shows the map.
     *
     * \return  Standard Result Code.
     */
    tResult ShowMap();
    void plotLane(LaneTurnClassification classification, std::vector<ODReader::Pose3D> points);
    tResult readPointerData(adtf::filter::cPinReader &inputPin, tPointerValue &pointerValue);


};