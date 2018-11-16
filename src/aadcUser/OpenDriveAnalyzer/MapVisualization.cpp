#include "MapVisualization.h"

ADTF_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER, fcMapVisualization)

fcMapVisualization::fcMapVisualization() : adtf::ui::cQtUIFilter(), m_pDisplayWidget(nullptr) {

    //Add position pin from mediadescription
    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory)) {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    } else {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    //Create Pin
    adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pConstTypePositionData = pTypePositionData;
    adtf::streaming::create_pin(*this, m_oReaderPos, "position", pConstTypePositionData);


    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePointerValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPointerValue", pTypePointerValue, g_PointerValueFactory))
    {
        adtf_ddl::access_element::find_index(g_PointerValueFactory, "ui64Address", g_ddlPointerValueIndex.address);
    } else {
        LOG_WARNING("No mediadescription for tPointerValue found!");
    }
    adtf::streaming::create_pin(*this, m_oReaderOpenDriveMapAnalyzerInstanceSynchronization, "OpenDriveMapAnalyzerInstanceSynchronization", pTypePointerValue);

    //Register Properties
    RegisterPropertyVariable("Show Position Trace", m_ShowTrace);
}

QWidget *fcMapVisualization::CreateView() {
    //Create Widget
    m_pDisplayWidget = new DisplayWidget(nullptr);

    //Draw map to widget
    return m_pDisplayWidget;
}


tVoid fcMapVisualization::ReleaseView() {
    m_pDisplayWidget = nullptr;
}

tResult fcMapVisualization::OnTimer() {
    //Read Position values
    tFloat32 f32x, f32y, f32heading;
    //Get Sample
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pReadSample;
    while (IS_OK(m_oReaderPos.GetNextSample(pReadSample))) {
        //Get Position values
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        f32x = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.x);
        f32y = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.y);
        f32heading = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.heading);
        //Convert to Pixel Coordinates
        tFloat32 pixX = m_pixelScaleX * (f32x - cOpenDriveMapAnalyzer::singleton->m_minX);
        tFloat32 pixY = GRAPHICSSCENE_HEIGHT - m_pixelScaleY * (f32y - cOpenDriveMapAnalyzer::singleton->m_minY);
        //Plot if within the graphicsscene
        if (pixX > 0 && pixX < GRAPHICSSCENE_WIDTH && pixY > 0 && pixY < GRAPHICSSCENE_HEIGHT) {
            m_pDisplayWidget->PlotPosition(pixX, pixY, f32heading, m_ShowTrace);
        }
    }


    tPointerValue value;
    RETURN_IF_FAILED(readPointerData(m_oReaderOpenDriveMapAnalyzerInstanceSynchronization, value));

    cOpenDriveMapAnalyzer::singleton = reinterpret_cast<cOpenDriveMapAnalyzer*>(value.address);
    if (!cOpenDriveMapAnalyzer::isInitialized()) {
        LOG_ERROR("Map: ERR_NOT_INITIALIZED.");
        //TODO: return cError(ERR_NOT_INITIALIZED);
        RETURN_NOERROR;
    }

    LOG_INFO("Received %u junction entries and %u casual lanes.", cOpenDriveMapAnalyzer::singleton->junctionEntries.size(), cOpenDriveMapAnalyzer::singleton->casualLanes.size());
    m_recievedMap = tTrue;

    static tBool isInit = tFalse;
   // LOG_INFO("On Timer: %d %d %d", isInit, cOpenDriveMapAnalyzer::singleton->junctionEntries.size(), cOpenDriveMapAnalyzer::singleton->casualLanes.size());
    if (m_recievedMap && (!cOpenDriveMapAnalyzer::singleton->junctionEntries.empty() || !cOpenDriveMapAnalyzer::singleton->casualLanes.empty())) { //TODO: what if a map gets loaded multiple times?
        LOG_INFO("Map loaded :-)");
        m_pDisplayWidget->ResetScene();
        ShowMap();
    }

    RETURN_NOERROR;
}


tResult fcMapVisualization::ShowMap() {

    //Find the minimum and maximum X and y points
    cOpenDriveMapAnalyzer::singleton->calculateSize();


    //Calculate scale from the fixed width and height
    m_pixelScaleX = GRAPHICSSCENE_WIDTH / fabs(cOpenDriveMapAnalyzer::singleton->m_maxX - cOpenDriveMapAnalyzer::singleton->m_minX);
    m_pixelScaleY = GRAPHICSSCENE_HEIGHT / fabs(cOpenDriveMapAnalyzer::singleton->m_maxY - cOpenDriveMapAnalyzer::singleton->m_minY);
    //Use the minimum scale of the two
    if (m_pixelScaleX < m_pixelScaleY) {
        m_pixelScaleY = m_pixelScaleX;
    } else {
        m_pixelScaleX = m_pixelScaleY;
    }
    LOG_INFO("Min X %.3f Max X %.3f Scale X %.3f", cOpenDriveMapAnalyzer::singleton->m_minX, cOpenDriveMapAnalyzer::singleton->m_maxX, m_pixelScaleX);
    LOG_INFO("Min Y %.3f Max Y %.3f Scale Y %.3f", cOpenDriveMapAnalyzer::singleton->m_minY, cOpenDriveMapAnalyzer::singleton->m_maxY, m_pixelScaleY);
    LOG_INFO("Min Z %.3f Max Z %.3f", cOpenDriveMapAnalyzer::singleton->m_minZ, cOpenDriveMapAnalyzer::singleton->m_maxZ);

    for (const auto &entry : cOpenDriveMapAnalyzer::singleton->junctionEntries) {
        plotLane(JUNCTION_RIGHT_TURN, entry.rightTurn);
        plotLane(JUNCTION_LEFT_TURN, entry.leftTurn);
        plotLane(JUNCTION_STRAIGHT, entry.straight);
    }
    for (const auto &casualLane : cOpenDriveMapAnalyzer::singleton->casualLanes) {
        plotLane(CASUAL, casualLane);
    }

    LOG_INFO("Found %u junction entries.", cOpenDriveMapAnalyzer::singleton->junctionEntries.size());

    RETURN_NOERROR;
}


void fcMapVisualization::plotLane(LaneTurnClassification classification, std::vector<ODReader::Pose3D> points) {
    if (points.empty()) return;

    QColor color = QColor(0, 128, 128);

    switch (classification) {
        case JUNCTION_RIGHT_TURN:   color = QColor(255, 0, 0);  break;
        case JUNCTION_LEFT_TURN:    color = QColor(0, 255, 0);  break;
        case JUNCTION_STRAIGHT:     color = QColor(0, 127, 255);  break;
        default:                                                break;
    }


    float entryX = (points.front().p.x - cOpenDriveMapAnalyzer::singleton->m_minX) * m_pixelScaleX;
    float entryY = GRAPHICSSCENE_HEIGHT - (points.front().p.y - cOpenDriveMapAnalyzer::singleton->m_minY) * m_pixelScaleY;

    float exitX = (points.back().p.x - cOpenDriveMapAnalyzer::singleton->m_minX) * m_pixelScaleX;
    float exitY = GRAPHICSSCENE_HEIGHT - (points.back().p.y - cOpenDriveMapAnalyzer::singleton->m_minY) * m_pixelScaleY;

    if (classification != CASUAL) {
        tFloat32 offset = int(classification) * 10.0f;
        m_pDisplayWidget->PlotText(entryX, entryY - offset, QString::number(openDriveReader::toEulerianAngle(points.front().q).yaw * 180.0 / M_PI), color);

        m_pDisplayWidget->PlotCircle(entryX, entryY, QColor(0, 255, 0));
        m_pDisplayWidget->PlotCircle(exitX, exitY, QColor(255, 0, 0));
        float angleLength = 20.0f;

        float angle = openDriveReader::toEulerianAngle(points.front().q).yaw;
        float targetLeftX  = entryX + cos(angle - HEADING_DIFF_OF_EQUALITY) * angleLength;
        float targetLeftY  = entryY - sin(angle - HEADING_DIFF_OF_EQUALITY) * angleLength;
        float targetRightX  = entryX + cos(angle + HEADING_DIFF_OF_EQUALITY) * angleLength;
        float targetRightY  = entryY - sin(angle + HEADING_DIFF_OF_EQUALITY) * angleLength;
        
        m_pDisplayWidget->DrawLine(entryX, entryY, targetRightX, targetRightY, QColor(255, 255, 255));
        m_pDisplayWidget->DrawLine(entryX, entryY, targetLeftX, targetLeftY, QColor(255, 255, 255));
    }

    for (unsigned int j = 0; j < points.size() - 1; j++) {
        //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
        //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
        //Note Y direction is flipped as map and pixel is opposite in y-direction
        //Pixel coordinates
        float pixX1 = (points[j].p.x - cOpenDriveMapAnalyzer::singleton->m_minX) * m_pixelScaleX;
        float pixX2 = (points[j + 1].p.x - cOpenDriveMapAnalyzer::singleton->m_minX) * m_pixelScaleX;
        float pixY1 = GRAPHICSSCENE_HEIGHT - (points[j].p.y - cOpenDriveMapAnalyzer::singleton->m_minY) * m_pixelScaleY;
        float pixY2 = GRAPHICSSCENE_HEIGHT - (points[j + 1].p.y - cOpenDriveMapAnalyzer::singleton->m_minY) * m_pixelScaleY;

        //Send pixel coordinates to draw
        m_pDisplayWidget->DrawLine(pixX1, pixY1, pixX2, pixY2, color);
    }

}


tResult fcMapVisualization::readPointerData(adtf::filter::cPinReader &inputPin, tPointerValue &pointerValue) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    adtf::ucom::object_ptr<const adtf::streaming::ISample> pReadSample;
    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));

    auto oDecoder = g_PointerValueFactory.MakeDecoderFor(*pReadSample);
    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlPointerValueIndex.address, &(pointerValue.address)));

    RETURN_NOERROR;
}