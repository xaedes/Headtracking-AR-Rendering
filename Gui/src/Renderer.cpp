#include "Renderer.h"
#include "common/math.h"
#include "common/Maybe.h"

namespace ar {

using common::Maybe;

Renderer::Renderer(const CoordinateSystems::Instances& coordSys)
    : m_coordSys(coordSys)
    // , m_scene(iffrenderer::SceneManager::GetInstance(iffrenderer::WindowType::Headless))
    , m_scene(iffrenderer::SceneManager::GetInstance(iffrenderer::WindowType::Windowed))
{
    m_window = m_scene.GetWindow();
    m_camera = m_scene.GetCamera();
    m_camera->SetFOV(45.0f);
    m_camera->SetResolution(800, 600);

    m_windTurbine = m_scene.CreateEntityFromFile("data/wtt_ned.obj");
    m_windTurbineNode = m_scene.CreateSceneNode("wind_turbine");
    m_windTurbineNode->AttachObject(m_windTurbine.get());
    m_windTurbineNode->SetScale(1.0);
    
    m_frameBuffer = m_scene.CreateFramebuffer(320, 240);
}

void Renderer::updateCoordinateSystems(const CoordinateSystems::Instances& coordSys)
{
    m_coordSys = coordSys;
    
    CoordinateSystems::Body::Coordinate copterPositionBody(0.0, 0.0, 0.0);
    Maybe<CoordinateSystems::NorthEastDown::Coordinate> copterPositionNED;
    coordSys.transform(copterPositionBody,copterPositionNED);
    if(copterPositionNED)
    {
        iffrenderer::Vector3<double> position(
            copterPositionNED->vec[0],
            copterPositionNED->vec[1],
            copterPositionNED->vec[2]
        );

        CoordinateSystems::Camera::Coordinate directionCamera(0.0, 0.0, 1.0);
        CoordinateSystems::Camera::Coordinate worldUpCamera(-1.0, 0.0, 0.0);
        Maybe<CoordinateSystems::NorthEastDown::Coordinate> directionNED;
        Maybe<CoordinateSystems::NorthEastDown::Coordinate> worldUpNED;

        coordSys.transformVector_<CoordinateSystems::Camera::Coordinate, CoordinateSystems::NorthEastDown::Coordinate>(
            directionCamera,
            directionNED
        );
        coordSys.transformVector_<CoordinateSystems::Camera::Coordinate, CoordinateSystems::NorthEastDown::Coordinate>(
            worldUpCamera, 
            worldUpNED
        );

        if(directionNED && worldUpNED)
        {

            iffrenderer::Vector3<double> direction(
                directionNED->vec[0],
                directionNED->vec[1],
                directionNED->vec[2]
            );
            iffrenderer::Vector3<double> worldUp(
                worldUpNED->vec[0],
                worldUpNED->vec[1],
                worldUpNED->vec[2]
            );
            
            m_camera->SetViewLookAt(position, position+direction, worldUp);

        }
    }


    if(coordSys.m_rotorBladeNED)
    {
        m_windTurbineNode->SetPosition(
            coordSys.m_rotorBladeNED->m_ned_rotorblade.m_translation[0],
            coordSys.m_rotorBladeNED->m_ned_rotorblade.m_translation[1],
            coordSys.m_rotorBladeNED->m_ned_rotorblade.m_translation[2]
        );
        CoordinateSystems::RollPitchYaw rpy(coordSys.m_rotorBladeNED->m_ned_rotorblade.m_rotation);
        m_windTurbineNode->SetRotation(
            rpy.m_roll,
            rpy.m_pitch,
            rpy.m_yaw
        );
        // todo only when changed
    }



}


void Renderer::render()
{
    m_scene.RenderCameraView();
    m_scene.RenderCameraView(m_frameBuffer);
}

bool Renderer::isKeyPressed(iffrenderer::Key key)
{
    m_window->PollWindowEvents();
    return m_window->IsKeyPressed(key);
}

void Renderer::saveColorMatrix(const WorldModel::Data::ColorMatrix& colorMatrix)
{
    cv::imwrite("colorMatrix.jpg", colorMatrix.m_color);
}

void Renderer::saveDepthMat(const cv::Mat_<float>& depthMat)
{
    cv::imwrite("depth.jpg", depthMat*255);
}

WorldModel::Data::ColorMatrix Renderer::getColorMatrix()
{
    WorldModel::Data::ColorMatrix colorMatrix;
    colorMatrix.m_color = m_scene.GetColorBuffer8UC3(m_frameBuffer, true);

    // saveColorMatrix(colorMatrix);

    return colorMatrix;
}

WorldModel::Data::PointCloudMatrix Renderer::getPointCloundMatrix()
{
    WorldModel::Data::PointCloudMatrix pointCloudMatrix;
    cv::Mat_<float> depthMat = m_scene.GetDepthMatrix32F(m_frameBuffer, true);
    float vertical_fov = m_camera->GetVerticalFOVinRadian();
    pointCloudMatrix.m_pcl = convertDepthMatToPointCloudMat(vertical_fov, depthMat);

    // saveDepthMat(depthMat);

    return pointCloudMatrix;
}

cv::Mat_<cv::Vec3f> Renderer::convertDistanceMatToPointCloudMat(float horizontal_fov, cv::Mat_<float> depthMat) const
{
    // coordinate system:
    // - camera is in origin
    // - z points from camera away
    // - x is parallel to matrix columns (i.e. along y-axis); matrix is centered around 0
    // - y is parallel to matrix rows (i.e. along x-axis); matrix is centered around 0
    // - this x,y swap is necessary for the resulting coordinate system to be
    //   right handed
    int w = depthMat.cols;
    int h = depthMat.rows;
    cv::Mat_<cv::Vec3f> pclMat(h,w);

    float inv_tan_half_fov=1./tan(horizontal_fov/2);

    for(int j=0; j<h; ++j)
    {
        float* depthMatRow = depthMat[j];
        cv::Vec3f* pclMatRow = pclMat[j];
        for(int i=0; i<w; ++i)
        {
            cv::Vec3f direction;
            // compute direction by computing the intersection of the ray for
            // this pixel with an image plane that is instantiated in a distance
            // where the width and height of the image plane is equal to the pixel
            // width and height of the depthMat
            // tan(horizontal_fov/2) = (w/2)/direction[2]
            direction[2] = (w/2)/inv_tan_half_fov;
            // x and y of the intersection point can now be computed by
            // offsetting them, to center it around 0. note: x,y get swapped
            direction[1] = i-(w-1)/2;
            direction[0] = j-(h-1)/2;

            pclMatRow[i] = direction * depthMatRow[i] / cv::norm(direction);
        }
    }

    return pclMat;
}

cv::Mat_<cv::Vec3f> Renderer::convertDepthMatToPointCloudMat(float vertical_fov, cv::Mat_<float> depthMat) const
{
    // coordinate system:
    // - camera is in origin
    // - z points from camera away
    // - x is parallel to matrix columns (i.e. along matrix y-axis); matrix is centered around 0
    // - y is parallel to matrix rows (i.e. along matrix x-axis); matrix is centered around 0
    // - this x,y swap is necessary for the resulting coordinate system to be
    //   right handed
    int w = depthMat.cols;
    int h = depthMat.rows;
    cv::Mat_<cv::Vec3f> pclMat(h,w);


    float tan_half_fov=tan(vertical_fov/2);
    float aspect_ratio=(float)w/(float)h;


    for(int j=0; j<h; ++j)
    {
        float* depthMatRow = depthMat[j];
        cv::Vec3f* pclMatRow = pclMat[j];
        for(int i=0; i<w; ++i)
        {

            // Camera::m_FOV is fov of y
            // 
            // depthMatRow[i] contains z coordinate ranging from nearPlane until farPlane
            float image_plane_half_w = depthMatRow[i]*aspect_ratio*tan_half_fov;
            float image_plane_half_h = depthMatRow[i]*tan_half_fov;
            float centeredI = (i-(w-1)/2.0f);
            float centeredJ = (j-(h-1)/2.0f);
            // not the swap of x and y by assigning i to [1] and j to [0]
            pclMatRow[i][1] = centeredI*image_plane_half_w/(w/2);
            pclMatRow[i][0] = centeredJ*image_plane_half_h/(h/2);
            pclMatRow[i][2] = depthMatRow[i];
        }
    }


    return pclMat;
}

} // namespace ar
