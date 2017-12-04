#pragma once

#include <iffrenderer/iffrenderer.h>
#include "CoordinateSystems/NorthEastDown.h"
#include "CoordinateSystems/Instances.h"
#include "WorldModel/Data.h"

namespace ar {

class Renderer
{
public:
	Renderer(const CoordinateSystems::Instances& coordSys);

	void updateCoordinateSystems(const CoordinateSystems::Instances& coordSys);
	void render();

	WorldModel::Data::ColorMatrix getColorMatrix();
	WorldModel::Data::PointCloudMatrix getPointCloundMatrix();
	bool isKeyPressed(iffrenderer::Key);
protected:
	void saveColorMatrix(const WorldModel::Data::ColorMatrix& colorMatrix);
	void saveDepthMat(const cv::Mat_<float>& depthMat);
	cv::Mat_<cv::Vec3f> convertDistanceMatToPointCloudMat(float fov_x, cv::Mat_<float> depthMat) const;
	cv::Mat_<cv::Vec3f> convertDepthMatToPointCloudMat(float fov_x, cv::Mat_<float> depthMat) const;

	//iffrenderer::GraphicsWindowPtr m_window;
	iffrenderer::FramebufferPtr m_frameBuffer;
	iffrenderer::EntityPtr m_windTurbine;
	iffrenderer::SceneNode* m_windTurbineNode;
	iffrenderer::CameraPtr m_camera;
	iffrenderer::SceneManager& m_scene;

	CoordinateSystems::Instances m_coordSys;
};

} // namespace ar
