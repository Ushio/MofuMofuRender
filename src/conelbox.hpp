#pragma once

#include "scene.hpp"

inline std::shared_ptr<rt::Scene> scene_conelbox() {
	rt::CameraSetting cameraSetting;
	cameraSetting._imageHeight = 256;
	cameraSetting._imageWidth = 256;
	//cameraSetting._imageHeight = 512;
	//cameraSetting._imageWidth = 512;
	cameraSetting._eye.z = 3.4;
	//cameraSetting._eye.x = 1.0;
	//cameraSetting._eye.y = 0.1;

	std::vector<std::shared_ptr<rt::SceneElement>> sceneElements;
	// back
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> front(new rt::PolygonSceneElement(
			{
				rt::Vec3(-size, size, -1.0),
				rt::Vec3(size, size, -1.0),
				rt::Vec3(-size, -size, -1.0),
				rt::Vec3(size, -size, -1.0)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(front);
	}
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> left(new rt::PolygonSceneElement(
			{
				rt::Vec3(-1.0, size, size),
				rt::Vec3(-1.0, size, -size),
				rt::Vec3(-1.0, -size, size),
				rt::Vec3(-1.0, -size, -size)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75, 0.25, 0.25)),
			false
		));
		sceneElements.push_back(left);
	}
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> right(new rt::PolygonSceneElement(
			{
				rt::Vec3(1.0, size, -size),
				rt::Vec3(1.0, size, size),
				rt::Vec3(1.0, -size, -size),
				rt::Vec3(1.0, -size, size)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.25, 0.25, 0.75)),
			false
		));
		sceneElements.push_back(right);
	}
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> floor(new rt::PolygonSceneElement(
			{
				rt::Vec3(-size, -1.0, -size),
				rt::Vec3(size, -1.0, -size),
				rt::Vec3(-size, -1.0, size),
				rt::Vec3(size, -1.0, size)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(floor);
	}
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> top(new rt::PolygonSceneElement(
			{
				rt::Vec3(size, 1.0, -size),
				rt::Vec3(-size, 1.0, -size),
				rt::Vec3(size, 1.0, size),
				rt::Vec3(-size, 1.0, size)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(top);
	}

	{
		double size = 0.7;
		std::shared_ptr<rt::PolygonSceneElement> top(new rt::PolygonSceneElement(
			{
				rt::Vec3(size, 0.99, -size),
				rt::Vec3(-size, 0.99, -size),
				rt::Vec3(size, 0.99, size),
				rt::Vec3(-size, 0.99, size)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(3.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(top);
	}
	return std::shared_ptr<rt::Scene>(new rt::Scene(cameraSetting, sceneElements));
}