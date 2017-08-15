#pragma once

#include "geometry.hpp"

struct Transform {
	Transform(rt::Vec3 translation, rt::Quat rotation, rt::Vec3 scale)
		:_translation(translation)
		, _rotation(rotation)
		, _scale(scale) {

	}
	rt::Vec3 operator*(const rt::Vec3 &p) const {
		return _rotation * (_scale * p) + _translation;
	}
	void apply(std::vector<rt::Vec3> &vertices) const {
		for (int i = 0; i < vertices.size(); ++i) {
			vertices[i] = (*this) * vertices[i];
		}
	}
private:
	rt::Vec3 _translation;
	rt::Quat _rotation;
	rt::Vec3 _scale;
};
