#pragma once

#include <math/sr_math.hpp>
#include <cstdint>

struct camera
{
    vec3 _position;
    vec3 _rotation;
    float _fov;
    float _aspectRatio;
    float _nearPlane;
    float _farPlane;
    mutable mat4 _viewMatrix;
    mutable mat4 _projectionMatrix;
    mutable bool _viewDirty = true;
    mutable bool _projectionDirty = true;

    void recalculateViewMatrix() const
    {
        mat4 cam_rm = rotationMatrix(_rotation.x,_rotation.y, _rotation.z);
        mat4 cam_tm = translationMatrix(vec3(-_position.x, -_position.y, -_position.z));
        _viewMatrix = cam_rm * cam_tm;
        _viewDirty = false;
    }

    void recalculateProjectionMatrix() const
    {
        float a = tanf(to_radians(_fov) * 0.5f);
        float b = a * _aspectRatio;
        float c = (_farPlane + _nearPlane) / (_farPlane - _nearPlane);
        float d = -(2.0f * _farPlane * _nearPlane) / (_farPlane - _nearPlane);
        _projectionMatrix = mat4(
            1 / b, 0.0f, 0.0f, 0.0f,
            0.0f, 1 / a, 0.0f, 0.0f,
            0.0f, 0.0f, -c, -1,
            0.0f, 0.0f, d, 0.0f);
        _projectionDirty = false;
    }
    camera() 
        : _position(0.0f, 0.0f, 0.0f), _rotation(0.0f, 0.0f, 0.0f), _fov(90.0f), _aspectRatio(4.0f/3.0f), _nearPlane(0.1f), _farPlane(100.0f)
    {
        recalculateViewMatrix();
        recalculateProjectionMatrix();
    }

    camera(vec3 position, vec3 rotation, float fov, float aspectRatio, float nearPlane, float farPlane)
        : _position(position), _rotation(rotation), _fov(fov), _aspectRatio(aspectRatio), _nearPlane(nearPlane), _farPlane(farPlane)
    {
        recalculateViewMatrix();
        recalculateProjectionMatrix();
    }

    const mat4 &view() const
    {
        if (_viewDirty)
            recalculateViewMatrix();
        return _viewMatrix;
    }

    const mat4 &projection() const
    {
        if (_projectionDirty)
            recalculateProjectionMatrix();
        return _projectionMatrix;
    }

    // setters controlados que marcan dirty
    void setPosition(const vec3 &p)
    {
        _position = p;
        _viewDirty = true;
    }
    void setRotation(const vec3 &r)
    {
        _rotation = r;
        _viewDirty = true;
    }
    void setFov(float f)
    {
        _fov = f;
        _projectionDirty = true;
    }
};