#include <math/sr_math.hpp>
#include <renderer/sr_camera.hpp>
#include <cmath>
#include <stdio.h>

camera::camera()
    : _position(0.0f, 0.0f, 0.0f), _rotation(0.0f, 0.0f, 0.0f), _fov(90.0f), _aspectRatio(4.0f / 3.0f), _nearPlane(0.1f), _farPlane(100.0f)
{
    recalculateViewMatrix();
    recalculateProjectionMatrix();
}

camera::camera(vec3 position, vec3 rotation, float fov, float aspectRatio, float nearPlane, float farPlane)
    : _position(position), _rotation(rotation), _fov(fov), _aspectRatio(aspectRatio), _nearPlane(nearPlane), _farPlane(farPlane)
{
    recalculateViewMatrix();
    recalculateProjectionMatrix();
}

void camera::recalculateRotationMatrix() const
{
    _rotationMatrix = rotationMatrix(-_rotation.x, -_rotation.y, -_rotation.z);
    _rotationMatrixDirty = false;
}

void camera::recalculateViewMatrix() const
{
    //_rotationMatrix = rotationMatrix(_rotation.x,_rotation.y, _rotation.z);
    mat4 cam_translation = translationMatrix(-_position);
    _viewMatrix = transpose(this->rotation()) * cam_translation;
    _viewDirty = false;
}

void camera::recalculateProjectionMatrix() const
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

const mat4 &camera::view() const
{
    if (_viewDirty)
        recalculateViewMatrix();
    return _viewMatrix;
}

const mat4 &camera::projection() const
{
    if (_projectionDirty)
        recalculateProjectionMatrix();
    return _projectionMatrix;
}

const mat4 &camera::rotation() const
{
    if (_rotationMatrixDirty)
        recalculateRotationMatrix();
    return _rotationMatrix;
}

const vec3 camera::lookVector() const
{
    mat4 cam_rt = this->rotation();
    vec4 lv = cam_rt * vec3(0, 0, -1);
    return vec3(lv.x, lv.y, lv.z);
}

const mat4 camera::worldMatrix() const
{
    mat4 t = translationMatrix(_position);
    mat4 r = this->rotation();
    return t * r;
}

// setters controlados que marcan dirty
void camera::setPosition(const vec3 &p)
{
    _position = p;
    _viewDirty = true;
}
void camera::setRotation(const vec3 &r)
{
    _rotation = r;
    _viewDirty = true;
    _rotationMatrixDirty = true;
}
void camera::setFov(float f)
{
    _fov = f;
    _projectionDirty = true;
}

void camera::lookAt(const vec3 &target, const vec3 &c_up)
{
    vec3 forward = target - _position;
    forward = normalize(forward);
    //printf("LookAt forward (pre-normalize): %f, %f, %f\n", forward.x, forward.y, forward.z);
    vec3 right = normalize(cross(forward, c_up));
    vec3 up = cross(right, forward);

    _rotationMatrix = mat4(
        right.x,    right.y,    right.z,    0.0f,
        up.x,       up.y,       up.z,       0.0f,
        -forward.x, -forward.y, -forward.z, 0.0f,
        0.0f,       0.0f,       0.0f,       1.0f);
    _rotation = getEulerAngles(_rotationMatrix);
    _rotationMatrixDirty = false;
    _viewDirty = true;
}